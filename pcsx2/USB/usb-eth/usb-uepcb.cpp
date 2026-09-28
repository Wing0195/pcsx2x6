// SPDX-FileCopyrightText: 2002-2026 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+
//
// usb-uepcb 1.7.1B (Beta)
// Created by Wing0195
// Based on: UE PCB 1.6 release
//
// 1.7.1B beta:
//   - Fixes mixed same-PC + remote Auto UDP P2P endpoint discovery. Loopback Clients are
//     advertised as 127.0.0.1 only to local recipients and translated to the Host's
//     reachable TCP-side address for LAN/VPN recipients.
//   - User-facing defaults are now TCP first and Auto UDP P2P first. New 1.7.1B config
//     keys avoid misinterpreting numeric selections saved by 1.7B.
//   - Keeps TCP bootstrap/session discovery, synchronized peer-list ACK/ACTIVATE, automatic
//     per-host UDP port allocation, and the proven 1.6 UDP synchronization core.
//   - Direct Internet NAT hole punching is not implemented; Auto UDP P2P is intended for
//     same-PC, LAN, or mutually-routable VPN/Tailscale peers.
//
// 1.6 release:
//   - Promotes Fast Attack + Slow/Smooth Recovery and Sync Hold to the stable UDP baseline.
//   - Default UDP tuning: Grace 4 ms, Decay 250 ms, Target 1-8, Queue 8, Sync Hold 30 ms.
//   - Removes UDP Retransmit/NACK recovery after testing showed it could increase stalls on Wi-Fi.
//   - Keeps per-peer IP + Port routing for reliable same-PC and cross-machine multi-instance play.
//   - Adaptive Playout and Global Stall Guard remain optional experimental Advanced settings.
//
// 1.5 changes:
//   - Adds selectable UDP / TCP transport while preserving the 1.4 UDP core.
//   - TCP uses explicit Host/Client hub topology with 4-byte BE frame length framing.
//   - TCP RX uses the fixed AN986 BULK IN wrapper (odd Ethernet lengths get 1-byte pad).
//   - Adds Advanced Settings gate for UDP jitter tuning.
//   - Keeps shared Saved IP history for UDP peers and TCP Host IP.
//
// 1.4 changes:
//   - Polished compact two-column UEPCB settings UI (paired ControllerBindingWidget file).
//   - History remains dropdown-based; History1..History10 stay hidden backend storage.
//   - Description text now follows the active Qt theme for high contrast.
//   - Expanded help text without changing synchronization/network behavior.
//   - Keeps the 1.3.1 shared 10-slot Peer IP history pool.
//   - Remember/Clear are native Boolean checkbox settings.
//   - Grace/Decay/MinTarget/MaxTarget/MaxQueue are user-adjustable settings.
//   - Defaults reproduce Claude 1.3/1.3.1 synchronization timing.
//   - Recovered diagnostic now counts ahead-arriving packets later promoted in order.
// Based on: usb-uepcb_fixed_udp_1_2_dynamic_buildfix.cpp
//
// Change from 1.2: the adaptive jitter buffer's grow/decay logic was
// asymmetric in a way that real-world Wi-Fi test logs showed pins
// target_packets at its maximum within the first ~10-40 seconds of a
// match and keeps it there for 93-98% of the session, even on a run
// where one side was on wired Ethernet. Root cause: jitter_insert() bumped
// target_packets the instant ANY packet arrived out of the expected order
// - including harmless, self-resolving 1-packet swaps, which measured
// logs showed recurring every ~4-5 seconds on average - while recovery
// required 120 consecutive perfectly-ordered deliveries (~6-7s at the
// observed packet rate), a bar that was reset to zero by the very same
// harmless events and so was essentially never reached.
//
// Fix in 1.3:
//   1) target_packets now only grows in jitter_promote(), at the moment a
//      missing packet's grace period genuinely expires (i.e. growing the
//      buffer would actually have helped) - not the instant a reorder is
//      first observed in jitter_insert().
//   2) Decay is time-based (configured interval since the last forced
//      skip) instead of a consecutive-success counter, so one isolated
//      hiccup no longer wipes out an otherwise-clean multi-minute streak.

#include "USB/qemu-usb/qusb.h"
#include "USB/qemu-usb/desc.h"
#include "USB/qemu-usb/USBinternal.h"
#include "USB/usb-eth/usb-uepcb.h"
#include "USB/USB.h"
#include "common/Console.h"
#include "common/SettingsInterface.h"
#include "StateWrapper.h"
#include "IopMem.h"

#include <cstring>
#include <cstdio>
#include <vector>
#include <atomic>
#include <chrono>
#include <deque>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <algorithm>
#include <map>
#include <array>
#include <unordered_map>
#include <utility>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#include <shellapi.h>
#pragma comment(lib, "ws2_32.lib")
#pragma comment(lib, "shell32.lib")
using socket_t = SOCKET;
#define UEPCB_INVALID_SOCKET INVALID_SOCKET
#else
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
using socket_t = int;
#define UEPCB_INVALID_SOCKET (-1)
#endif

namespace usb_uepcb
{
	static const u8 uepcb_dev_descriptor[] = {
		0x12, 0x01, 0x10, 0x01, 0xFF, 0x00, 0x00, 0x40,
		0x9A, 0x0B, 0x00, 0x05, 0x01, 0x02, 0x01, 0x02, 0x03, 0x01};

	static const u8 uepcb_config_descriptor[] = {
		0x09, 0x02, 0x27, 0x00, 0x01, 0x01, 0x00, 0xC0, 0x32,
		0x09, 0x04, 0x00, 0x00, 0x03, 0xFF, 0x00, 0x00, 0x00,
		0x07, 0x05, 0x81, 0x02, 0x40, 0x00, 0x00,
		0x07, 0x05, 0x02, 0x02, 0x40, 0x00, 0x00,
		0x07, 0x05, 0x83, 0x03, 0x08, 0x00, 0x0A};

	static const char* uepcb_strings[] = {
		"", "Namco", "UE PCB v1.7B (TCP/UDP Auto P2P)", ""};

	// Trailer appended to every UDP wire packet, *after* the raw Ethernet
	// frame. This is purely an emulator-side addition (real UE PCB hardware
	// never sees this - it only exists between PCSX2 instances), so it is
	// safe to change the wire format freely. It lets us detect drops /
	// reordering per-peer without touching the AN986 driver protocol at all.
	static constexpr int kWireTrailerSize = sizeof(u32);


	typedef struct UePcbState
	{
		USBDevice dev{};
		USBDesc desc{};
		USBDescDevice desc_dev{};

		u8 an986_regs[0x40] = {};
		u8 eeprom_word = 0;
		std::vector<u8> tx_accum;

		// UDP receive thread writes to pending_rx. The USB IN handler promotes
		// a small bounded batch at the USB polling boundary. This keeps network
		// scheduling from directly changing the game-visible RX queue timing.
		std::deque<std::vector<u8>> pending_rx;
		std::mutex pending_lock;

		std::vector<u8> rx_partial;
		size_t rx_offset = 0;

		u8 mac[6] = {0x00, 0x90, 0x2E, 0x11, 0x22, 0x33};
		u32 config_port = 0;
		int udp_port = 7500;
		std::string broadcast_ip;
		std::array<std::string, 3> peer_ips{};
		// 1.6: every UDP peer is a full endpoint (IP + port). This lets
		// multiple PCSX2 instances share one host IP without fighting over
		// the same destination port.
		std::array<int, 3> peer_ports{{7500, 7500, 7500}};
		u8 mii_phyaddr = 1;
		u8 mii_reg = 1;
		bool init_done = false;
		bool link_event_sent = false;

		std::mutex in_lock;
		std::deque<std::vector<u8>> in_q;

		// Outgoing sequence counter, one per this instance. Peers use it to
		// detect gaps in what they receive *from us*.
		std::atomic<u32> tx_seq{0};

		// Per-peer bounded adaptive jitter buffers. Sequence numbers are used
		// to restore order within each sender stream. The buffer is deliberately
		// small so network trouble cannot turn into seconds of game latency.
		struct JitterPacket
		{
			u32 seq = 0;
			std::vector<u8> data;
			std::chrono::steady_clock::time_point arrival{};
			// Diagnostic only: packet arrived ahead of the then-current playback point.
			bool diag_was_ahead = false;
			// Diagnostic: records when experimental Adaptive Playout actually held a
			// packet beyond the legacy readiness point. It never changes timing.
			bool diag_playout_held = false;
			std::chrono::steady_clock::time_point diag_playout_hold_since{};
		};

		struct PeerJitter
		{
			std::map<u32, JitterPacket> packets;
			u32 next_seq = 0;
			bool seq_valid = false;
			u32 target_packets = 1;

			// 1.3.3 diagnostics only. These counters never affect playback timing.
			u64 diag_rx = 0;
			u64 diag_ahead = 0;
			u64 diag_recovered = 0;
			u64 diag_forced_skip = 0;
			u64 diag_late = 0;
			u64 diag_duplicate = 0;
			u32 diag_max_ahead = 0;
			// Claude UePcb 1.3: replaces the old "stable_deliveries" counter.
			// Decay is now time-based (see jitter_decay_interval) so a single
			// isolated reorder can't zero out several seconds of otherwise
			// clean delivery and re-arm a long climb back to the max.
			std::chrono::steady_clock::time_point last_target_change{};
			bool last_target_change_valid = false;
			std::chrono::steady_clock::time_point gap_since{};
			bool gap_active = false;

			// Hold / gap diagnostics.
			bool gap_hold_counted = false;
			u64 diag_hold_events = 0;
			u64 diag_max_gap_ms = 0;
			// 1.6 Fast Attack diagnostics. Count only real target jumps caused
			// by gap pressure, not ordinary forced-skip fallback increments.
			u64 diag_fast_attack = 0;
			u32 diag_peak_target = 1;
	};

		std::unordered_map<u64, PeerJitter> peer_jitter;
		std::mutex jitter_lock;
		u32 loss_log_suppress = 0;

		// 1.3.3 diagnostic: forced skips on different peers in a very short
		// window are more suggestive of a local emulator/USB scheduling stall
		// than independent network loss. This is logging only.
		std::chrono::steady_clock::time_point diag_last_forced_time{};
		u64 diag_last_forced_peer = 0;
		bool diag_last_forced_valid = false;

		// User-tunable adaptive jitter parameters. 1.6 defaults are tuned
		// for Fast Attack + Slow/Smooth Recovery: ordinary short jitter is
		// ignored, real gaps can raise the target quickly, then recovery drops
		// one level every 250 ms (8 -> 1 in about 1.75 seconds).
		size_t jitter_max_packets = 8;
		u32 jitter_min_target = 1;
		u32 jitter_max_target = 8;
		std::chrono::milliseconds jitter_grace{4};
		std::chrono::milliseconds jitter_decay_interval{250};

		// 1.6 advanced synchronization controls. Sync Hold is the recommended
		// default; experimental controls remain opt-in for testing.
		bool beta_sync_hold = false;
		std::chrono::milliseconds beta_sync_hold_time{30};
		bool beta_adaptive_playout = false;
		std::chrono::milliseconds beta_playout_max{3};
		bool beta_global_stall_guard = false;

		// Transport selection: 0 = UDP, 1 = TCP.
		int connection_mode = 0;
		bool advanced_settings = false;

		// UDP transport.
		socket_t udp_sock = UEPCB_INVALID_SOCKET;

		u64 diag_playout_events = 0;
		u64 diag_playout_total_hold_ms = 0;
		u64 diag_playout_max_hold_ms = 0;

		// TCP transport. Host is an N-way hub; Client keeps one connection to Host.
		bool tcp_is_host = true;
		int tcp_port = 7500;
		std::string tcp_host_ip = "127.0.0.1";
		std::string tcp_bind_ip = "0.0.0.0";
		// Internal TCP data-mode semantics remain: 0 = stay TCP, 1 = auto UDP P2P.
		// Auto mode uses TCP as a bootstrap session and switches all currently-connected
		// peers to an auto-discovered UDP P2P mesh after peer-list/ACK/ACTIVATE.
		int tcp_data_mode = 1;
		std::mutex tcp_sock_lock;
		socket_t tcp_listen_sock = UEPCB_INVALID_SOCKET;
		socket_t tcp_connect_sock = UEPCB_INVALID_SOCKET;

		struct TcpPeerMeta
		{
			sockaddr_in remote_addr{};
			bool hello_received = false;
			u16 udp_port = 0;
			std::array<u8, 6> mac{};
			u32 ack_generation = 0;
		};

		std::mutex tcp_peers_lock;
		std::vector<socket_t> tcp_peers;
		std::unordered_map<socket_t, TcpPeerMeta> tcp_peer_meta;

		// Auto UDP P2P state. The TCP connection remains alive as the control
		// channel after activation, while Ethernet frames move to these endpoints.
		std::mutex auto_udp_lock;
		std::vector<sockaddr_in> auto_udp_peers;
		std::atomic<bool> auto_udp_ready{false};
		u32 auto_session_generation = 0;
		u32 auto_client_generation = 0;

		std::thread recv_thread;
		std::thread udp_recv_thread;
		std::atomic<bool> thread_stop{false};
	} UePcbState;

	static void wsa_ensure()
	{
#ifdef _WIN32
		static bool init = []() {
			WSADATA w;
			WSAStartup(MAKEWORD(2, 2), &w);
			return true;
		}();
		(void)init;
#endif
	}

	static void sock_close(socket_t x)
	{
#ifdef _WIN32
		closesocket(x);
#else
		close(x);
#endif
	}

	static u16 mii_val(u8 reg)
	{
		switch (reg)
		{
			case 0x00: return 0x3100;
			case 0x01: return 0x786d;
			case 0x02: return 0x001d;
			case 0x03: return 0x2411;
			case 0x04: return 0x05e1;
			case 0x05: return 0x0001;
			default: return 0x0000;
		}
	}

	static void an986_read_local(UePcbState* s, int reg, int length, u8* data)
	{
		u8 tmp[8] = {};
		int n = 0;
		if (reg == 0x2B && s->init_done && !s->link_event_sent)
		{
			tmp[0] = 0x18; tmp[1] = 0x01; tmp[3] = 0x60; n = 5;
			s->link_event_sent = true;
		}
		else if (reg == 0x25)
		{
			const u16 val = (s->mii_phyaddr == 1) ? mii_val(s->mii_reg) : 0xFFFF;
			tmp[0] = s->mii_phyaddr;
			tmp[1] = val & 0xFF;
			tmp[2] = (val >> 8) & 0xFF;
			tmp[3] = 0x80 | (s->mii_reg & 0x1F);
			n = 4;
		}
		else if (reg == 0x28)
		{
			tmp[0] = 0x80 | (s->mii_reg & 0x1F);
			n = 2;
		}
		else if (reg == 0x2B)
		{
			tmp[3] = 0x60; n = 5;
		}
		else if (reg == 0x23)
		{
			tmp[0] = 0x04; n = 2;
		}
		else if (reg == 0x21)
		{
			tmp[2] = 0x04; n = 3;
			if (s->eeprom_word < 3)
			{
				tmp[0] = s->mac[s->eeprom_word * 2];
				tmp[1] = s->mac[s->eeprom_word * 2 + 1];
			}
		}
		else if (reg == 0x10)
		{
			std::memcpy(tmp, s->mac, 6); n = 6;
		}
		else if (reg == 0x01)
		{
			n = 2;
		}
		else
		{
			n = length;
		}
		std::memset(data, 0, length);
		const int copy = std::min<int>(std::min<int>(length, n), (int)sizeof(tmp));
		if (copy > 0)
			std::memcpy(data, tmp, copy);
	}

	static std::vector<u8> to_bulkin(const u8* eth, int len)
	{
		std::vector<u8> v;
		const int pad = len & 1;
		const int p = len + pad + 8;
		v.reserve(2 + p);
		v.push_back(static_cast<u8>(p & 0xFF));
		v.push_back(static_cast<u8>((p >> 8) & 0xFF));
		v.insert(v.end(), eth, eth + len);
		if (pad)
			v.push_back(0);
		v.insert(v.end(), 8, 0);
		return v;
	}

	static u64 mac_key(const u8* mac)
	{
		u64 k = 0;
		for (int i = 0; i < 6; i++)
			k = (k << 8) | mac[i];
		return k;
	}

	// Adaptive jitter parameters are stored per UePcbState in 1.3.3 so they
	// can be tuned from the USB settings UI without recompiling. 1.6 defaults
	// are Grace=4ms, Decay=250ms, MinTarget=1, MaxTarget=8, Queue=8.

	static bool seq_before(u32 a, u32 b)
	{
		// Serial-number arithmetic handles uint32 wraparound.
		return static_cast<s32>(a - b) < 0;
	}

	static void jitter_insert(UePcbState* s, u64 peer_key, u32 seq, std::vector<u8>&& data)
	{
		const auto now = std::chrono::steady_clock::now();
		std::lock_guard<std::mutex> lock(s->jitter_lock);

		auto& jb = s->peer_jitter[peer_key];
		++jb.diag_rx;

		if (!jb.seq_valid)
		{
			jb.next_seq = seq;
			jb.target_packets = s->jitter_min_target;
			jb.diag_peak_target = s->jitter_min_target;
			jb.seq_valid = true;
		}

		// Packet arrived after the playback point or is a duplicate.
		if (seq_before(seq, jb.next_seq))
		{
			++jb.diag_late;
			return;
		}

		if (jb.packets.find(seq) != jb.packets.end())
		{
			++jb.diag_duplicate;
			return;
		}

		const bool diag_was_ahead = (seq != jb.next_seq);
		if (diag_was_ahead)
		{
			++jb.diag_ahead;
			const u32 ahead = seq - jb.next_seq;
			if (ahead > jb.diag_max_ahead)
				jb.diag_max_ahead = ahead;
		}

		// Claude UePcb 1.3: target_packets is deliberately NOT touched here
		// anymore. A packet simply arriving out of order (seq != jb.next_seq)
		// is the ordinary case of two adjacent UDP datagrams swapping order
		// by a couple of milliseconds - real Wi-Fi links do this constantly,
		// and it self-resolves almost immediately. In 1.2 this bumped the
		// buffer target on every such event, which measured test logs showed
		// pinned target_packets at its max within the first ~10-40 seconds
		// of every match and kept it there almost the whole game. Growing
		// the buffer only happens in jitter_promote() now, at the point a
		// gap's grace period genuinely expires - i.e. only when growing the
		// buffer would actually have helped. gap_active/gap_since are also
		// tracked solely in jitter_promote() now (single source of truth
		// for that countdown), so they're not touched here either.

		// Never allow an unbounded backlog.
		if (jb.packets.size() >= s->jitter_max_packets)
		{
			auto farthest = std::prev(jb.packets.end());

			// Keep packets closest to the playback point.
			if (seq_before(farthest->first, seq))
				return;

			jb.packets.erase(farthest);
		}

		jb.packets.emplace(seq, UePcbState::JitterPacket{
			seq, std::move(data), now, diag_was_ahead});
	}


	static std::chrono::milliseconds beta_playout_delay_locked(const UePcbState* s)
	{
		if (!s->beta_adaptive_playout)
			return std::chrono::milliseconds(0);

		u32 highest_target = s->jitter_min_target;
		for (const auto& entry : s->peer_jitter)
		{
			if (entry.second.target_packets > highest_target)
				highest_target = entry.second.target_packets;
		}

		const u32 steps = (highest_target > s->jitter_min_target) ?
			(highest_target - s->jitter_min_target) : 0;
		long long delay_ms = static_cast<long long>(steps);
		if (delay_ms > s->beta_playout_max.count())
			delay_ms = s->beta_playout_max.count();
		return std::chrono::milliseconds(delay_ms);
	}

	static void jitter_promote(UePcbState* s)
	{
		std::lock_guard<std::mutex> jitter_lock(s->jitter_lock);
		std::lock_guard<std::mutex> pending_lock(s->pending_lock);

		while (s->pending_rx.size() < 64)
		{
			u64 selected_peer = 0;
			u32 selected_seq = 0;
			std::chrono::steady_clock::time_point selected_arrival{};
			bool selected = false;
			bool selected_is_forced_skip = false;

			const auto now = std::chrono::steady_clock::now();
			const auto beta_playout_delay = beta_playout_delay_locked(s);

			for (auto& entry : s->peer_jitter)
			{
				auto& jb = entry.second;
				if (jb.packets.empty())
					continue;

				auto expected = jb.packets.find(jb.next_seq);

				if (expected != jb.packets.end())
				{
					const auto age =
						std::chrono::duration_cast<std::chrono::milliseconds>(
							now - expected->second.arrival);

					// Legacy 1.5.1 readiness is preserved. The experimental time-based
					// playout option only adds a small common minimum age after
					// a real forced skip has raised a peer's target buffer.
					const bool legacy_ready =
						(jb.packets.size() >= jb.target_packets || age >= s->jitter_grace);
					const bool playout_ready =
						(!s->beta_adaptive_playout || age >= beta_playout_delay);

					// Diagnostic only: remember the first moment a packet was already
					// legacy-ready but Adaptive Playout intentionally kept it waiting.
					if (legacy_ready && !playout_ready && !expected->second.diag_playout_held)
					{
						expected->second.diag_playout_held = true;
						expected->second.diag_playout_hold_since = now;
					}

					if (legacy_ready && playout_ready)
					{
						if (!selected || expected->second.arrival < selected_arrival)
						{
							selected = true;
							selected_peer = entry.first;
							selected_seq = expected->first;
							selected_arrival = expected->second.arrival;
							selected_is_forced_skip = false;
						}
					}
				}
				else
				{
					// A sequence gap exists. 1.5.1 would skip when the packet
					// target filled or the short grace expired. Sync Hold beta
					// deliberately prefers synchronization: it ignores queue
					// depth and waits a bounded amount of real time instead.
					if (!jb.gap_active)
					{
						jb.gap_active = true;
						jb.gap_since = now;
						jb.gap_hold_counted = false;
					}

					const auto gap_age =
						std::chrono::duration_cast<std::chrono::milliseconds>(
							now - jb.gap_since);
					if (static_cast<u64>(gap_age.count()) > jb.diag_max_gap_ms)
						jb.diag_max_gap_ms = static_cast<u64>(gap_age.count());

					// 1.6 Fast Attack. Do not wait for a forced skip before the
					// adaptive target reacts. Once a real gap survives Jitter Grace,
					// combine gap age with the amount of already-arrived future data.
					// High RTT by itself never changes the target; only actual missing/
					// reordered delivery pressure does.
					if (gap_age >= s->jitter_grace && jb.target_packets < s->jitter_max_target)
					{
						u32 age_target = s->jitter_min_target;
						const long long age_ms = gap_age.count();
						if (age_ms >= 25)
							age_target = s->jitter_max_target;
						else if (age_ms >= 18)
							age_target = std::min<u32>(s->jitter_max_target, 6);
						else if (age_ms >= 12)
							age_target = std::min<u32>(s->jitter_max_target, 5);
						else if (age_ms >= 8)
							age_target = std::min<u32>(s->jitter_max_target, 3);
						else
							age_target = std::min<u32>(s->jitter_max_target, 2);

						const size_t ahead_depth = jb.packets.size();
						u32 depth_target = s->jitter_min_target;
						if (ahead_depth >= 8)
							depth_target = s->jitter_max_target;
						else if (ahead_depth >= 4)
							depth_target = std::min<u32>(s->jitter_max_target, 5);
						else if (ahead_depth >= 2)
							depth_target = std::min<u32>(s->jitter_max_target, 3);
						else if (ahead_depth >= 1)
							depth_target = std::min<u32>(s->jitter_max_target, 2);

						u32 desired_target = (age_target > depth_target) ? age_target : depth_target;
						desired_target = std::clamp(desired_target, s->jitter_min_target, s->jitter_max_target);
						if (desired_target > jb.target_packets)
						{
							jb.target_packets = desired_target;
							jb.last_target_change = now;
							jb.last_target_change_valid = true;
							++jb.diag_fast_attack;
							if (jb.target_packets > jb.diag_peak_target)
								jb.diag_peak_target = jb.target_packets;
							Console.WriteLn(
								"UePcb 1.6 Fast Attack: target buffer -> %u (peer key %llu, gap=%lldms, ahead=%zu)",
								jb.target_packets, static_cast<unsigned long long>(entry.first),
								static_cast<long long>(age_ms), ahead_depth);
						}
					}

					std::chrono::milliseconds hold_limit = s->beta_sync_hold_time;
					if (hold_limit < s->jitter_grace)
						hold_limit = s->jitter_grace;

					if (s->beta_sync_hold && gap_age >= s->jitter_grace &&
						gap_age < hold_limit && !jb.gap_hold_counted)
					{
						++jb.diag_hold_events;
						jb.gap_hold_counted = true;
					}

					bool gap_ready = false;
					if (s->beta_sync_hold)
						gap_ready = (gap_age >= hold_limit);
					else
						gap_ready =
							(jb.packets.size() >= jb.target_packets || gap_age >= s->jitter_grace);

					if (gap_ready)
					{
						auto first = jb.packets.begin();
						if (!selected || first->second.arrival < selected_arrival)
						{
							selected = true;
							selected_peer = entry.first;
							selected_seq = first->first;
							selected_arrival = first->second.arrival;
							selected_is_forced_skip = true;
						}
					}
				}
			}

			if (!selected)
				break;

			auto peer_it = s->peer_jitter.find(selected_peer);
			if (peer_it == s->peer_jitter.end())
				break;

			auto& jb = peer_it->second;
			auto packet_it = jb.packets.find(selected_seq);
			if (packet_it == jb.packets.end())
				break;

			if (!selected_is_forced_skip && packet_it->second.diag_was_ahead)
				++jb.diag_recovered;

			if (packet_it->second.diag_playout_held)
			{
				const auto held_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
					now - packet_it->second.diag_playout_hold_since);
				const u64 held = static_cast<u64>(std::max<long long>(0, held_ms.count()));
				++s->diag_playout_events;
				s->diag_playout_total_hold_ms += held;
				if (held > s->diag_playout_max_hold_ms)
					s->diag_playout_max_hold_ms = held;
			}

			s->pending_rx.push_back(std::move(packet_it->second.data));
			jb.packets.erase(packet_it);
			jb.next_seq = selected_seq + 1;
			jb.gap_active = false;
			jb.gap_hold_counted = false;

			if (!jb.last_target_change_valid)
			{
				jb.last_target_change = now;
				jb.last_target_change_valid = true;
			}

			if (selected_is_forced_skip)
			{
				++jb.diag_forced_skip;

				bool possible_local_stall = false;
				const u64 previous_forced_peer = s->diag_last_forced_peer;
				if (s->diag_last_forced_valid && previous_forced_peer != selected_peer)
				{
					const auto burst_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
						now - s->diag_last_forced_time);
					if (burst_ms.count() >= 0 && burst_ms.count() <= 100)
					{
						possible_local_stall = true;
						Console.WriteLn(
							"UePcb DIAG: possible local stall - forced skips on different peers within %lldms",
							static_cast<long long>(burst_ms.count()));
					}
				}

				// Experimental Global Stall Guard prevents a same-process scheduling hitch
				// from leaving several peer buffers artificially enlarged for
				// seconds. If the second peer confirms a <100ms multi-peer stall,
				// undo one step on the previous peer and don't grow this peer.
				if (possible_local_stall && s->beta_global_stall_guard)
				{
					auto previous_it = s->peer_jitter.find(previous_forced_peer);
					if (previous_it != s->peer_jitter.end() &&
						previous_it->second.target_packets > s->jitter_min_target)
					{
						--previous_it->second.target_packets;
						previous_it->second.last_target_change = now;
					}
					Console.WriteLn("UePcb EXP: Global Stall Guard suppressed multi-peer buffer growth");
				}
				else if (jb.target_packets < s->jitter_max_target)
				{
					++jb.target_packets;
					if (jb.target_packets > jb.diag_peak_target)
						jb.diag_peak_target = jb.target_packets;
					Console.WriteLn(
						"UePcb: target buffer -> %u (peer key %llu) - forced-skip fallback growth",
						jb.target_packets, static_cast<unsigned long long>(selected_peer));
				}

				s->diag_last_forced_time = now;
				s->diag_last_forced_peer = selected_peer;
				s->diag_last_forced_valid = true;
				jb.last_target_change = now;
			}
			else
			{
				const auto since_change =
					std::chrono::duration_cast<std::chrono::milliseconds>(now - jb.last_target_change);

				if (jb.target_packets > s->jitter_min_target &&
					since_change >= s->jitter_decay_interval)
				{
					--jb.target_packets;
					jb.last_target_change = now;
					Console.WriteLn(
						"UePcb: target buffer -> %u (peer key %llu) - stable for %lldms, relaxing",
						jb.target_packets, static_cast<unsigned long long>(selected_peer),
						static_cast<long long>(since_change.count()));
				}
			}
		}
	}

	static bool tcp_recv_exact(socket_t c, u8* buf, int len)
	{
		int off = 0;
		while (off < len)
		{
			const int n = recv(c, reinterpret_cast<char*>(buf + off), len - off, 0);
			if (n <= 0)
				return false;
			off += n;
		}
		return true;
	}

	static bool tcp_send_exact(socket_t c, const u8* buf, int len)
	{
		int off = 0;
		while (off < len)
		{
			const int n = send(c, reinterpret_cast<const char*>(buf + off), len - off, 0);
			if (n <= 0)
				return false;
			off += n;
		}
		return true;
	}

	static bool tcp_send_frame(socket_t c, const u8* eth, int len)
	{
		if (c == UEPCB_INVALID_SOCKET || len < 14 || len > 2048)
			return false;
		const u8 hdr[4] = {
			static_cast<u8>((len >> 24) & 0xff), static_cast<u8>((len >> 16) & 0xff),
			static_cast<u8>((len >> 8) & 0xff), static_cast<u8>(len & 0xff)};
		return tcp_send_exact(c, hdr, 4) && tcp_send_exact(c, eth, len);
	}

	static void tcp_push_received_frame(UePcbState* s, const u8* eth, int len)
	{
		// IMPORTANT: use the fixed AN986 BULK IN wrapper shared with UDP.
		// Odd Ethernet lengths are padded to an even boundary before the 8-byte
		// status trailer, and the 2-byte reported length includes that pad.
		std::lock_guard<std::mutex> lk(s->in_lock);
		if (s->in_q.size() >= 64)
			s->in_q.pop_front();
		s->in_q.push_back(to_bulkin(eth, len));
	}

	static constexpr u8 kAutoCtrlMagic[4] = {'U', 'P', '1', '7'};
	static constexpr u8 kAutoCtrlVersion = 1;
	enum : u8
	{
		AUTO_CTRL_HELLO = 1,
		AUTO_CTRL_PEER_LIST = 2,
		AUTO_CTRL_ACK = 3,
		AUTO_CTRL_ACTIVATE = 4,
	};

	static bool tcp_send_payload(socket_t c, const u8* data, int len)
	{
		if (c == UEPCB_INVALID_SOCKET || !data || len <= 0 || len > 4096)
			return false;
		const u8 hdr[4] = {
			static_cast<u8>((len >> 24) & 0xff), static_cast<u8>((len >> 16) & 0xff),
			static_cast<u8>((len >> 8) & 0xff), static_cast<u8>(len & 0xff)};
		return tcp_send_exact(c, hdr, 4) && tcp_send_exact(c, data, len);
	}

	static bool auto_is_control(const u8* data, int len)
	{
		return data && len >= 6 && std::memcmp(data, kAutoCtrlMagic, 4) == 0 && data[5] == kAutoCtrlVersion;
	}

	static void auto_put_u16(std::vector<u8>& out, u16 v)
	{
		out.push_back(static_cast<u8>((v >> 8) & 0xff));
		out.push_back(static_cast<u8>(v & 0xff));
	}

	static void auto_put_u32(std::vector<u8>& out, u32 v)
	{
		out.push_back(static_cast<u8>((v >> 24) & 0xff));
		out.push_back(static_cast<u8>((v >> 16) & 0xff));
		out.push_back(static_cast<u8>((v >> 8) & 0xff));
		out.push_back(static_cast<u8>(v & 0xff));
	}

	static u16 auto_get_u16(const u8* p)
	{
		return static_cast<u16>((static_cast<u16>(p[0]) << 8) | static_cast<u16>(p[1]));
	}

	static u32 auto_get_u32(const u8* p)
	{
		return (static_cast<u32>(p[0]) << 24) | (static_cast<u32>(p[1]) << 16) |
			(static_cast<u32>(p[2]) << 8) | static_cast<u32>(p[3]);
	}

	static void auto_clear_udp_peers(UePcbState* s)
	{
		std::lock_guard<std::mutex> lk(s->auto_udp_lock);
		s->auto_udp_peers.clear();
	}

	static void auto_set_udp_peers(UePcbState* s, const std::vector<sockaddr_in>& peers)
	{
		std::lock_guard<std::mutex> lk(s->auto_udp_lock);
		s->auto_udp_peers = peers;
	}

	static bool auto_send_hello(UePcbState* s, socket_t c)
	{
		if (s->udp_sock == UEPCB_INVALID_SOCKET || s->udp_port <= 0 || s->udp_port > 65535)
			return false;
		std::vector<u8> msg;
		msg.reserve(14);
		msg.insert(msg.end(), kAutoCtrlMagic, kAutoCtrlMagic + 4);
		msg.push_back(AUTO_CTRL_HELLO);
		msg.push_back(kAutoCtrlVersion);
		auto_put_u16(msg, static_cast<u16>(s->udp_port));
		msg.insert(msg.end(), s->mac, s->mac + 6);
		std::lock_guard<std::mutex> lk(s->tcp_peers_lock);
		return tcp_send_payload(c, msg.data(), static_cast<int>(msg.size()));
	}

	static bool auto_send_ack(UePcbState* s, socket_t c, u32 generation)
	{
		std::vector<u8> msg;
		msg.insert(msg.end(), kAutoCtrlMagic, kAutoCtrlMagic + 4);
		msg.push_back(AUTO_CTRL_ACK);
		msg.push_back(kAutoCtrlVersion);
		auto_put_u32(msg, generation);
		std::lock_guard<std::mutex> lk(s->tcp_peers_lock);
		return tcp_send_payload(c, msg.data(), static_cast<int>(msg.size()));
	}

	static bool auto_send_activate(UePcbState* s, socket_t c, u32 generation)
	{
		std::vector<u8> msg;
		msg.insert(msg.end(), kAutoCtrlMagic, kAutoCtrlMagic + 4);
		msg.push_back(AUTO_CTRL_ACTIVATE);
		msg.push_back(kAutoCtrlVersion);
		auto_put_u32(msg, generation);
		std::lock_guard<std::mutex> lk(s->tcp_peers_lock);
		return tcp_send_payload(c, msg.data(), static_cast<int>(msg.size()));
	}

	static bool auto_ipv4_is_loopback(const sockaddr_in& addr)
	{
		const u32 host_order = ntohl(addr.sin_addr.s_addr);
		return (host_order & 0xff000000u) == 0x7f000000u;
	}

	static void auto_host_rebuild_session(UePcbState* s)
	{
		if (!s->tcp_is_host || s->tcp_data_mode != 1 || s->udp_sock == UEPCB_INVALID_SOCKET)
			return;

		struct ReadyPeer
		{
			socket_t sock = UEPCB_INVALID_SOCKET;
			sockaddr_in remote{};
			u16 udp_port = 0;
			std::array<u8, 6> mac{};
		};

		std::vector<ReadyPeer> ready;
		u32 generation = 0;
		{
			std::lock_guard<std::mutex> lk(s->tcp_peers_lock);
			++s->auto_session_generation;
			if (s->auto_session_generation == 0)
				++s->auto_session_generation;
			generation = s->auto_session_generation;
			for (socket_t p : s->tcp_peers)
			{
				auto it = s->tcp_peer_meta.find(p);
				if (it == s->tcp_peer_meta.end() || !it->second.hello_received)
					continue;
				it->second.ack_generation = 0;
				ReadyPeer rp;
				rp.sock = p;
				rp.remote = it->second.remote_addr;
				rp.udp_port = it->second.udp_port;
				rp.mac = it->second.mac;
				ready.push_back(rp);
			}
		}

		s->auto_udp_ready = false;
		std::vector<sockaddr_in> host_peers;
		for (const ReadyPeer& rp : ready)
		{
			sockaddr_in ep = rp.remote;
			ep.sin_port = htons(rp.udp_port);
			host_peers.push_back(ep);
			char ipbuf[INET_ADDRSTRLEN] = {};
			if (inet_ntop(AF_INET, &ep.sin_addr, ipbuf, sizeof(ipbuf)))
				Console.WriteLn("UePcb 1.7.1B: discovered Client UDP endpoint %s:%u", ipbuf, static_cast<unsigned>(rp.udp_port));
		}
		auto_set_udp_peers(s, host_peers);

		for (const ReadyPeer& recipient : ready)
		{
			std::vector<std::pair<sockaddr_in, std::array<u8, 6>>> entries;

			// Resolve the Host address from the exact TCP path used by this recipient.
			// Same-PC clients see 127.0.0.1, while a remote LAN/VPN client sees the
			// Host interface address it actually connected to. The same address is also
			// used below to translate *other* same-PC Clients for remote recipients.
			sockaddr_in host_ep{};
			bool have_host_ep = false;
#ifdef _WIN32
			int host_len = sizeof(host_ep);
#else
			socklen_t host_len = sizeof(host_ep);
#endif
			if (getsockname(recipient.sock, reinterpret_cast<sockaddr*>(&host_ep), &host_len) == 0)
			{
				have_host_ep = true;
				host_ep.sin_port = htons(static_cast<u16>(s->udp_port));
				std::array<u8, 6> host_mac{};
				std::memcpy(host_mac.data(), s->mac, 6);
				entries.emplace_back(host_ep, host_mac);
			}

			for (const ReadyPeer& other : ready)
			{
				if (other.sock == recipient.sock)
					continue;

				sockaddr_in ep = other.remote;
				ep.sin_port = htons(other.udp_port);

				// Critical mixed local/remote fix: 127.0.0.1 is meaningful only on the
				// Host machine. If P3/P4 connected to P1 through loopback, never advertise
				// that loopback address to an iMac/remote recipient. Translate it to the
				// Host's reachable address on this recipient's TCP connection instead.
				if (auto_ipv4_is_loopback(ep) && !auto_ipv4_is_loopback(recipient.remote) && have_host_ep)
					ep.sin_addr = host_ep.sin_addr;

				entries.emplace_back(ep, other.mac);
				if (entries.size() >= 3)
					break;
			}

			std::vector<u8> msg;
			msg.reserve(12 + entries.size() * 12);
			msg.insert(msg.end(), kAutoCtrlMagic, kAutoCtrlMagic + 4);
			msg.push_back(AUTO_CTRL_PEER_LIST);
			msg.push_back(kAutoCtrlVersion);
			msg.push_back(static_cast<u8>(entries.size()));
			msg.push_back(0);
			auto_put_u32(msg, generation);
			for (const auto& entry : entries)
			{
				const u8* ip = reinterpret_cast<const u8*>(&entry.first.sin_addr.s_addr);
				msg.insert(msg.end(), ip, ip + 4);
				auto_put_u16(msg, ntohs(entry.first.sin_port));
				msg.insert(msg.end(), entry.second.begin(), entry.second.end());
			}
			{
				std::lock_guard<std::mutex> lk(s->tcp_peers_lock);
				tcp_send_payload(recipient.sock, msg.data(), static_cast<int>(msg.size()));
			}
		}

		Console.WriteLn("UePcb 1.7.1B: TCP bootstrap peer list generation %u sent (%d client%s, UDP local %d)",
			generation, static_cast<int>(ready.size()), ready.size() == 1 ? "" : "s", s->udp_port);
	}

	static void auto_host_try_activate(UePcbState* s, u32 generation)
	{
		std::vector<socket_t> ready_sockets;
		{
			std::lock_guard<std::mutex> lk(s->tcp_peers_lock);
			if (generation == 0 || generation != s->auto_session_generation)
				return;
			bool have_peer = false;
			for (socket_t p : s->tcp_peers)
			{
				auto it = s->tcp_peer_meta.find(p);
				// Do not activate a partial UDP mesh. If any connected Client is
				// running Stay TCP, has no UDP socket, or has not completed HELLO,
				// the whole session safely remains on TCP.
				if (it == s->tcp_peer_meta.end() || !it->second.hello_received)
					return;
				have_peer = true;
				if (it->second.ack_generation != generation)
					return;
				ready_sockets.push_back(p);
			}
			if (!have_peer)
				return;
		}

		for (socket_t p : ready_sockets)
			auto_send_activate(s, p, generation);
		s->auto_udp_ready = true;
		Console.WriteLn("UePcb 1.7.1B: Auto UDP P2P ACTIVE generation %u (%d peer%s)",
			generation, static_cast<int>(ready_sockets.size()), ready_sockets.size() == 1 ? "" : "s");
	}

	static void auto_client_handle_peer_list(UePcbState* s, socket_t conn, const u8* data, int len)
	{
		if (len < 12)
			return;
		const u8 count = data[6];
		const u32 generation = auto_get_u32(data + 8);
		if (count > 3 || len != 12 + static_cast<int>(count) * 12)
			return;

		s->auto_udp_ready = false;
		s->auto_client_generation = generation;
		std::vector<sockaddr_in> peers;
		const u8* p = data + 12;
		for (u8 i = 0; i < count; ++i, p += 12)
		{
			sockaddr_in ep{};
			ep.sin_family = AF_INET;
			std::memcpy(&ep.sin_addr.s_addr, p, 4);
			ep.sin_port = htons(auto_get_u16(p + 4));
			// The Host already excludes this Client from its own list. Still skip
			// an accidental same-MAC entry to protect against malformed control data.
			if (std::memcmp(p + 6, s->mac, 6) != 0)
			{
				peers.push_back(ep);
				char ipbuf[INET_ADDRSTRLEN] = {};
				if (inet_ntop(AF_INET, &ep.sin_addr, ipbuf, sizeof(ipbuf)))
					Console.WriteLn("UePcb 1.7.1B: peer-list UDP endpoint %s:%u", ipbuf, static_cast<unsigned>(ntohs(ep.sin_port)));
			}
		}
		auto_set_udp_peers(s, peers);
		auto_send_ack(s, conn, generation);
		Console.WriteLn("UePcb 1.7.1B: received Auto UDP peer list generation %u (%d endpoint%s); ACK sent",
			generation, static_cast<int>(peers.size()), peers.size() == 1 ? "" : "s");
	}

	static bool auto_handle_client_control(UePcbState* s, socket_t conn, const u8* data, int len)
	{
		if (!auto_is_control(data, len))
			return false;
		if (s->tcp_data_mode != 1)
		{
			Console.Warning("UePcb 1.7.1B: Auto UDP control received while After Connect is Stay TCP; ignoring control packet");
			return true;
		}
		const u8 type = data[4];
		if (type == AUTO_CTRL_PEER_LIST)
			auto_client_handle_peer_list(s, conn, data, len);
		else if (type == AUTO_CTRL_ACTIVATE && len == 10)
		{
			const u32 generation = auto_get_u32(data + 6);
			if (generation == s->auto_client_generation && generation != 0)
			{
				s->auto_udp_ready = true;
				Console.WriteLn("UePcb 1.7.1B: Auto UDP P2P ACTIVE generation %u", generation);
			}
		}
		return true;
	}

	static bool auto_handle_host_control(UePcbState* s, socket_t conn, const u8* data, int len)
	{
		if (!auto_is_control(data, len))
			return false;
		if (s->tcp_data_mode != 1)
		{
			Console.Warning("UePcb 1.7.1B: Auto UDP control received while After Connect is Stay TCP; ignoring control packet");
			return true;
		}

		const u8 type = data[4];
		if (type == AUTO_CTRL_HELLO && len == 14)
		{
			const u16 udp_port = auto_get_u16(data + 6);
			if (udp_port == 0)
				return true;
			{
				std::lock_guard<std::mutex> lk(s->tcp_peers_lock);
				auto it = s->tcp_peer_meta.find(conn);
				if (it == s->tcp_peer_meta.end())
					return true;
				it->second.hello_received = true;
				it->second.udp_port = udp_port;
				std::copy(data + 8, data + 14, it->second.mac.begin());
			}
			auto_host_rebuild_session(s);
		}
		else if (type == AUTO_CTRL_ACK && len == 10)
		{
			const u32 generation = auto_get_u32(data + 6);
			{
				std::lock_guard<std::mutex> lk(s->tcp_peers_lock);
				auto it = s->tcp_peer_meta.find(conn);
				if (it != s->tcp_peer_meta.end())
					it->second.ack_generation = generation;
			}
			auto_host_try_activate(s, generation);
		}
		return true;
	}

	static void tcp_remove_peer(UePcbState* s, socket_t p)
	{
		bool owned = false;
		{
			std::lock_guard<std::mutex> lk(s->tcp_peers_lock);
			for (auto it = s->tcp_peers.begin(); it != s->tcp_peers.end(); ++it)
			{
				if (*it == p)
				{
					s->tcp_peers.erase(it);
					owned = true;
					break;
				}
			}
			s->tcp_peer_meta.erase(p);
		}
		if (owned)
			sock_close(p);
		if (owned && s->tcp_is_host && s->tcp_data_mode == 1 && !s->thread_stop)
			auto_host_rebuild_session(s);
	}

	static void tcp_host_flood(UePcbState* s, socket_t src, const u8* eth, int len)
	{
		std::lock_guard<std::mutex> lk(s->tcp_peers_lock);
		for (socket_t p : s->tcp_peers)
		{
			if (p != src && p != UEPCB_INVALID_SOCKET)
				tcp_send_frame(p, eth, len);
		}
	}

	static void tcp_send_packet(UePcbState* s, const u8* eth, int len)
	{
		std::lock_guard<std::mutex> lk(s->tcp_peers_lock);
		for (socket_t p : s->tcp_peers)
		{
			if (p != UEPCB_INVALID_SOCKET)
				tcp_send_frame(p, eth, len);
		}
	}

	static void tcp_client_recv_loop(UePcbState* s, socket_t conn)
	{
		while (!s->thread_stop)
		{
			u8 hdr[4];
			if (!tcp_recv_exact(conn, hdr, 4))
				break;
			const u32 ln = (static_cast<u32>(hdr[0]) << 24) | (static_cast<u32>(hdr[1]) << 16) |
				(static_cast<u32>(hdr[2]) << 8) | static_cast<u32>(hdr[3]);
			if (ln == 0 || ln > 4096)
				break;
			std::vector<u8> payload(ln);
			if (!tcp_recv_exact(conn, payload.data(), static_cast<int>(ln)))
				break;
			if (auto_handle_client_control(s, conn, payload.data(), static_cast<int>(ln)))
				continue;
			if (ln < 14 || ln > 2048)
				break;
			tcp_push_received_frame(s, payload.data(), static_cast<int>(ln));
		}
	}

	static void tcp_client_loop(UePcbState* s)
	{
		while (!s->thread_stop)
		{
			s->auto_udp_ready = false;
			auto_clear_udp_peers(s);
			socket_t conn = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
			if (conn != UEPCB_INVALID_SOCKET)
			{
				{
					std::lock_guard<std::mutex> lk(s->tcp_sock_lock);
					s->tcp_connect_sock = conn;
				}

				sockaddr_in a{};
				a.sin_family = AF_INET;
				a.sin_port = htons(static_cast<u16>(s->tcp_port));
				const bool connected =
					(inet_pton(AF_INET, s->tcp_host_ip.c_str(), &a.sin_addr) == 1 &&
					 connect(conn, reinterpret_cast<sockaddr*>(&a), sizeof(a)) == 0);

				bool still_owned = false;
				{
					std::lock_guard<std::mutex> lk(s->tcp_sock_lock);
					if (s->tcp_connect_sock == conn)
					{
						s->tcp_connect_sock = UEPCB_INVALID_SOCKET;
						still_owned = true;
					}
				}

				if (s->thread_stop || !still_owned)
				{
					if (still_owned)
						sock_close(conn);
					break;
				}

				if (connected)
				{
					int nd = 1;
					setsockopt(conn, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&nd), sizeof(nd));
					{
						std::lock_guard<std::mutex> lk(s->tcp_peers_lock);
						s->tcp_peers.assign(1, conn);
					}
					Console.WriteLn("UePcb: TCP CLIENT connected %s:%d", s->tcp_host_ip.c_str(), s->tcp_port);
					if (s->tcp_data_mode == 1 && s->udp_sock != UEPCB_INVALID_SOCKET)
					{
						if (auto_send_hello(s, conn))
							Console.WriteLn("UePcb 1.7.1B: Auto UDP HELLO sent (local UDP %d)", s->udp_port);
					}
					tcp_client_recv_loop(s, conn);

					bool owned = false;
					{
						std::lock_guard<std::mutex> lk(s->tcp_peers_lock);
						if (!s->tcp_peers.empty() && s->tcp_peers[0] == conn)
						{
							s->tcp_peers.clear();
							owned = true;
						}
					}
					if (owned)
						sock_close(conn);
					s->auto_udp_ready = false;
					auto_clear_udp_peers(s);
				}
				else
				{
					sock_close(conn);
				}
			}

			for (int i = 0; i < 20 && !s->thread_stop; ++i)
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	}

	static void tcp_host_loop(UePcbState* s)
	{
		socket_t ls = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (ls == UEPCB_INVALID_SOCKET)
			return;
		int one = 1;
		setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&one), sizeof(one));
		sockaddr_in a{};
		a.sin_family = AF_INET;
		a.sin_port = htons(static_cast<u16>(s->tcp_port));
		if (inet_pton(AF_INET, s->tcp_bind_ip.c_str(), &a.sin_addr) != 1 ||
			bind(ls, reinterpret_cast<sockaddr*>(&a), sizeof(a)) != 0)
		{
			Console.Error("UePcb: TCP HOST bind %s:%d FAILED", s->tcp_bind_ip.c_str(), s->tcp_port);
			sock_close(ls);
			return;
		}
		listen(ls, 8);
		{
			std::lock_guard<std::mutex> lk(s->tcp_sock_lock);
			s->tcp_listen_sock = ls;
		}
		Console.WriteLn("UePcb: TCP HOST listening %s:%d", s->tcp_bind_ip.c_str(), s->tcp_port);

		while (!s->thread_stop)
		{
			fd_set rf;
			FD_ZERO(&rf);
			FD_SET(ls, &rf);
			socket_t maxfd = ls;
			std::vector<socket_t> snap;
			{
				std::lock_guard<std::mutex> lk(s->tcp_peers_lock);
				snap = s->tcp_peers;
			}
			for (socket_t p : snap)
			{
				FD_SET(p, &rf);
#ifndef _WIN32
				if (p > maxfd) maxfd = p;
#endif
			}
			timeval tv{0, 200000};
			const int r = select(static_cast<int>(maxfd) + 1, &rf, nullptr, nullptr, &tv);
			if (s->thread_stop)
				break;
			if (r <= 0)
				continue;

			if (FD_ISSET(ls, &rf))
			{
				sockaddr_in remote{};
#ifdef _WIN32
				int remote_len = sizeof(remote);
#else
				socklen_t remote_len = sizeof(remote);
#endif
				socket_t conn = accept(ls, reinterpret_cast<sockaddr*>(&remote), &remote_len);
				if (conn != UEPCB_INVALID_SOCKET)
				{
					int nd = 1;
					setsockopt(conn, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&nd), sizeof(nd));
					bool accepted = false;
					{
						std::lock_guard<std::mutex> lk(s->tcp_peers_lock);
						if (s->tcp_peers.size() < 3)
						{
							s->tcp_peers.push_back(conn);
							UePcbState::TcpPeerMeta meta;
							meta.remote_addr = remote;
							s->tcp_peer_meta[conn] = meta;
							accepted = true;
							Console.WriteLn("UePcb: TCP peer connected (%d total)", static_cast<int>(s->tcp_peers.size()));
						}
					}
					if (!accepted)
					{
						Console.Warning("UePcb 1.7.1B: refusing extra TCP peer; UE PCB session supports Host + 3 Clients");
						sock_close(conn);
					}
				}
			}

			for (socket_t p : snap)
			{
				if (!FD_ISSET(p, &rf))
					continue;
				u8 hdr[4];
				if (!tcp_recv_exact(p, hdr, 4))
				{
					tcp_remove_peer(s, p);
					continue;
				}
				const u32 ln = (static_cast<u32>(hdr[0]) << 24) | (static_cast<u32>(hdr[1]) << 16) |
					(static_cast<u32>(hdr[2]) << 8) | static_cast<u32>(hdr[3]);
				if (ln == 0 || ln > 4096)
				{
					tcp_remove_peer(s, p);
					continue;
				}
				std::vector<u8> payload(ln);
				if (!tcp_recv_exact(p, payload.data(), static_cast<int>(ln)))
				{
					tcp_remove_peer(s, p);
					continue;
				}
				if (auto_handle_host_control(s, p, payload.data(), static_cast<int>(ln)))
					continue;
				if (ln < 14 || ln > 2048)
				{
					tcp_remove_peer(s, p);
					continue;
				}
				tcp_push_received_frame(s, payload.data(), static_cast<int>(ln));
				tcp_host_flood(s, p, payload.data(), static_cast<int>(ln));
			}
		}
	}

	static void tcp_transport_loop(UePcbState* s)
	{
		if (s->tcp_is_host)
			tcp_host_loop(s);
		else
			tcp_client_loop(s);
	}

	static void udp_recv_loop(UePcbState* s)
	{
		u8 buf[2048];

		while (!s->thread_stop)
		{
			sockaddr_in src_addr{};
#ifdef _WIN32
			int addr_len = sizeof(src_addr);
#else
			socklen_t addr_len = sizeof(src_addr);
#endif

			const int n = recvfrom(
				s->udp_sock, reinterpret_cast<char*>(buf), sizeof(buf), 0,
				reinterpret_cast<sockaddr*>(&src_addr), &addr_len);

			if (s->thread_stop)
				break;
			if (n <= 0)
				continue;


			if (n < static_cast<int>(14 + kWireTrailerSize))
				continue;

			const int ethlen = n - kWireTrailerSize;

			// Ignore our own Ethernet source MAC.
			if (std::memcmp(buf + 6, s->mac, 6) == 0)
				continue;

			u32 seq = 0;
			std::memcpy(&seq, buf + ethlen, sizeof(seq));

			const u64 peer_key = mac_key(buf + 6);

			// Diagnostics only. They do not stall the receiver or emulator.
			{
				std::lock_guard<std::mutex> lock(s->jitter_lock);
				auto& jb = s->peer_jitter[peer_key];

				if (jb.seq_valid)
				{
					const s32 delta = static_cast<s32>(seq - jb.next_seq);

					if (delta > 0 && s->loss_log_suppress == 0)
					{
						Console.WriteLn(
							"UePcb: RX jitter/gap from %02x:%02x:%02x:%02x:%02x:%02x "
							"(expected seq %u, got %u, target buffer %u)",
							buf[6], buf[7], buf[8], buf[9], buf[10], buf[11],
							jb.next_seq, seq, jb.target_packets);
						s->loss_log_suppress = 120;
					}
					else if (delta < 0 && s->loss_log_suppress == 0)
					{
						Console.WriteLn(
							"UePcb: RX late/out-of-order from %02x:%02x:%02x:%02x:%02x:%02x "
							"(next seq %u, got %u)",
							buf[6], buf[7], buf[8], buf[9], buf[10], buf[11],
							jb.next_seq, seq);
						s->loss_log_suppress = 120;
					}
					else if (s->loss_log_suppress > 0)
					{
						--s->loss_log_suppress;
					}
				}
			}

			jitter_insert(s, peer_key, seq, to_bulkin(buf, ethlen));
		}
	}

	static bool has_direct_peers(const UePcbState* s)
	{
		for (const std::string& ip : s->peer_ips)
		{
			if (!ip.empty())
				return true;
		}
		return false;
	}

	static void udp_send_to(UePcbState* s, const std::string& ip, int port, const u8* wire, int wire_len)
	{
		if (s->udp_sock == UEPCB_INVALID_SOCKET || ip.empty() || port <= 0 || port > 65535)
			return;

		sockaddr_in dst{};
		dst.sin_family = AF_INET;
		dst.sin_port = htons(static_cast<u16>(port));

		if (inet_pton(AF_INET, ip.c_str(), &dst.sin_addr) != 1)
			return;

		sendto(s->udp_sock, reinterpret_cast<const char*>(wire), wire_len, 0,
			reinterpret_cast<sockaddr*>(&dst), sizeof(dst));
	}

	static void udp_send_wire_to_destinations(UePcbState* s, const u8* wire, int wire_len)
	{
		if (s->connection_mode == 1 && s->tcp_data_mode == 1)
		{
			std::vector<sockaddr_in> peers;
			{
				std::lock_guard<std::mutex> lk(s->auto_udp_lock);
				peers = s->auto_udp_peers;
			}
			for (const sockaddr_in& dst : peers)
			{
				if (s->udp_sock != UEPCB_INVALID_SOCKET)
					sendto(s->udp_sock, reinterpret_cast<const char*>(wire), wire_len, 0,
						reinterpret_cast<const sockaddr*>(&dst), sizeof(dst));
			}
			return;
		}

		if (has_direct_peers(s))
		{
			for (size_t i = 0; i < s->peer_ips.size(); ++i)
			{
				if (!s->peer_ips[i].empty())
					udp_send_to(s, s->peer_ips[i], s->peer_ports[i], wire, wire_len);
			}
			return;
		}
		// Broadcast remains a single-port convenience mode. Multi-instance
		// routing should use Direct Peer endpoints with explicit per-peer ports.
		udp_send_to(s, s->broadcast_ip, s->udp_port, wire, wire_len);
	}

	static void udp_send_packet(UePcbState* s, const u8* eth, int len)
	{
		if (s->udp_sock == UEPCB_INVALID_SOCKET)
			return;
		if (len + kWireTrailerSize > 2048)
			return;

		u8 wire[2048];
		std::memcpy(wire, eth, len);
		const u32 seq = s->tx_seq.fetch_add(1, std::memory_order_relaxed);
		std::memcpy(wire + len, &seq, sizeof(seq));
		const int wire_len = len + kWireTrailerSize;


		udp_send_wire_to_destinations(s, wire, wire_len);
	}

	static void uepcb_handle_reset(USBDevice* dev)
	{
		UePcbState* s = USB_CONTAINER_OF(dev, UePcbState, dev);
		std::memset(s->an986_regs, 0, sizeof(s->an986_regs));
		std::memcpy(s->an986_regs + 0x10, s->mac, 6);
	}

	static void uepcb_handle_control(USBDevice* dev, USBPacket* p, int request,
		int value, int index, int length, u8* data)
	{
		UePcbState* s = USB_CONTAINER_OF(dev, UePcbState, dev);
		if (usb_desc_handle_control(dev, p, request, value, index, length, data) >= 0)
			return;

		const u8 bRequest = request & 0xFF;

		if (bRequest == 0xF0 || bRequest == 0xF1)
		{
			if (s->an986_regs[0x10] == 0)
				std::memcpy(s->an986_regs + 0x10, s->mac, 6);

			if (bRequest == 0xF0)
			{
				an986_read_local(s, index, length, data);

				// 精準模擬 EventFlag 狀態暫存器 (對應 an986.irx)
				if (index == 0x20 && length >= 2)
				{
					data[0] = 0x2D; // Tx/Rx Ready
					data[1] = 0x78; // Link Complete
				}
				else if (index == 0x25 && length >= 2)
				{
					data[1] |= 0x24;
				}
				else if (index == 0x2B && length >= 4)
				{
					data[0] = 0x00;
					data[1] = 0x00;
					data[2] = 0x00;
					data[3] = 0x60; // EventFlag: Link Up & RX Buffer Ready
				}
				for (int i = 0; i < length; i++)
				{
					const int reg = index + i;
					if (reg >= 0x10 && reg < 0x16)
						data[i] = s->mac[reg - 0x10];
				}
				if (index == 0x21 && length >= 2 && s->eeprom_word < 3)
				{
					data[0] = s->mac[s->eeprom_word * 2];
					data[1] = s->mac[s->eeprom_word * 2 + 1];
				}
				p->actual_length = length;
			}
			else
			{
				if (index == 0x25 && length >= 4)
				{
					s->mii_phyaddr = data[0];
					s->mii_reg = data[3] & 0x1F;
				}
				else if (index == 0x7C)
					s->init_done = true;

				for (int i = 0; i < length && (index + i) < (int)sizeof(s->an986_regs); i++)
					s->an986_regs[index + i] = data[i];

				if (index == 0x20 && length >= 1)
					s->eeprom_word = data[0];
			}
			return;
		}
	}

	static void uepcb_handle_data(USBDevice* dev, USBPacket* p)
	{
		UePcbState* s = USB_CONTAINER_OF(dev, UePcbState, dev);

		if (iopMem)
		{
			static bool s_patched_all = false;
			if (!s_patched_all)
			{
				u8* m = iopMem->Main;
				static const u8 pat[12] = {0x04, 0x00, 0x42, 0x30, 0x15, 0x00, 0x40, 0x14, 0x0a, 0x00, 0x02, 0x24};
				for (u32 k = 0; k + 12 < 0x800000; k++)
				{
					if (std::memcmp(m + k, pat, 12) == 0)
					{
						m[k + 4] = 0; m[k + 5] = 0; m[k + 6] = 0; m[k + 7] = 0;
						s_patched_all = true;
						break;
					}
				}
			}
		}

		// AN986.IRX only ever drives Bulk OUT on EP2 and Bulk IN on EP1; EP3 is
		// descriptor-compatible but unused (no sceUsbdInterruptTransfer import
		// in the driver). Reject anything else instead of silently treating an
		// unexpected endpoint as ethernet traffic.
		const u8 ep = p->ep ? p->ep->nr : 0;

		switch (p->pid)
		{
			case USB_TOKEN_OUT:
			{
				if (ep != 2)
				{
					p->status = USB_RET_STALL;
					break;
				}
				u8 pkt[2048];
				int np = std::min<int>(p->buffer_size, (int)sizeof(pkt));
				usb_packet_copy(p, pkt, np);
				s->tx_accum.insert(s->tx_accum.end(), pkt, pkt + np);
				if (s->tx_accum.size() > 8192)
					s->tx_accum.clear();
				while (s->tx_accum.size() >= 2)
				{
					int ethlen = s->tx_accum[0] | (s->tx_accum[1] << 8);
					int total = 2 + ethlen;
					if (ethlen < 14 || ethlen > 1600)
					{
						s->tx_accum.erase(s->tx_accum.begin());
						continue;
					}
					if ((int)s->tx_accum.size() < total)
						break;
					u8 buf[2048];
					int n = std::min<int>(total, (int)sizeof(buf));
					std::memcpy(buf, s->tx_accum.data(), n);
					s->tx_accum.erase(s->tx_accum.begin(), s->tx_accum.begin() + total);

					// Transport send: manual UDP, TCP hub, or TCP-bootstrap -> UDP P2P.
					if (s->connection_mode == 1)
					{
						if (s->tcp_data_mode == 1 && s->auto_udp_ready && s->udp_sock != UEPCB_INVALID_SOCKET)
							udp_send_packet(s, buf + 2, ethlen);
						else
							tcp_send_packet(s, buf + 2, ethlen);
					}
					else
						udp_send_packet(s, buf + 2, ethlen);
				}
				break;
			}
			case USB_TOKEN_IN:
			{
				if (ep == 3)
				{
					// No sceUsbdInterruptTransfer import is present in
					// AN986.IRX - NAK is closer to actual driver usage than
					// fabricating an interrupt status packet.
					p->status = USB_RET_NAK;
					break;
				}
				if (ep != 1)
				{
					p->status = USB_RET_STALL;
					break;
				}
				if (s->connection_mode == 0 || (s->connection_mode == 1 && s->tcp_data_mode == 1))
				{
				// Promote a bounded batch only when the emulated USB controller
				// actually polls the IN endpoint. No peer is waited for and no
				// artificial delay is introduced.
				// Adaptively reorder a small amount of UDP jitter only when the
				// emulated USB IN endpoint is actually polled.
				jitter_promote(s);

				{
					std::lock_guard<std::mutex> lk(s->pending_lock);
					std::lock_guard<std::mutex> qlk(s->in_lock);
					int moved = 0;
					while (!s->pending_rx.empty() && s->in_q.size() < 64 && moved < 4)
					{
						s->in_q.push_back(std::move(s->pending_rx.front()));
						s->pending_rx.pop_front();
						++moved;
					}
				}
				}
				if (!s->rx_partial.empty())
				{
					const int remain = static_cast<int>(s->rx_partial.size() - s->rx_offset);
					const int copyLen = std::min<int>(p->buffer_size, remain);

					usb_packet_copy(p, s->rx_partial.data() + s->rx_offset, copyLen);
					s->rx_offset += copyLen;

					if (s->rx_offset >= s->rx_partial.size())
					{
						s->rx_partial.clear();
						s->rx_offset = 0;
					}
				}
				else
				{
					std::lock_guard<std::mutex> lk(s->in_lock);
					if (!s->in_q.empty())
					{
						s->rx_partial = std::move(s->in_q.front());
						s->in_q.pop_front();
						s->rx_offset = 0;

						const int copyLen = std::min<int>(p->buffer_size, static_cast<int>(s->rx_partial.size()));
						usb_packet_copy(p, s->rx_partial.data(), copyLen);
						s->rx_offset = copyLen;

						if (s->rx_offset >= s->rx_partial.size())
						{
							s->rx_partial.clear();
							s->rx_offset = 0;
						}
					}
					else
					{
						p->status = USB_RET_NAK;
					}
				}
				break;
			}
			default:
				p->status = USB_RET_STALL;
				break;
		}
	}

	static bool parse_mac_hex(const std::string& mh, u8* mac)
	{
		if (mh.size() < 12)
			return false;
		const auto hx = [](char c) -> int {
			if (c >= '0' && c <= '9') return c - '0';
			if (c >= 'a' && c <= 'f') return c - 'a' + 10;
			if (c >= 'A' && c <= 'F') return c - 'A' + 10;
			return -1;
		};
		for (int i = 0; i < 6; i++)
		{
			const int hi = hx(mh[i * 2]);
			const int lo = hx(mh[i * 2 + 1]);
			if (hi < 0 || lo < 0)
				return false;
			mac[i] = static_cast<u8>((hi << 4) | lo);
		}
		return true;
	}

	static void load_runtime_settings(UePcbState* s, SettingsInterface& si, u32 port, bool initial)
	{
		s->config_port = port;
		// 1.7.1B user-facing order is TCP first, UDP second. Keep the internal
		// transport semantics unchanged (0=UDP, 1=TCP) by translating here.
		const int connection_mode_ui = std::clamp(USB::GetConfigInt(si, port, "UePcb", "ConnectionMode171", 0), 0, 1);
		s->connection_mode = (connection_mode_ui == 0) ? 1 : 0;
		s->advanced_settings = USB::GetConfigBool(si, port, "UePcb", "AdvancedSettings", false);
		s->broadcast_ip = s->advanced_settings ?
			USB::GetConfigString(si, port, "UePcb", "TargetIP", "255.255.255.255") : "255.255.255.255";
		s->udp_port = USB::GetConfigInt(si, port, "UePcb", "Port", 7500);
		s->tcp_port = s->udp_port;
		s->tcp_is_host = (USB::GetConfigInt(si, port, "UePcb", "TCPRole", 0) == 0);
		// 1.7.1B user-facing order is Auto UDP P2P first, Stay TCP second. Internal
		// semantics remain 0=Stay TCP, 1=Auto UDP P2P.
		const int tcp_data_mode_ui = std::clamp(USB::GetConfigInt(si, port, "UePcb", "TCPDataMode171", 0), 0, 1);
		s->tcp_data_mode = (tcp_data_mode_ui == 0) ? 1 : 0;
		s->tcp_bind_ip = USB::GetConfigString(si, port, "UePcb", "TCPBindIP", "0.0.0.0");
		s->tcp_host_ip = USB::GetConfigString(si, port, "UePcb", "TCPHostIP", "127.0.0.1");

		const int grace_ms = s->advanced_settings ? std::clamp(USB::GetConfigInt(si, port, "UePcb", "JitterGraceMs", 4), 0, 20) : 4;
		const int decay_ms = s->advanced_settings ? std::clamp(USB::GetConfigInt(si, port, "UePcb", "JitterDecayMs", 250), 250, 60000) : 250;
		const int min_target = s->advanced_settings ? std::clamp(USB::GetConfigInt(si, port, "UePcb", "JitterMinTarget", 1), 1, 8) : 1;
		const int max_target = s->advanced_settings ? std::clamp(USB::GetConfigInt(si, port, "UePcb", "JitterMaxTarget", 8), min_target, 8) : 8;
		const int max_packets = s->advanced_settings ? std::clamp(USB::GetConfigInt(si, port, "UePcb", "JitterMaxPackets", 8), max_target, 32) : 8;
		s->jitter_grace = std::chrono::milliseconds(grace_ms);
		s->jitter_decay_interval = std::chrono::milliseconds(decay_ms);
		s->jitter_min_target = static_cast<u32>(min_target);
		s->jitter_max_target = static_cast<u32>(max_target);
		s->jitter_max_packets = static_cast<size_t>(max_packets);

		s->beta_sync_hold = s->advanced_settings ?
			USB::GetConfigBool(si, port, "UePcb", "BetaSyncHold", true) : true;
		const int sync_hold_ms = s->advanced_settings ?
			std::clamp(USB::GetConfigInt(si, port, "UePcb", "BetaSyncHoldMs", 30), 3, 100) : 30;
		s->beta_sync_hold_time = std::chrono::milliseconds(sync_hold_ms);
		s->beta_adaptive_playout = s->advanced_settings ?
			USB::GetConfigBool(si, port, "UePcb", "BetaAdaptivePlayout", false) : false;
		const int playout_max_ms = s->advanced_settings ?
			std::clamp(USB::GetConfigInt(si, port, "UePcb", "BetaPlayoutMaxMs", 3), 0, 20) : 3;
		s->beta_playout_max = std::chrono::milliseconds(playout_max_ms);
		s->beta_global_stall_guard = s->advanced_settings ?
			USB::GetConfigBool(si, port, "UePcb", "BetaGlobalStallGuard", false) : false;

		// 1.6 keeps a single source of truth for each endpoint: the IP text
		// stored in Peer1IP..Peer3IP (or TCPHostIP) plus its explicit peer port.
		// Saved history is now only a UI convenience and never silently overrides
		// a typed address in the transport backend.
		for (int i = 0; i < 3; ++i)
		{
			const std::string ip_key = "Peer" + std::to_string(i + 1) + "IP";
			const std::string port_key = "Peer" + std::to_string(i + 1) + "Port";
			s->peer_ips[i] = USB::GetConfigString(si, port, "UePcb", ip_key.c_str(), "");
			s->peer_ports[i] = std::clamp(
				USB::GetConfigInt(si, port, "UePcb", port_key.c_str(), 7500), 1, 65535);
		}
		const std::string mh = USB::GetConfigString(si, port, "UePcb", "MacHex", "");
		u8 new_mac[6] = {};
		if (parse_mac_hex(mh, new_mac))
		{
			std::memcpy(s->mac, new_mac, 6);
		}
		else if (initial)
		{
			static std::mt19937 rng(std::random_device{}());
			const u32 r = rng();
			s->mac[0] = 0x00; s->mac[1] = 0x90; s->mac[2] = 0x2E;
			s->mac[3] = static_cast<u8>(r & 0xFF);
			s->mac[4] = static_cast<u8>((r >> 8) & 0xFF);
			s->mac[5] = static_cast<u8>((r >> 16) & 0xFF);
		}
		std::memcpy(s->an986_regs + 0x10, s->mac, 6);

		Console.WriteLn("UePcb 1.7.1B settings: mode=%s port=%d TCPAfter=%s Grace=%dms Decay=%dms Target=%d-%d Queue=%d Hold=%s(%dms) Playout=%s(max %dms) StallGuard=%s",
			s->connection_mode == 0 ? "UDP" : "TCP", s->udp_port,
			s->tcp_data_mode == 0 ? "StayTCP" : "AutoUDP",
			grace_ms, decay_ms, min_target, max_target, max_packets,
			s->beta_sync_hold ? "ON" : "OFF", sync_hold_ms,
			s->beta_adaptive_playout ? "ON" : "OFF", playout_max_ms,
			s->beta_global_stall_guard ? "ON" : "OFF");
	}

	static void stop_transport(UePcbState* s)
	{
		s->thread_stop = true;

		if (s->udp_sock != UEPCB_INVALID_SOCKET)
		{
			sock_close(s->udp_sock);
			s->udp_sock = UEPCB_INVALID_SOCKET;
		}
		{
			std::lock_guard<std::mutex> lk(s->tcp_sock_lock);
			if (s->tcp_listen_sock != UEPCB_INVALID_SOCKET)
			{
				sock_close(s->tcp_listen_sock);
				s->tcp_listen_sock = UEPCB_INVALID_SOCKET;
			}
			if (s->tcp_connect_sock != UEPCB_INVALID_SOCKET)
			{
				sock_close(s->tcp_connect_sock);
				s->tcp_connect_sock = UEPCB_INVALID_SOCKET;
			}
		}
		{
			std::lock_guard<std::mutex> lk(s->tcp_peers_lock);
			for (socket_t p : s->tcp_peers)
				if (p != UEPCB_INVALID_SOCKET)
					sock_close(p);
			s->tcp_peers.clear();
			s->tcp_peer_meta.clear();
		}
		s->auto_udp_ready = false;
		auto_clear_udp_peers(s);
		if (s->recv_thread.joinable())
			s->recv_thread.join();
		if (s->udp_recv_thread.joinable())
			s->udp_recv_thread.join();
	}

	static void clear_transport_queues(UePcbState* s)
	{
		{
			std::lock_guard<std::mutex> lk(s->jitter_lock);
			s->peer_jitter.clear();
			s->diag_last_forced_valid = false;
		}
		{
			std::lock_guard<std::mutex> lk(s->pending_lock);
			s->pending_rx.clear();
		}
		{
			std::lock_guard<std::mutex> lk(s->in_lock);
			s->in_q.clear();
			// Do not touch rx_partial here. The emulated USB thread may be
			// finishing a packet already handed to EP1 while Apply restarts
			// only the host transport. Let that partial packet complete.
		}
	}

	static bool open_udp_socket(UePcbState* s, bool auto_allocate)
	{
		const int base_port = s->udp_port;
		const int attempts = auto_allocate ? 16 : 1;

		for (int i = 0; i < attempts; ++i)
		{
			const int candidate = base_port + i;
			if (candidate > 65535)
				break;

			socket_t sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
			if (sock == UEPCB_INVALID_SOCKET)
			{
				Console.Error("UePcb: UDP socket creation failed");
				return false;
			}

			int one = 1;
			if (!auto_allocate)
				setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&one), sizeof(one));
#ifdef _WIN32
			else
				setsockopt(sock, SOL_SOCKET, SO_EXCLUSIVEADDRUSE, reinterpret_cast<const char*>(&one), sizeof(one));
#endif
			setsockopt(sock, SOL_SOCKET, SO_BROADCAST, reinterpret_cast<const char*>(&one), sizeof(one));
			int rcvbuf = 256 * 1024;
			setsockopt(sock, SOL_SOCKET, SO_RCVBUF, reinterpret_cast<const char*>(&rcvbuf), sizeof(rcvbuf));
#ifdef _WIN32
			DWORD recv_timeout_ms = 200;
			setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&recv_timeout_ms), sizeof(recv_timeout_ms));
#else
			timeval recv_timeout{0, 200000};
			setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&recv_timeout), sizeof(recv_timeout));
#endif

			sockaddr_in bind_addr{};
			bind_addr.sin_family = AF_INET;
			bind_addr.sin_port = htons(static_cast<u16>(candidate));
			bind_addr.sin_addr.s_addr = INADDR_ANY;
			if (bind(sock, reinterpret_cast<sockaddr*>(&bind_addr), sizeof(bind_addr)) == 0)
			{
				s->udp_sock = sock;
				s->udp_port = candidate;
				if (auto_allocate)
					Console.WriteLn("UePcb 1.7.1B: Auto UDP local port allocated: %d", candidate);
				return true;
			}

			sock_close(sock);
		}

		Console.Error("UePcb: UDP bind starting at port %d FAILED", base_port);
		s->udp_sock = UEPCB_INVALID_SOCKET;
		return false;
	}

	static bool start_transport(UePcbState* s)
	{
		s->thread_stop = false;
		s->auto_udp_ready = false;
		auto_clear_udp_peers(s);

		if (s->connection_mode == 0)
		{
			if (!open_udp_socket(s, false))
				return false;
			s->recv_thread = std::thread(udp_recv_loop, s);
			return true;
		}

		// Auto P2P uses TCP only for session/bootstrap/control. Open a dedicated
		// UDP socket first so the negotiated HELLO advertises the actual bound port.
		// If UDP cannot be opened, keep TCP alive as a safe fallback instead of
		// making the whole device fail to start.
		if (s->tcp_data_mode == 1)
		{
			if (open_udp_socket(s, true))
				s->udp_recv_thread = std::thread(udp_recv_loop, s);
			else
				Console.Warning("UePcb 1.7.1B: Auto UDP unavailable; session will remain on TCP");
		}

		Console.WriteLn("UePcb: TCP mode role=%s session-port=%d after-connect=%s %s=%s",
			s->tcp_is_host ? "HOST" : "CLIENT", s->tcp_port,
			s->tcp_data_mode == 0 ? "STAY-TCP" : "AUTO-UDP-P2P",
			s->tcp_is_host ? "listen" : "host",
			s->tcp_is_host ? s->tcp_bind_ip.c_str() : s->tcp_host_ip.c_str());
		s->recv_thread = std::thread(tcp_transport_loop, s);
		return true;
	}

	static void uepcb_handle_destroy(USBDevice* dev)
	{
		UePcbState* s = USB_CONTAINER_OF(dev, UePcbState, dev);
		stop_transport(s);

		if (s->connection_mode == 0 || (s->connection_mode == 1 && s->tcp_data_mode == 1))
		{
			std::lock_guard<std::mutex> lock(s->jitter_lock);
			for (const auto& entry : s->peer_jitter)
			{
				const auto& jb = entry.second;
				Console.WriteLn(
					"UePcb DIAG summary peer=%llu RX=%llu Ahead=%llu Recovered=%llu ForcedSkip=%llu Late=%llu Duplicate=%llu MaxAhead=%u Hold=%llu MaxGapMs=%llu FastAttack=%llu PeakTarget=%u FinalBuffer=%u Queue=%zu",
					static_cast<unsigned long long>(entry.first),
					static_cast<unsigned long long>(jb.diag_rx),
					static_cast<unsigned long long>(jb.diag_ahead),
					static_cast<unsigned long long>(jb.diag_recovered),
					static_cast<unsigned long long>(jb.diag_forced_skip),
					static_cast<unsigned long long>(jb.diag_late),
					static_cast<unsigned long long>(jb.diag_duplicate),
					jb.diag_max_ahead,
					static_cast<unsigned long long>(jb.diag_hold_events),
					static_cast<unsigned long long>(jb.diag_max_gap_ms),
					static_cast<unsigned long long>(jb.diag_fast_attack),
					jb.diag_peak_target, jb.target_packets, jb.packets.size());
			}
			Console.WriteLn("UePcb EXP summary PlayoutEvents=%llu PlayoutHoldTotalMs=%llu PlayoutHoldMaxMs=%llu",
				static_cast<unsigned long long>(s->diag_playout_events),
				static_cast<unsigned long long>(s->diag_playout_total_hold_ms),
				static_cast<unsigned long long>(s->diag_playout_max_hold_ms));
		}
		delete s;
	}

	USBDevice* UePcbDevice::CreateDevice(SettingsInterface& si, u32 port, u32 subtype) const
	{
		wsa_ensure();
		UePcbState* s = new UePcbState();
		load_runtime_settings(s, si, port, true);

		s->dev.speed = USB_SPEED_FULL;
		s->desc.full = &s->desc_dev;
		s->desc.str = uepcb_strings;

		if (usb_desc_parse_dev(uepcb_dev_descriptor, sizeof(uepcb_dev_descriptor), s->desc, s->desc_dev) < 0)
			goto fail;
		if (usb_desc_parse_config(uepcb_config_descriptor, sizeof(uepcb_config_descriptor), s->desc_dev) < 0)
			goto fail;

		s->dev.klass.handle_attach = usb_desc_attach;
		s->dev.klass.handle_reset = uepcb_handle_reset;
		s->dev.klass.handle_control = uepcb_handle_control;
		s->dev.klass.handle_data = uepcb_handle_data;
		s->dev.klass.unrealize = uepcb_handle_destroy;
		s->dev.klass.usb_desc = &s->desc;
		s->dev.klass.product_desc = s->desc.str[2];

		usb_desc_init(&s->dev);
		usb_ep_init(&s->dev);
		uepcb_handle_reset(&s->dev);
		if (!start_transport(s))
			Console.Warning("UePcb: transport could not start; change settings and Apply to retry without restarting the game.");
		return &s->dev;

	fail:
		uepcb_handle_destroy(&s->dev);
		return nullptr;
	}

	void UePcbDevice::UpdateSettings(USBDevice* dev, SettingsInterface& si) const
	{
		UePcbState* s = USB_CONTAINER_OF(dev, UePcbState, dev);
		Console.WriteLn("UePcb 1.7.1B: applying settings live (transport restart only; game stays running)");

		// Stop socket/thread activity before changing strings, ports or mode.
		// tx_seq deliberately survives so remote peers do not see our UDP
		// sequence counter jump backwards after an Apply.
		stop_transport(s);
		clear_transport_queues(s);
		load_runtime_settings(s, si, s->config_port, false);
		if (!start_transport(s))
			Console.Error("UePcb 1.7.1B: live Apply completed, but transport restart failed");
		else
			Console.WriteLn("UePcb 1.7.1B: live Apply complete");
	}

	const char* UePcbDevice::Name() const { return "UE PCB (Namco arcade TCP/UDP) 1.7.1B"; }
	const char* UePcbDevice::TypeName() const { return "UePcb"; }
	const char* UePcbDevice::IconName() const { return ""; }

	bool UePcbDevice::Freeze(USBDevice* dev, StateWrapper& sw) const
	{
		UePcbState* s = USB_CONTAINER_OF(dev, UePcbState, dev);
		if (!sw.DoMarker("UePcbDevice"))
			return false;
		sw.DoBytes(s->an986_regs, sizeof(s->an986_regs));
		return true;
	}

	std::span<const SettingInfo> UePcbDevice::Settings(u32 subtype) const
	{
		static const char* connection_modes[] = {"TCP", "UDP", nullptr};
		static const char* tcp_roles[] = {"Host", "Client", nullptr};
		static const char* tcp_data_modes[] = {"Auto UDP P2P", "Stay TCP", nullptr};
		static const SettingInfo settings[] = {
			{.type = SettingInfo::Type::IntegerList, .name = "ConnectionMode171", .display_name = "Connection Mode",
				.description = "TCP is the recommended default and can automatically switch to a discovered UDP P2P mesh. UDP remains available for manual Direct Peer/Broadcast setup.",
				.default_value = "0", .min_value = "0", .options = connection_modes},
			{.type = SettingInfo::Type::Integer, .name = "Port", .display_name = "Local UDP / TCP Port",
				.description = "UDP: this emulator instance's local receive port. TCP: the shared session port used by Host and all Clients. In Auto UDP P2P, each machine automatically allocates a free UDP port starting from this value. Default: 7500. Manual UDP same-PC multi-instance play still requires unique local ports per instance.",
				.default_value = "7500", .min_value = "1", .max_value = "65535", .step_value = "1"},

			{.type = SettingInfo::Type::String, .name = "TargetIP", .display_name = "Broadcast Address",
				.description = "UDP only. Used when all Direct Peer IPs are empty. Default: 255.255.255.255.", .default_value = "255.255.255.255"},
			{.type = SettingInfo::Type::String, .name = "Peer1IP", .display_name = "Direct Peer 1 IP", .description = "UDP peer IPv4 address. The custom UI also offers the shared saved-IP history in this editable field.", .default_value = ""},
			{.type = SettingInfo::Type::Integer, .name = "Peer1Port", .display_name = "Peer 1 Port", .description = "UDP destination port for Peer 1. Default: 7500.", .default_value = "7500", .min_value = "1", .max_value = "65535", .step_value = "1"},
			{.type = SettingInfo::Type::String, .name = "Peer2IP", .display_name = "Direct Peer 2 IP", .description = "UDP peer IPv4 address. The custom UI also offers the shared saved-IP history in this editable field.", .default_value = ""},
			{.type = SettingInfo::Type::Integer, .name = "Peer2Port", .display_name = "Peer 2 Port", .description = "UDP destination port for Peer 2. Default: 7500.", .default_value = "7500", .min_value = "1", .max_value = "65535", .step_value = "1"},
			{.type = SettingInfo::Type::String, .name = "Peer3IP", .display_name = "Direct Peer 3 IP", .description = "UDP peer IPv4 address. The custom UI also offers the shared saved-IP history in this editable field.", .default_value = ""},
			{.type = SettingInfo::Type::Integer, .name = "Peer3Port", .display_name = "Peer 3 Port", .description = "UDP destination port for Peer 3. Default: 7500.", .default_value = "7500", .min_value = "1", .max_value = "65535", .step_value = "1"},
			// Legacy 1.5/1.6 saved-IP selector keys retained for migration only.
			{.type = SettingInfo::Type::Integer, .name = "Peer1HistorySlot", .display_name = "Legacy Peer 1 Saved IP", .description = "Legacy migration key.", .default_value = "0", .min_value = "0", .max_value = "10", .step_value = "1"},
			{.type = SettingInfo::Type::Integer, .name = "Peer2HistorySlot", .display_name = "Legacy Peer 2 Saved IP", .description = "Legacy migration key.", .default_value = "0", .min_value = "0", .max_value = "10", .step_value = "1"},
			{.type = SettingInfo::Type::Integer, .name = "Peer3HistorySlot", .display_name = "Legacy Peer 3 Saved IP", .description = "Legacy migration key.", .default_value = "0", .min_value = "0", .max_value = "10", .step_value = "1"},

			{.type = SettingInfo::Type::IntegerList, .name = "TCPRole", .display_name = "TCP Role",
				.description = "Host listens and relays frames to all Clients. Client connects to the Host.", .default_value = "0", .min_value = "0", .options = tcp_roles},
			{.type = SettingInfo::Type::IntegerList, .name = "TCPDataMode171", .display_name = "After Connect",
				.description = "Auto UDP P2P is the recommended default: TCP exchanges endpoints and then activates a direct UDP mesh. 1.7.1B also translates same-PC loopback peers to a Host address reachable by remote Clients. Stay TCP remains available as the second option. Direct Internet NAT hole punching is not included.",
				.default_value = "0", .min_value = "0", .options = tcp_data_modes},
			{.type = SettingInfo::Type::String, .name = "TCPBindIP", .display_name = "Listen IP",
				.description = "TCP Host only. 0.0.0.0 listens on all LAN/VPN adapters; 127.0.0.1 limits it to this PC.", .default_value = "0.0.0.0"},
			{.type = SettingInfo::Type::String, .name = "TCPHostIP", .display_name = "TCP Host IP",
				.description = "TCP Client only. Same PC: 127.0.0.1. LAN/VPN: enter the Host machine's LAN or VPN IP.", .default_value = "127.0.0.1"},
			{.type = SettingInfo::Type::Integer, .name = "TCPHostHistorySlot", .display_name = "TCP Host Saved IP",
				.description = "0=Manual TCP Host IP, 1-10=shared saved address.", .default_value = "0", .min_value = "0", .max_value = "10", .step_value = "1"},

			{.type = SettingInfo::Type::Boolean, .name = "RememberCurrentIPs", .display_name = "Remember Current IPs",
				.description = "UI action storage key. The custom UEPCB page uses a Save Current IPs push button.", .default_value = "false"},
			{.type = SettingInfo::Type::Boolean, .name = "ClearIPHistory", .display_name = "Clear Shared IP History",
				.description = "UI action storage key. The custom UEPCB page uses a Clear Saved IPs push button.", .default_value = "false"},

			{.type = SettingInfo::Type::String, .name = "History1", .display_name = "Saved IP 1", .description = "Hidden history storage.", .default_value = ""},
			{.type = SettingInfo::Type::String, .name = "History2", .display_name = "Saved IP 2", .description = "Hidden history storage.", .default_value = ""},
			{.type = SettingInfo::Type::String, .name = "History3", .display_name = "Saved IP 3", .description = "Hidden history storage.", .default_value = ""},
			{.type = SettingInfo::Type::String, .name = "History4", .display_name = "Saved IP 4", .description = "Hidden history storage.", .default_value = ""},
			{.type = SettingInfo::Type::String, .name = "History5", .display_name = "Saved IP 5", .description = "Hidden history storage.", .default_value = ""},
			{.type = SettingInfo::Type::String, .name = "History6", .display_name = "Saved IP 6", .description = "Hidden history storage.", .default_value = ""},
			{.type = SettingInfo::Type::String, .name = "History7", .display_name = "Saved IP 7", .description = "Hidden history storage.", .default_value = ""},
			{.type = SettingInfo::Type::String, .name = "History8", .display_name = "Saved IP 8", .description = "Hidden history storage.", .default_value = ""},
			{.type = SettingInfo::Type::String, .name = "History9", .display_name = "Saved IP 9", .description = "Hidden history storage.", .default_value = ""},
			{.type = SettingInfo::Type::String, .name = "History10", .display_name = "Saved IP 10", .description = "Hidden history storage.", .default_value = ""},

			{.type = SettingInfo::Type::Boolean, .name = "AdvancedSettings", .display_name = "Advanced Settings",
				.description = "Normal play can leave this OFF. UE PCB 1.6 then forces the tested UDP defaults: Grace 4 ms, Decay 250 ms, Target 1-8, Queue 8, and Sync Hold ON at 30 ms. Enable only to tune these values or test experimental options.", .default_value = "false"},
			{.type = SettingInfo::Type::Integer, .name = "JitterGraceMs", .display_name = "Jitter Grace (ms)", .description = "UDP only. Normal short reorder is ignored inside this grace window; after it, Fast Attack may raise the target before forced skip.", .default_value = "4", .min_value = "0", .max_value = "20", .step_value = "1"},
			{.type = SettingInfo::Type::Integer, .name = "JitterDecayMs", .display_name = "Buffer Decay (ms)", .description = "UDP only. Stable time before target buffer relaxes by one. 250 ms gives roughly 1.75 s for 8 -> 1.", .default_value = "250", .min_value = "250", .max_value = "60000", .step_value = "250"},
			{.type = SettingInfo::Type::Integer, .name = "JitterMinTarget", .display_name = "Minimum Target Buffer", .description = "UDP only. Default 1.", .default_value = "1", .min_value = "1", .max_value = "8", .step_value = "1"},
			{.type = SettingInfo::Type::Integer, .name = "JitterMaxTarget", .display_name = "Maximum Target Buffer", .description = "UDP only. Default 8 for 1.6 Fast Attack.", .default_value = "8", .min_value = "1", .max_value = "8", .step_value = "1"},
			{.type = SettingInfo::Type::Integer, .name = "JitterMaxPackets", .display_name = "Maximum Jitter Queue", .description = "UDP only. Default 8; keep small to avoid latency buildup.", .default_value = "8", .min_value = "1", .max_value = "32", .step_value = "1"},

			// 1.6 advanced synchronization controls. Sync Hold is recommended ON; experimental controls default OFF.
			{.type = SettingInfo::Type::Boolean, .name = "BetaSyncHold", .display_name = "Sync Hold",
				.description = "UDP advanced setting. Recommended ON and enabled automatically when Advanced Settings is OFF. Holds a missing sequence for a bounded time before forced skip; default 30 ms.", .default_value = "true"},
			{.type = SettingInfo::Type::Integer, .name = "BetaSyncHoldMs", .display_name = "Sync Hold Time",
				.description = "Maximum missing-packet hold when Sync Hold is enabled. Default and recommended starting point: 30 ms.", .default_value = "30", .min_value = "3", .max_value = "100", .step_value = "1"},
			{.type = SettingInfo::Type::Boolean, .name = "BetaAdaptivePlayout", .display_name = "Adaptive Playout",
				.description = "UDP EXPERIMENTAL advanced setting. Default OFF. Adds a small playout delay when the adaptive target rises. It can make some links slower or worse; use only for A/B testing.", .default_value = "false"},
			{.type = SettingInfo::Type::Integer, .name = "BetaPlayoutMaxMs", .display_name = "Playout Maximum",
				.description = "Maximum target playout delay. Default: 3 ms. Actual observed hold can be longer because delivery occurs on the next USB poll; keep this value small.", .default_value = "3", .min_value = "0", .max_value = "20", .step_value = "1"},
			{.type = SettingInfo::Type::Boolean, .name = "BetaGlobalStallGuard", .display_name = "Global Stall Guard",
				.description = "UDP experimental advanced setting. Default OFF. If different peers forced-skip within 100 ms, treats it as a likely local emulator stall. It is not recommended for normal play because it may make some connections worse.", .default_value = "false"},

			{.type = SettingInfo::Type::String, .name = "MacHex", .display_name = "MAC 12-hex", .description = "Leave blank to auto-generate a unique MAC per emulator instance.", .default_value = ""}};
		return settings;
	}

} // namespace usb_uepcb