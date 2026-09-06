// SPDX-FileCopyrightText: 2002-2026 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+
//
// usb-uepcb1.5 (TCP / UDP Select)
// Based on: Claude UePcb 1.3
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
		"", "Namco", "UE PCB v1.5 (TCP/UDP)", ""};

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
		int udp_port = 7500;
		std::string broadcast_ip;
		std::array<std::string, 3> peer_ips{};
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

		// 1.3.3 user-tunable adaptive jitter parameters. Defaults reproduce
		// Claude 1.3/1.3.1 behavior exactly. Values are clamped on load.
		size_t jitter_max_packets = 8;
		u32 jitter_min_target = 1;
		u32 jitter_max_target = 4;
		std::chrono::milliseconds jitter_grace{3};
		std::chrono::milliseconds jitter_decay_interval{4000};

		// Transport selection: 0 = UDP, 1 = TCP.
		int connection_mode = 0;
		bool advanced_settings = false;

		// UDP transport.
		socket_t udp_sock = UEPCB_INVALID_SOCKET;

		// TCP transport. Host is an N-way hub; Client keeps one connection to Host.
		bool tcp_is_host = true;
		int tcp_port = 7500;
		std::string tcp_host_ip = "127.0.0.1";
		std::string tcp_bind_ip = "0.0.0.0";
		std::mutex tcp_sock_lock;
		socket_t tcp_listen_sock = UEPCB_INVALID_SOCKET;
		std::mutex tcp_peers_lock;
		std::vector<socket_t> tcp_peers;

		std::thread recv_thread;
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
	// can be tuned from the USB settings UI without recompiling. Defaults
	// remain Grace=3ms, Decay=4000ms, MinTarget=1, MaxTarget=4, Queue=8.

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
			// Claude UePcb 1.3: true only when we're giving up on a packet
			// that's still missing after its grace period - i.e. real
			// evidence that the current target_packets wasn't deep enough.
			// False for the ordinary "next packet was already sitting
			// there" case, even if it arrived slightly out of send order.
			bool selected_is_forced_skip = false;

			const auto now = std::chrono::steady_clock::now();

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

					if (jb.packets.size() >= jb.target_packets || age >= s->jitter_grace)
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
					// A sequence gap exists. Give the missing packet a very
					// short grace period so normal UDP reordering can recover.
					if (!jb.gap_active)
					{
						jb.gap_active = true;
						jb.gap_since = now;
					}

					const auto gap_age =
						std::chrono::duration_cast<std::chrono::milliseconds>(
							now - jb.gap_since);

					if (jb.packets.size() >= jb.target_packets || gap_age >= s->jitter_grace)
					{
						// The missing packet is considered too late. Advance to
						// the earliest packet already buffered; never block PCSX2.
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

			// Diagnostic only: if a packet first arrived ahead but is now promoted
			// exactly at the expected playback point, that reorder was recovered.
			if (!selected_is_forced_skip && packet_it->second.diag_was_ahead)
				++jb.diag_recovered;

			s->pending_rx.push_back(std::move(packet_it->second.data));
			jb.packets.erase(packet_it);
			jb.next_seq = selected_seq + 1;
			jb.gap_active = false;

			if (!jb.last_target_change_valid)
			{
				jb.last_target_change = now;
				jb.last_target_change_valid = true;
			}

			if (selected_is_forced_skip)
			{
				++jb.diag_forced_skip;

				// Diagnostic only: multiple different peers forced-skipping within
				// 100ms strongly resembles the same-PC 1P stall seen in test logs.
				if (s->diag_last_forced_valid && s->diag_last_forced_peer != selected_peer)
				{
					const auto burst_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
						now - s->diag_last_forced_time);
					if (burst_ms.count() >= 0 && burst_ms.count() <= 100)
					{
						Console.WriteLn(
							"UePcb DIAG: possible local stall - forced skips on different peers within %lldms",
							static_cast<long long>(burst_ms.count()));
					}
				}
				s->diag_last_forced_time = now;
				s->diag_last_forced_peer = selected_peer;
				s->diag_last_forced_valid = true;

				// Real, unresolved gap - this is the only place target_packets
				// grows now. One step at a time, same as 1.2.
				if (jb.target_packets < s->jitter_max_target)
				{
					++jb.target_packets;
					Console.WriteLn(
						"UePcb: target buffer -> %u (peer key %llu) - forced skip of a missing packet",
						jb.target_packets, static_cast<unsigned long long>(selected_peer));
				}
				jb.last_target_change = now;
			}
			else
			{
				// Time-based decay: relax by one step once it's genuinely
				// been calm for s->jitter_decay_interval, rather than requiring
				// N consecutive perfect deliveries that any single hiccup
				// would reset to zero.
				const auto since_change =
					std::chrono::duration_cast<std::chrono::milliseconds>(now - jb.last_target_change);

				if (jb.target_packets > s->jitter_min_target && since_change >= s->jitter_decay_interval)
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

	static void tcp_remove_peer(UePcbState* s, socket_t p)
	{
		std::lock_guard<std::mutex> lk(s->tcp_peers_lock);
		for (auto it = s->tcp_peers.begin(); it != s->tcp_peers.end(); ++it)
		{
			if (*it == p)
			{
				s->tcp_peers.erase(it);
				break;
			}
		}
		sock_close(p);
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
			if (ln < 14 || ln > 2048)
				break;
			std::vector<u8> eth(ln);
			if (!tcp_recv_exact(conn, eth.data(), static_cast<int>(ln)))
				break;
			tcp_push_received_frame(s, eth.data(), static_cast<int>(ln));
		}
	}

	static void tcp_client_loop(UePcbState* s)
	{
		while (!s->thread_stop)
		{
			socket_t conn = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
			if (conn != UEPCB_INVALID_SOCKET)
			{
				sockaddr_in a{};
				a.sin_family = AF_INET;
				a.sin_port = htons(static_cast<u16>(s->tcp_port));
				if (inet_pton(AF_INET, s->tcp_host_ip.c_str(), &a.sin_addr) == 1 &&
					connect(conn, reinterpret_cast<sockaddr*>(&a), sizeof(a)) == 0)
				{
					int nd = 1;
					setsockopt(conn, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&nd), sizeof(nd));
					{
						std::lock_guard<std::mutex> lk(s->tcp_peers_lock);
						s->tcp_peers.assign(1, conn);
					}
					Console.WriteLn("UePcb: TCP CLIENT connected %s:%d", s->tcp_host_ip.c_str(), s->tcp_port);
					tcp_client_recv_loop(s, conn);
					{
						std::lock_guard<std::mutex> lk(s->tcp_peers_lock);
						if (!s->tcp_peers.empty() && s->tcp_peers[0] == conn)
							s->tcp_peers.clear();
					}
					sock_close(conn);
				}
				else
					sock_close(conn);
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
				socket_t conn = accept(ls, nullptr, nullptr);
				if (conn != UEPCB_INVALID_SOCKET)
				{
					int nd = 1;
					setsockopt(conn, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&nd), sizeof(nd));
					std::lock_guard<std::mutex> lk(s->tcp_peers_lock);
					s->tcp_peers.push_back(conn);
					Console.WriteLn("UePcb: TCP peer connected (%d total)", static_cast<int>(s->tcp_peers.size()));
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
				if (ln < 14 || ln > 2048)
				{
					tcp_remove_peer(s, p);
					continue;
				}
				std::vector<u8> eth(ln);
				if (!tcp_recv_exact(p, eth.data(), static_cast<int>(ln)))
				{
					tcp_remove_peer(s, p);
					continue;
				}
				tcp_push_received_frame(s, eth.data(), static_cast<int>(ln));
				tcp_host_flood(s, p, eth.data(), static_cast<int>(ln));
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

	static void udp_send_to(UePcbState* s, const std::string& ip, const u8* wire, int wire_len)
	{
		if (s->udp_sock == UEPCB_INVALID_SOCKET || ip.empty())
			return;

		sockaddr_in dst{};
		dst.sin_family = AF_INET;
		dst.sin_port = htons(static_cast<u16>(s->udp_port));

		if (inet_pton(AF_INET, ip.c_str(), &dst.sin_addr) != 1)
			return;

		sendto(s->udp_sock, reinterpret_cast<const char*>(wire), wire_len, 0,
			reinterpret_cast<sockaddr*>(&dst), sizeof(dst));
	}

	static void udp_send_packet(UePcbState* s, const u8* eth, int len)
	{
		if (s->udp_sock == UEPCB_INVALID_SOCKET)
			return;
		if (len + kWireTrailerSize > 2048)
			return;

		// Build the wire packet once (eth frame + trailing seq number) and
		// reuse it for every destination, instead of re-touching the
		// sequence counter per peer - all peers should see the same seq
		// stream from us, not a separate counter each.
		u8 wire[2048];
		std::memcpy(wire, eth, len);
		const u32 seq = s->tx_seq.fetch_add(1, std::memory_order_relaxed);
		std::memcpy(wire + len, &seq, sizeof(seq));
		const int wire_len = len + kWireTrailerSize;

		if (has_direct_peers(s))
		{
			for (const std::string& ip : s->peer_ips)
				udp_send_to(s, ip, wire, wire_len);
			return;
		}

		udp_send_to(s, s->broadcast_ip, wire, wire_len);
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

		if (iopMem && iopMem->Main)
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

					// Transport send: UDP mesh/broadcast or TCP hub/client.
					if (s->connection_mode == 1)
						tcp_send_packet(s, buf + 2, ethlen);
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
				if (s->connection_mode == 0)
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

	static void uepcb_handle_destroy(USBDevice* dev)
	{
		UePcbState* s = USB_CONTAINER_OF(dev, UePcbState, dev);
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
		}
		{
			std::lock_guard<std::mutex> lk(s->tcp_peers_lock);
			for (socket_t p : s->tcp_peers)
				if (p != UEPCB_INVALID_SOCKET)
					sock_close(p);
			s->tcp_peers.clear();
		}
		if (s->recv_thread.joinable())
			s->recv_thread.join();

		if (s->connection_mode == 0)
		{
			std::lock_guard<std::mutex> lock(s->jitter_lock);
			for (const auto& entry : s->peer_jitter)
			{
				const auto& jb = entry.second;
				Console.WriteLn(
					"UePcb DIAG summary peer=%llu RX=%llu Ahead=%llu Recovered=%llu ForcedSkip=%llu Late=%llu Duplicate=%llu MaxAhead=%u FinalBuffer=%u Queue=%zu",
					static_cast<unsigned long long>(entry.first),
					static_cast<unsigned long long>(jb.diag_rx),
					static_cast<unsigned long long>(jb.diag_ahead),
					static_cast<unsigned long long>(jb.diag_recovered),
					static_cast<unsigned long long>(jb.diag_forced_skip),
					static_cast<unsigned long long>(jb.diag_late),
					static_cast<unsigned long long>(jb.diag_duplicate),
					jb.diag_max_ahead, jb.target_packets, jb.packets.size());
			}
		}
		delete s;
	}

	USBDevice* UePcbDevice::CreateDevice(SettingsInterface& si, u32 port, u32 subtype) const
	{
		wsa_ensure();
		UePcbState* s = new UePcbState();
		{
			s->connection_mode = std::clamp(USB::GetConfigInt(si, port, TypeName(), "ConnectionMode", 0), 0, 1);
			s->advanced_settings = USB::GetConfigBool(si, port, TypeName(), "AdvancedSettings", false);
			s->broadcast_ip = s->advanced_settings ?
				USB::GetConfigString(si, port, TypeName(), "TargetIP", "255.255.255.255") : "255.255.255.255";
			s->udp_port = USB::GetConfigInt(si, port, TypeName(), "Port", 7500);
			s->tcp_port = s->udp_port;
			s->tcp_is_host = (USB::GetConfigInt(si, port, TypeName(), "TCPRole", 0) == 0);
			s->tcp_bind_ip = USB::GetConfigString(si, port, TypeName(), "TCPBindIP", "0.0.0.0");
			s->tcp_host_ip = USB::GetConfigString(si, port, TypeName(), "TCPHostIP", "127.0.0.1");
			const int tcp_history_slot = USB::GetConfigInt(si, port, TypeName(), "TCPHostHistorySlot", 0);

			// 1.3.3 manual adaptive jitter tuning. Clamp values defensively so a
			// typo cannot create an unbounded queue or excessive wait.
			const int grace_ms = s->advanced_settings ? std::clamp(USB::GetConfigInt(si, port, TypeName(), "JitterGraceMs", 3), 0, 20) : 3;
			const int decay_ms = s->advanced_settings ? std::clamp(USB::GetConfigInt(si, port, TypeName(), "JitterDecayMs", 4000), 250, 60000) : 4000;
			const int min_target = s->advanced_settings ? std::clamp(USB::GetConfigInt(si, port, TypeName(), "JitterMinTarget", 1), 1, 8) : 1;
			const int max_target = s->advanced_settings ? std::clamp(USB::GetConfigInt(si, port, TypeName(), "JitterMaxTarget", 4), min_target, 8) : 4;
			const int max_packets = s->advanced_settings ? std::clamp(USB::GetConfigInt(si, port, TypeName(), "JitterMaxPackets", 8), max_target, 32) : 8;
			s->jitter_grace = std::chrono::milliseconds(grace_ms);
			s->jitter_decay_interval = std::chrono::milliseconds(decay_ms);
			s->jitter_min_target = static_cast<u32>(min_target);
			s->jitter_max_target = static_cast<u32>(max_target);
			s->jitter_max_packets = static_cast<size_t>(max_packets);
			Console.WriteLn("UePcb: Jitter tuning Grace=%dms Decay=%dms Min=%d Max=%d Queue=%d",
				grace_ms, decay_ms, min_target, max_target, max_packets);

			// 1.3.3 shared Peer IP history. Boolean is now confirmed for native
			// Remember/Clear checkboxes; peer history selection remains numeric
			// until a dynamic editable list UI is intentionally added later.
			static constexpr int kHistorySlots = 10;
			std::array<std::string, kHistorySlots> history{};
			for (int i = 0; i < kHistorySlots; ++i)
			{
				const std::string key = "History" + std::to_string(i + 1);
				history[i] = USB::GetConfigString(si, port, TypeName(), key.c_str(), "");
			}

			if (tcp_history_slot >= 1 && tcp_history_slot <= kHistorySlots && !history[tcp_history_slot - 1].empty())
				s->tcp_host_ip = history[tcp_history_slot - 1];

			const std::string config_section = "USB" + std::to_string(port + 1);
			const auto real_key = [this](const std::string& key) {
				return std::string(TypeName()) + "_" + key;
			};

			const bool clear_history =
				USB::GetConfigBool(si, port, TypeName(), "ClearIPHistory", false);
			if (clear_history)
			{
				for (int i = 0; i < kHistorySlots; ++i)
				{
					history[i].clear();
					const std::string key = "History" + std::to_string(i + 1);
					si.SetStringValue(config_section.c_str(), real_key(key).c_str(), "");
				}
				for (int i = 0; i < 3; ++i)
				{
					const std::string key = "Peer" + std::to_string(i + 1) + "HistorySlot";
					si.SetUIntValue(config_section.c_str(), real_key(key).c_str(), 0);
				}
				si.SetUIntValue(config_section.c_str(), real_key("TCPHostHistorySlot").c_str(), 0);
				s->tcp_host_ip = USB::GetConfigString(si, port, TypeName(), "TCPHostIP", "127.0.0.1");
				si.SetBoolValue(config_section.c_str(), real_key("ClearIPHistory").c_str(), false);
				Console.WriteLn("UePcb: shared Peer IP history cleared");
			}

			std::array<std::string, 3> manual_peer_ips{};
			for (int i = 0; i < 3; ++i)
			{
				const std::string ip_key = "Peer" + std::to_string(i + 1) + "IP";
				manual_peer_ips[i] = USB::GetConfigString(si, port, TypeName(), ip_key.c_str(), "");
				s->peer_ips[i] = manual_peer_ips[i];

				const std::string slot_key = "Peer" + std::to_string(i + 1) + "HistorySlot";
				const int slot = USB::GetConfigInt(si, port, TypeName(), slot_key.c_str(), 0);
				if (slot >= 1 && slot <= kHistorySlots && !history[slot - 1].empty())
					s->peer_ips[i] = history[slot - 1];
			}

			const bool remember_history =
				USB::GetConfigBool(si, port, TypeName(), "RememberCurrentIPs", false);
			if (remember_history && !clear_history)
			{
				std::array<std::string, kHistorySlots> updated{};
				int used = 0;

				const auto add_unique = [&](const std::string& ip) {
					if (ip.empty() || used >= kHistorySlots)
						return;
					for (int j = 0; j < used; ++j)
					{
						if (updated[j] == ip)
							return;
					}
					updated[used++] = ip;
				};

				// Current resolved addresses become MRU entries. UDP stores Peer1-3;
				// TCP Client stores the Host IP in the same shared history pool.
				if (s->connection_mode == 1 && !s->tcp_is_host)
					add_unique(s->tcp_host_ip);
				else
					for (const std::string& ip : s->peer_ips)
						add_unique(ip);
				for (const std::string& ip : history)
					add_unique(ip);

				history = updated;
				for (int i = 0; i < kHistorySlots; ++i)
				{
					const std::string key = "History" + std::to_string(i + 1);
					si.SetStringValue(config_section.c_str(), real_key(key).c_str(), history[i].c_str());
				}
				si.SetBoolValue(config_section.c_str(), real_key("RememberCurrentIPs").c_str(), false);
				Console.WriteLn("UePcb: remembered current Peer IPs into shared history");
			}

			for (int i = 0; i < 3; ++i)
			{
				Console.WriteLn("UePcb: Peer%d resolved IP = %s", i + 1,
					s->peer_ips[i].empty() ? "<empty>" : s->peer_ips[i].c_str());
			}

			const std::string mh = USB::GetConfigString(si, port, TypeName(), "MacHex", "");
			if (mh.size() >= 12)
			{
				const auto hx = [](char c) -> int {
					if (c >= '0' && c <= '9') return c - '0';
					if (c >= 'a' && c <= 'f') return c - 'a' + 10;
					if (c >= 'A' && c <= 'F') return c - 'A' + 10;
					return 0;
				};
				for (int i = 0; i < 6; i++)
					s->mac[i] = static_cast<u8>((hx(mh[i * 2]) << 4) | hx(mh[i * 2 + 1]));
			}
			else
			{
				static std::mt19937 rng(std::random_device{}());
				const u32 r = rng();
				s->mac[0] = 0x00; s->mac[1] = 0x90; s->mac[2] = 0x2E;
				s->mac[3] = static_cast<u8>(r & 0xFF);
				s->mac[4] = static_cast<u8>((r >> 8) & 0xFF);
				s->mac[5] = static_cast<u8>((r >> 16) & 0xFF);
			}

			if (s->connection_mode == 0)
			{
			s->udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
			if (s->udp_sock != UEPCB_INVALID_SOCKET)
			{
				int one = 1;
				setsockopt(s->udp_sock, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&one), sizeof(one));
				setsockopt(s->udp_sock, SOL_SOCKET, SO_BROADCAST, reinterpret_cast<const char*>(&one), sizeof(one));

				// Give the kernel receive buffer more headroom than the tiny
				// OS default. Under a burst (host briefly stalls a frame,
				// e.g. a hitch from disk I/O or GC), a small kernel buffer
				// overflows and silently drops datagrams before
				// udp_recv_loop() ever reads them - a loss that's invisible
				// even to the new seq-gap logging above, since the packet
				// never reaches userland at all.
				int rcvbuf = 256 * 1024;
				setsockopt(s->udp_sock, SOL_SOCKET, SO_RCVBUF, reinterpret_cast<const char*>(&rcvbuf), sizeof(rcvbuf));

				sockaddr_in bind_addr{};
				bind_addr.sin_family = AF_INET;
				bind_addr.sin_port = htons(static_cast<u16>(s->udp_port));
				bind_addr.sin_addr.s_addr = INADDR_ANY;

				bind(s->udp_sock, reinterpret_cast<sockaddr*>(&bind_addr), sizeof(bind_addr));
			}
			}
			else
			{
				Console.WriteLn("UePcb: TCP mode role=%s port=%d %s=%s",
					s->tcp_is_host ? "HOST" : "CLIENT", s->tcp_port,
					s->tcp_is_host ? "listen" : "host",
					s->tcp_is_host ? s->tcp_bind_ip.c_str() : s->tcp_host_ip.c_str());
			}

		}

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
		s->recv_thread = (s->connection_mode == 1) ? std::thread(tcp_transport_loop, s) : std::thread(udp_recv_loop, s);
		return &s->dev;

	fail:
		uepcb_handle_destroy(&s->dev);
		return nullptr;
	}

	const char* UePcbDevice::Name() const { return "UE PCB (Namco arcade TCP/UDP) 1.5"; }
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
		static const char* connection_modes[] = {"UDP", "TCP", nullptr};
		static const char* tcp_roles[] = {"Host", "Client", nullptr};
		static const SettingInfo settings[] = {
			{.type = SettingInfo::Type::IntegerList, .name = "ConnectionMode", .display_name = "Connection Mode",
				.description = "UDP is the low-latency Direct Peer/Broadcast mode. TCP uses a Host/Client hub.",
				.default_value = "0", .min_value = "0", .options = connection_modes},
			{.type = SettingInfo::Type::Integer, .name = "Port", .display_name = "Network Port",
				.description = "UDP or TCP port used by every player. Default: 7500.",
				.default_value = "7500", .min_value = "1", .max_value = "65535", .step_value = "1"},

			{.type = SettingInfo::Type::String, .name = "TargetIP", .display_name = "Broadcast Address",
				.description = "UDP only. Used when all Direct Peer IPs are empty. Default: 255.255.255.255.", .default_value = "255.255.255.255"},
			{.type = SettingInfo::Type::String, .name = "Peer1IP", .display_name = "Direct Peer 1 IP", .description = "UDP manual peer address.", .default_value = ""},
			{.type = SettingInfo::Type::Integer, .name = "Peer1HistorySlot", .display_name = "Peer 1 Saved IP", .description = "0=Manual, 1-10=saved.", .default_value = "0", .min_value = "0", .max_value = "10", .step_value = "1"},
			{.type = SettingInfo::Type::String, .name = "Peer2IP", .display_name = "Direct Peer 2 IP", .description = "UDP manual peer address.", .default_value = ""},
			{.type = SettingInfo::Type::Integer, .name = "Peer2HistorySlot", .display_name = "Peer 2 Saved IP", .description = "0=Manual, 1-10=saved.", .default_value = "0", .min_value = "0", .max_value = "10", .step_value = "1"},
			{.type = SettingInfo::Type::String, .name = "Peer3IP", .display_name = "Direct Peer 3 IP", .description = "UDP manual peer address.", .default_value = ""},
			{.type = SettingInfo::Type::Integer, .name = "Peer3HistorySlot", .display_name = "Peer 3 Saved IP", .description = "0=Manual, 1-10=saved.", .default_value = "0", .min_value = "0", .max_value = "10", .step_value = "1"},

			{.type = SettingInfo::Type::IntegerList, .name = "TCPRole", .display_name = "TCP Role",
				.description = "Host listens and relays frames to all Clients. Client connects to the Host.", .default_value = "0", .min_value = "0", .options = tcp_roles},
			{.type = SettingInfo::Type::String, .name = "TCPBindIP", .display_name = "Listen IP",
				.description = "TCP Host only. 0.0.0.0 listens on all LAN/VPN adapters; 127.0.0.1 limits it to this PC.", .default_value = "0.0.0.0"},
			{.type = SettingInfo::Type::String, .name = "TCPHostIP", .display_name = "TCP Host IP",
				.description = "TCP Client only. Same PC: 127.0.0.1. LAN/VPN: enter the Host machine's LAN or VPN IP.", .default_value = "127.0.0.1"},
			{.type = SettingInfo::Type::Integer, .name = "TCPHostHistorySlot", .display_name = "TCP Host Saved IP",
				.description = "0=Manual TCP Host IP, 1-10=shared saved address.", .default_value = "0", .min_value = "0", .max_value = "10", .step_value = "1"},

			{.type = SettingInfo::Type::Boolean, .name = "RememberCurrentIPs", .display_name = "Remember Current IPs",
				.description = "Check and Apply/Save. UDP remembers resolved Peer1-3; TCP Client remembers the Host IP. Resets automatically.", .default_value = "false"},
			{.type = SettingInfo::Type::Boolean, .name = "ClearIPHistory", .display_name = "Clear Shared IP History",
				.description = "Check and Apply/Save to clear all 10 saved IPs and reset selectors to Manual. Resets automatically.", .default_value = "false"},

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
				.description = "Enable manual UDP jitter tuning. When disabled, the proven defaults are forced: Grace 3, Decay 4000, Min 1, Max 4, Queue 8.", .default_value = "false"},
			{.type = SettingInfo::Type::Integer, .name = "JitterGraceMs", .display_name = "Jitter Grace (ms)", .description = "UDP only. Missing-sequence grace before forced skip.", .default_value = "3", .min_value = "0", .max_value = "20", .step_value = "1"},
			{.type = SettingInfo::Type::Integer, .name = "JitterDecayMs", .display_name = "Buffer Decay (ms)", .description = "UDP only. Stable time before target buffer relaxes by one.", .default_value = "4000", .min_value = "250", .max_value = "60000", .step_value = "250"},
			{.type = SettingInfo::Type::Integer, .name = "JitterMinTarget", .display_name = "Minimum Target Buffer", .description = "UDP only. Default 1.", .default_value = "1", .min_value = "1", .max_value = "8", .step_value = "1"},
			{.type = SettingInfo::Type::Integer, .name = "JitterMaxTarget", .display_name = "Maximum Target Buffer", .description = "UDP only. Default 4.", .default_value = "4", .min_value = "1", .max_value = "8", .step_value = "1"},
			{.type = SettingInfo::Type::Integer, .name = "JitterMaxPackets", .display_name = "Maximum Jitter Queue", .description = "UDP only. Default 8; keep small to avoid latency buildup.", .default_value = "8", .min_value = "1", .max_value = "32", .step_value = "1"},
			{.type = SettingInfo::Type::String, .name = "MacHex", .display_name = "MAC 12-hex", .description = "Leave blank to auto-generate a unique MAC per emulator instance.", .default_value = ""}};
		return settings;
	}

} // namespace usb_uepcb