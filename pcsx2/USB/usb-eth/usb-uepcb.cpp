// SPDX-FileCopyrightText: 2002-2026 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+

#include "USB/qemu-usb/qusb.h"
#include "USB/qemu-usb/desc.h"
#include "USB/qemu-usb/USBinternal.h"
#include "USB/usb-eth/usb-uepcb.h"
#include "USB/USB.h"
#include "common/Console.h"
#include "StateWrapper.h"
#include "IopMem.h"

#include <cstring>
#include <cstdio>
#include <vector>
#include <atomic>
#include <chrono>
#include <deque>
#include <mutex>
#include <condition_variable>
#include <random>
#include <string>
#include <thread>
#include <algorithm>
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
		"", "Namco", "UE PCB v3.2 (NAT Traversal + Dynamic Sync)", ""};

	static constexpr int kWireTrailerSize = sizeof(u32);
	static constexpr u32 kNatHolePunchMagic = 0xFFFFFFFF; // UDP 打洞魔術字串

	typedef struct UePcbState
	{
		USBDevice dev{};
		USBDesc desc{};
		USBDescDevice desc_dev{};

		u8 an986_regs[0x40] = {};
		u8 eeprom_word = 0;
		std::vector<u8> tx_accum;

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
		std::condition_variable cv_rx;
		std::deque<std::vector<u8>> in_q;

		std::atomic<u32> tx_seq{0};
		std::unordered_map<u64, u32> peer_last_seq;
		u32 loss_log_suppress = 0;

		socket_t udp_sock = UEPCB_INVALID_SOCKET;
		std::thread recv_thread;
		std::thread keepalive_thread; // UDP 打洞心跳執行緒
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

	// 背景心跳執行緒：每 1 秒發送一次 NAT 打洞封包
	static void udp_keepalive_loop(UePcbState* s)
	{
		const u32 hole_punch_pkt = kNatHolePunchMagic;
		while (!s->thread_stop)
		{
			for (const std::string& ip : s->peer_ips)
			{
				if (!ip.empty())
				{
					udp_send_to(s, ip, reinterpret_cast<const u8*>(&hole_punch_pkt), sizeof(hole_punch_pkt));
				}
			}
			std::this_thread::sleep_for(std::chrono::seconds(1));
		}
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
			int n = recvfrom(s->udp_sock, reinterpret_cast<char*>(buf), sizeof(buf), 0,
				reinterpret_cast<sockaddr*>(&src_addr), &addr_len);

			if (s->thread_stop)
				break;

			// 攔截並過濾 NAT 打洞心跳封包 (4 Bytes = Magic)
			if (n == sizeof(u32))
			{
				u32 magic = 0;
				std::memcpy(&magic, buf, sizeof(magic));
				if (magic == kNatHolePunchMagic)
					continue; // 成功打洞 / 維持通訊埠，不餵給網卡
			}

			if (n < (int)(14 + kWireTrailerSize))
				continue;

			const int ethlen = n - kWireTrailerSize;

			// 防自我環回 (Filter Out Self Packets by MAC)
			if (std::memcmp(buf + 6, s->mac, 6) == 0)
				continue;

			// 推入接收佇列並即時喚醒 USB 執行緒
			{
				std::lock_guard<std::mutex> lk(s->pending_lock);
				s->pending_rx.push_back(to_bulkin(buf, ethlen));
			}
			s->cv_rx.notify_one();
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

	static void udp_send_packet(UePcbState* s, const u8* eth, int len)
	{
		if (s->udp_sock == UEPCB_INVALID_SOCKET || len + kWireTrailerSize > 2048)
			return;

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

				if (index == 0x20 && length >= 2)
				{
					data[0] = 0x2D; data[1] = 0x78;
				}
				else if (index == 0x25 && length >= 2)
				{
					data[1] |= 0x24;
				}
				else if (index == 0x2B && length >= 4)
				{
					data[0] = 0x00; data[1] = 0x00; data[2] = 0x00; data[3] = 0x60;
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

					udp_send_packet(s, buf + 2, ethlen);
				}
				break;
			}
			case USB_TOKEN_IN:
			{
				if (ep == 3)
				{
					p->status = USB_RET_NAK;
					break;
				}
				if (ep != 1)
				{
					p->status = USB_RET_STALL;
					break;
				}

				// 1. 將 pending 封包推進主佇列
				{
					std::lock_guard<std::mutex> lk(s->pending_lock);
					std::lock_guard<std::mutex> qlk(s->in_lock);
					while (!s->pending_rx.empty())
					{
						s->in_q.push_back(std::move(s->pending_rx.front()));
						s->pending_rx.pop_front();
					}
				}

				// 2. 平滑微秒同步 (非阻塞 Wait)
				if (s->rx_partial.empty())
				{
					std::unique_lock<std::mutex> lk(s->in_lock);
					if (s->in_q.empty())
					{
						// 最多微秒級等待 5ms，若有 UDP 包立刻喚醒，絕不引發 Lockstep 卡死
						s->cv_rx.wait_for(lk, std::chrono::milliseconds(5), [&]() {
							std::lock_guard<std::mutex> plk(s->pending_lock);
							return !s->pending_rx.empty() || !s->in_q.empty() || s->thread_stop;
						});

						std::lock_guard<std::mutex> plk(s->pending_lock);
						while (!s->pending_rx.empty())
						{
							s->in_q.push_back(std::move(s->pending_rx.front()));
							s->pending_rx.pop_front();
						}
					}

					if (!s->in_q.empty())
					{
						s->rx_partial = std::move(s->in_q.front());
						s->in_q.pop_front();
						s->rx_offset = 0;
					}
				}

				// 3. USB 數據拷貝
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
					p->status = USB_RET_NAK;
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
		s->cv_rx.notify_all();

		if (s->udp_sock != UEPCB_INVALID_SOCKET)
		{
			sock_close(s->udp_sock);
			s->udp_sock = UEPCB_INVALID_SOCKET;
		}
		if (s->recv_thread.joinable())
			s->recv_thread.join();
		if (s->keepalive_thread.joinable())
			s->keepalive_thread.join();
		delete s;
	}

	USBDevice* UePcbDevice::CreateDevice(SettingsInterface& si, u32 port, u32 subtype) const
	{
		wsa_ensure();
		UePcbState* s = new UePcbState();
		{
			s->broadcast_ip = USB::GetConfigString(si, port, TypeName(), "TargetIP", "255.255.255.255");
			s->udp_port = USB::GetConfigInt(si, port, TypeName(), "Port", 7500);

			for (int i = 0; i < 3; ++i)
			{
				const std::string key = "Peer" + std::to_string(i + 1) + "IP";
				s->peer_ips[i] = USB::GetConfigString(si, port, TypeName(), key.c_str(), "");
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

			s->udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
			if (s->udp_sock != UEPCB_INVALID_SOCKET)
			{
				int one = 1;
				setsockopt(s->udp_sock, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&one), sizeof(one));
				setsockopt(s->udp_sock, SOL_SOCKET, SO_BROADCAST, reinterpret_cast<const char*>(&one), sizeof(one));

				int rcvbuf = 256 * 1024;
				setsockopt(s->udp_sock, SOL_SOCKET, SO_RCVBUF, reinterpret_cast<const char*>(&rcvbuf), sizeof(rcvbuf));

				sockaddr_in bind_addr{};
				bind_addr.sin_family = AF_INET;
				bind_addr.sin_port = htons(static_cast<u16>(s->udp_port));
				bind_addr.sin_addr.s_addr = INADDR_ANY;

				bind(s->udp_sock, reinterpret_cast<sockaddr*>(&bind_addr), sizeof(bind_addr));
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

		s->recv_thread = std::thread(udp_recv_loop, s);
		s->keepalive_thread = std::thread(udp_keepalive_loop, s); // 啟動打洞線程

		return &s->dev;

	fail:
		uepcb_handle_destroy(&s->dev);
		return nullptr;
	}

	const char* UePcbDevice::Name() const { return "UE PCB (Namco arcade Direct UDP v3.2)"; }
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
		static const SettingInfo settings[] = {
			{.type = SettingInfo::Type::String,
				.name = "TargetIP",
				.display_name = "Target Broadcast / Subnet IP (LAN Mode)",
				.description = "Used when Direct Peer IPs are empty. Default: 255.255.255.255",
				.default_value = "255.255.255.255"},
			{.type = SettingInfo::Type::Integer,
				.name = "Port",
				.display_name = "UDP Port",
				.description = "UDP port for LAN / Internet P2P communication (Default: 7500).",
				.default_value = "7500",
				.min_value = "1",
				.max_value = "65535",
				.step_value = "1"},
			{.type = SettingInfo::Type::String,
				.name = "Peer1IP",
				.display_name = "Direct Peer 1 IP (Public / Remote)",
				.description = "Enter IPv4 address of Remote Player 1. Auto UDP Hole Punch enabled.",
				.default_value = ""},
			{.type = SettingInfo::Type::String,
				.name = "Peer2IP",
				.display_name = "Direct Peer 2 IP (Public / Remote)",
				.description = "Enter IPv4 address of Remote Player 2. Leave blank if unused.",
				.default_value = ""},
			{.type = SettingInfo::Type::String,
				.name = "Peer3IP",
				.display_name = "Direct Peer 3 IP (Public / Remote)",
				.description = "Enter IPv4 address of Remote Player 3. Leave blank if unused.",
				.default_value = ""},
			{.type = SettingInfo::Type::String,
				.name = "MacHex",
				.display_name = "MAC 12-hex (optional override)",
				.description = "Leave BLANK - auto generates unique MAC per emulator.",
				.default_value = ""}};
		return settings;
	}
} // namespace usb_uepcb