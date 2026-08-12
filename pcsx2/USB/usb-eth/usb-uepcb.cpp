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
#include <random>
#include <string>
#include <thread>
#include <algorithm>

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
		"", "Namco", "UE PCB v2.6", ""};

	typedef struct UePcbState
	{
		USBDevice dev{};
		USBDesc desc{};
		USBDescDevice desc_dev{};

		u8 an986_regs[0x40] = {};
		u8 eeprom_word = 0;
		std::vector<u8> tx_accum;

		std::vector<u8> rx_partial;
		size_t rx_offset = 0;

		bool is_host = false;
		u8 mac[6] = {0x00, 0x90, 0x2E, 0x11, 0x22, 0x33};
		int peer_port = 7500;
		std::string peer_ip;
		std::string bind_ip;
		u8 mii_phyaddr = 1;
		u8 mii_reg = 1;
		bool init_done = false;
		bool link_event_sent = false;
		std::mutex in_lock;
		std::deque<std::vector<u8>> in_q;
		std::mutex sock_lock;
		socket_t listen_sock = UEPCB_INVALID_SOCKET;
		std::thread peer_thread;
		std::atomic<bool> peer_stop{false};
		std::mutex peers_lock;
		std::vector<socket_t> peers;
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

	static bool ip_is_own(const std::string& ip)
	{
		if (ip.empty() || ip == "0.0.0.0" || ip.rfind("127.", 0) == 0)
			return false;
		wsa_ensure();
		sockaddr_in a{};
		a.sin_family = AF_INET;
		a.sin_port = 0;
		if (inet_pton(AF_INET, ip.c_str(), &a.sin_addr) != 1)
			return false;
		socket_t t = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (t == UEPCB_INVALID_SOCKET)
			return false;
		const bool local = (bind(t, reinterpret_cast<sockaddr*>(&a), sizeof(a)) == 0);
		sock_close(t);
		return local;
	}

	static void set_fast_socket(socket_t c)
	{
		int nd = 1;
		setsockopt(c, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&nd), sizeof(nd));
#ifdef _WIN32
		DWORD tv = 500;
		setsockopt(c, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&tv), sizeof(tv));
		setsockopt(c, SOL_SOCKET, SO_SNDTIMEO, reinterpret_cast<const char*>(&tv), sizeof(tv));
#else
		struct timeval tv;
		tv.tv_sec = 0;
		tv.tv_usec = 500000;
		setsockopt(c, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&tv), sizeof(tv));
		setsockopt(c, SOL_SOCKET, SO_SNDTIMEO, reinterpret_cast<const char*>(&tv), sizeof(tv));
#endif
	}

	static bool recv_exact(socket_t c, u8* buf, int len)
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

	static bool send_exact(socket_t c, const u8* buf, int len)
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

	// [v2.6 修復] 安全的防積壓佇列 (Safe Queue FIFO)
	static void push_in_q(UePcbState* s, std::vector<u8> v)
	{
		std::lock_guard<std::mutex> lk(s->in_lock);
		// 只有在初始化完成後才進行彈性平滑，且只剔除最舊的一個，絕不一次清空
		if (s->init_done && s->in_q.size() >= 8)
		{
			s->in_q.pop_front();
		}
		else if (!s->init_done && s->in_q.size() >= 1024)
		{
			s->in_q.pop_front();
		}
		s->in_q.push_back(std::move(v));
	}

	static bool pop_in_q(UePcbState* s, std::vector<u8>& out)
	{
		std::lock_guard<std::mutex> lk(s->in_lock);
		if (s->in_q.empty())
			return false;
		out = std::move(s->in_q.front());
		s->in_q.pop_front();
		return true;
	}

	static void peer_recv_loop(UePcbState* s, socket_t conn)
	{
		while (!s->peer_stop)
		{
			u8 hdr[4];
			if (!recv_exact(conn, hdr, 4))
				break;
			const u32 ln = (hdr[0] << 24) | (hdr[1] << 16) | (hdr[2] << 8) | hdr[3];
			if (ln < 14 || ln > 2048)
				break;
			std::vector<u8> eth(ln);
			if (!recv_exact(conn, eth.data(), ln))
				break;
			push_in_q(s, to_bulkin(eth.data(), static_cast<int>(eth.size())));
		}
	}

	static void peer_send_all(UePcbState* s, const u8* eth, int len)
	{
		const u8 hdr[4] = {static_cast<u8>((len >> 24) & 0xFF), static_cast<u8>((len >> 16) & 0xFF),
			static_cast<u8>((len >> 8) & 0xFF), static_cast<u8>(len & 0xFF)};
		std::lock_guard<std::mutex> lk(s->peers_lock);
		for (socket_t p : s->peers)
		{
			if (p != UEPCB_INVALID_SOCKET)
			{
				send_exact(p, hdr, 4);
				send_exact(p, eth, len);
			}
		}
	}

	static void hub_remove(UePcbState* s, socket_t p)
	{
		std::lock_guard<std::mutex> lk(s->peers_lock);
		for (auto it = s->peers.begin(); it != s->peers.end(); ++it)
		{
			if (*it == p)
			{
				s->peers.erase(it);
				break;
			}
		}
		sock_close(p);
	}

	static void hub_flood(UePcbState* s, socket_t src, const u8* eth, int len)
	{
		const u8 hdr[4] = {static_cast<u8>((len >> 24) & 0xFF), static_cast<u8>((len >> 16) & 0xFF),
			static_cast<u8>((len >> 8) & 0xFF), static_cast<u8>(len & 0xFF)};
		std::lock_guard<std::mutex> lk(s->peers_lock);
		for (socket_t p : s->peers)
		{
			if (p != src && p != UEPCB_INVALID_SOCKET)
			{
				send_exact(p, hdr, 4);
				send_exact(p, eth, len);
			}
		}
	}

	static void hub_join_loop(UePcbState* s)
	{
		while (!s->peer_stop)
		{
			socket_t conn = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
			if (conn != UEPCB_INVALID_SOCKET)
			{
				set_fast_socket(conn);
				sockaddr_in a{};
				a.sin_family = AF_INET;
				a.sin_port = htons(static_cast<u16>(s->peer_port));
				inet_pton(AF_INET, s->peer_ip.c_str(), &a.sin_addr);
				if (connect(conn, reinterpret_cast<sockaddr*>(&a), sizeof(a)) == 0)
				{
					{
						std::lock_guard<std::mutex> lk(s->peers_lock);
						s->peers.assign(1, conn);
					}
					peer_recv_loop(s, conn);
					{
						std::lock_guard<std::mutex> lk(s->peers_lock);
						if (!s->peers.empty())
						{
							sock_close(s->peers[0]);
							s->peers.clear();
						}
					}
				}
				else
					sock_close(conn);
			}
			for (int i = 0; i < 10 && !s->peer_stop; i++)
				std::this_thread::sleep_for(std::chrono::milliseconds(50));
		}
	}

	static void hub_host_loop(UePcbState* s)
	{
		socket_t ls = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (ls == UEPCB_INVALID_SOCKET)
			return;
		int one = 1;
		setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&one), sizeof(one));
		sockaddr_in a{};
		a.sin_family = AF_INET;
		a.sin_port = htons(static_cast<u16>(s->peer_port));
		inet_pton(AF_INET, s->bind_ip.c_str(), &a.sin_addr);
		if (bind(ls, reinterpret_cast<sockaddr*>(&a), sizeof(a)) != 0)
		{
			sock_close(ls);
			return;
		}
		listen(ls, 8);
		{
			std::lock_guard<std::mutex> lk(s->sock_lock);
			s->listen_sock = ls;
		}
		while (!s->peer_stop)
		{
			fd_set rf;
			FD_ZERO(&rf);
			FD_SET(ls, &rf);
			socket_t maxfd = ls;
			{
				std::lock_guard<std::mutex> lk(s->peers_lock);
				for (socket_t p : s->peers)
				{
					FD_SET(p, &rf);
					if (p > maxfd)
						maxfd = p;
				}
			}
			timeval tv{0, 10000};
			const int r = select(static_cast<int>(maxfd) + 1, &rf, nullptr, nullptr, &tv);
			if (s->peer_stop)
				break;
			if (r <= 0)
				continue;
			if (FD_ISSET(ls, &rf))
			{
				socket_t conn = accept(ls, nullptr, nullptr);
				if (conn != UEPCB_INVALID_SOCKET)
				{
					set_fast_socket(conn);
					std::lock_guard<std::mutex> lk(s->peers_lock);
					s->peers.push_back(conn);
				}
			}
			std::vector<socket_t> snap;
			{
				std::lock_guard<std::mutex> lk(s->peers_lock);
				snap = s->peers;
			}
			for (socket_t p : snap)
			{
				if (!FD_ISSET(p, &rf))
					continue;
				u8 hdr[4];
				if (!recv_exact(p, hdr, 4))
				{
					hub_remove(s, p);
					continue;
				}
				const u32 ln = (hdr[0] << 24) | (hdr[1] << 16) | (hdr[2] << 8) | hdr[3];
				if (ln < 14 || ln > 2048)
				{
					hub_remove(s, p);
					continue;
				}
				std::vector<u8> eth(ln);
				if (!recv_exact(p, eth.data(), ln))
				{
					hub_remove(s, p);
					continue;
				}
				push_in_q(s, to_bulkin(eth.data(), static_cast<int>(ln)));
				hub_flood(s, p, eth.data(), static_cast<int>(ln));
			}
		}
		std::lock_guard<std::mutex> lk(s->peers_lock);
		for (socket_t p : s->peers)
			if (p != UEPCB_INVALID_SOCKET)
				sock_close(p);
		s->peers.clear();
	}

	static void peer_thread_main(UePcbState* s)
	{
		wsa_ensure();
		if (s->is_host)
			hub_host_loop(s);
		else
			hub_join_loop(s);
	}

#ifdef _WIN32
	static void open_firewall_port(int port)
	{
		char params[256];
		std::snprintf(params, sizeof(params),
			"advfirewall firewall add rule name=\"uepcb-netplay-%d\" dir=in action=allow protocol=TCP localport=%d",
			port, port);
		SHELLEXECUTEINFOA sei{};
		sei.cbSize = sizeof(sei);
		sei.fMask = SEE_MASK_NOCLOSEPROCESS;
		sei.lpVerb = "runas";
		sei.lpFile = "netsh";
		sei.lpParameters = params;
		sei.nShow = SW_HIDE;
		if (ShellExecuteExA(&sei) && sei.hProcess)
		{
			WaitForSingleObject(sei.hProcess, 5000);
			CloseHandle(sei.hProcess);
		}
	}
#endif

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
					data[3] |= 0x60;
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
		const u8 ep = p->ep->nr;

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

		switch (p->pid)
		{
			case USB_TOKEN_OUT:
			{
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
					peer_send_all(s, buf + 2, ethlen);
				}
				break;
			}
			case USB_TOKEN_IN:
			{
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
					std::vector<u8> frame;
					if (pop_in_q(s, frame) && !frame.empty())
					{
						s->rx_partial = std::move(frame);
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
		s->peer_stop = true;
		{
			std::lock_guard<std::mutex> lk(s->sock_lock);
			if (s->listen_sock != UEPCB_INVALID_SOCKET)
			{
				sock_close(s->listen_sock);
				s->listen_sock = UEPCB_INVALID_SOCKET;
			}
		}
		{
			std::lock_guard<std::mutex> lk(s->peers_lock);
			for (socket_t p : s->peers)
				if (p != UEPCB_INVALID_SOCKET)
					sock_close(p);
			s->peers.clear();
		}
		if (s->peer_thread.joinable())
			s->peer_thread.join();
		delete s;
	}

	USBDevice* UePcbDevice::CreateDevice(SettingsInterface& si, u32 port, u32 subtype) const
	{
		UePcbState* s = new UePcbState();
		{
			const std::string peer = USB::GetConfigString(si, port, TypeName(), "PeerIP", "");
			const bool has_peer = !peer.empty() && peer != "0.0.0.0";
			s->is_host = !has_peer || ip_is_own(peer);
			s->peer_port = USB::GetConfigInt(si, port, TypeName(), "Port", 7500);
			if (s->is_host)
			{
				s->bind_ip = USB::GetConfigString(si, port, TypeName(), "BindIP", "0.0.0.0");
#ifdef _WIN32
				open_firewall_port(s->peer_port);
#endif
			}
			else
				s->peer_ip = peer;

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
		s->peer_thread = std::thread(peer_thread_main, s);
		return &s->dev;

	fail:
		uepcb_handle_destroy(&s->dev);
		return nullptr;
	}

	const char* UePcbDevice::Name() const { return "UE PCB (Namco arcade net v2.6)"; }
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
				.name = "PeerIP",
				.display_name = "1P's IP (same value on every PC)",
				.description = "Cross-PC: enter the 1P's LAN IP here on EVERY PC (same value). "
							   "The PC whose own IP matches auto-becomes 1P (listens); the rest (2P-4P) connect. "
							   "Same-PC: leave blank on 1P, 127.0.0.1 on the others.",
				.default_value = ""},
			{.type = SettingInfo::Type::String,
				.name = "BindIP",
				.display_name = "Bind IP (1P listen)",
				.description = "1P listen address. 0.0.0.0 = all adapters (LAN/VPN); "
							   "127.0.0.1 = same PC only. Ignored on 2P-4P.",
				.default_value = "0.0.0.0"},
			{.type = SettingInfo::Type::Integer,
				.name = "Port",
				.display_name = "TCP Port",
				.description = "TCP port for the peer link (1P opens it and adds a firewall rule automatically).",
				.default_value = "7500",
				.min_value = "1",
				.max_value = "65535",
				.step_value = "1"},
			{.type = SettingInfo::Type::String,
				.name = "MacHex",
				.display_name = "MAC 12-hex (optional override)",
				.description = "Leave BLANK - each emu auto-picks a unique MAC (00902e + random). "
							   "Only set this (12 hex) if you need a fixed MAC.",
				.default_value = ""}};
		return settings;
	}
} // namespace usb_uepcb