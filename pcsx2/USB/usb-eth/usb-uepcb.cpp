/*
 * PCSX2 - PS2 Emulator for PCs
 * Copyright (C) 2002-2026  PCSX2 Dev Team
 *
 * Special Patch for System 256 / UE PCB (AN986 USB Fast Ethernet)
 * Complete implementation with OHCI DMA crash prevention, netplay sync, and AN986 logic.
 */

#include "PrecompiledHeader.h"
#include "usb-uepcb.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
using socket_t = SOCKET;
#define IS_VALID_SOCKET(s) ((s) != INVALID_SOCKET)
#define CLOSE_SOCKET(s) closesocket(s)
#else
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
using socket_t = int;
#define INVALID_SOCKET (-1)
#define IS_VALID_SOCKET(s) ((s) >= 0)
#define CLOSE_SOCKET(s) close(s)
#endif

// -----------------------------------------------------------------------------
// Constants & Structures
// -----------------------------------------------------------------------------
constexpr size_t MAX_PEERS = 8;
constexpr size_t MAX_PKT_SIZE = 2048;

struct UePcbPeer
{
	socket_t fd = INVALID_SOCKET;
	std::string ip;
	uint16_t port = 0;
	bool connected = false;
};

struct UePcbState
{
	USBDevice dev;

	// Netplay sockets & threads
	socket_t listen_fd = INVALID_SOCKET;
	std::array<UePcbPeer, MAX_PEERS> peers;
	std::atomic<bool> running{false};
	std::thread net_thread;

	// Packet Queues
	std::mutex in_lock;
	std::deque<std::vector<u8>> in_q;

	std::vector<u8> tx_accum;
	std::vector<u8> current_rx_chunk;
	size_t current_rx_offset = 0;

	// AN986 Internal State Registers
	u8 registers[0x100]{};
	u8 phy_registers[0x20]{};
	u8 mac_addr[6]{0x00, 0x90, 0x2E, 0xED, 0xC6, 0xD6};

	int port_num = 0;
};

// -----------------------------------------------------------------------------
// Forward Declarations
// -----------------------------------------------------------------------------
static void netplay_init(UePcbState* s, int port);
static void netplay_close(UePcbState* s);
static void peer_send_all(UePcbState* s, const u8* data, size_t len);

// -----------------------------------------------------------------------------
// Network Loop & Helper Functions
// -----------------------------------------------------------------------------
static void set_nonblocking(socket_t fd)
{
#ifdef _WIN32
	u_long mode = 1;
	ioctlsocket(fd, FIONBIO, &mode);
#else
	int flags = fcntl(fd, F_GETFL, 0);
	fcntl(fd, F_SETFL, flags | O_NONBLOCK);
#endif
}

static void set_nodelay(socket_t fd)
{
	int one = 1;
	setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, (const char*)&one, sizeof(one));
}

static void netplay_thread_proc(UePcbState* s)
{
	while (s->running)
	{
		fd_set readfds;
		FD_ZERO(&readfds);

		socket_t max_fd = s->listen_fd;
		if (IS_VALID_SOCKET(s->listen_fd))
		{
			FD_SET(s->listen_fd, &readfds);
		}

		for (const auto& peer : s->peers)
		{
			if (peer.connected && IS_VALID_SOCKET(peer.fd))
			{
				FD_SET(peer.fd, &readfds);
				if (peer.fd > max_fd)
					max_fd = peer.fd;
			}
		}

		struct timeval tv;
		tv.tv_sec = 0;
		tv.tv_usec = 10000; // 10ms timeout

		int activity = select((int)max_fd + 1, &readfds, nullptr, nullptr, &tv);

		if (activity < 0)
		{
			if (!s->running) break;
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			continue;
		}

		// Accept new peer connection
		if (IS_VALID_SOCKET(s->listen_fd) && FD_ISSET(s->listen_fd, &readfds))
		{
			sockaddr_in cli_addr{};
			socklen_t clilen = sizeof(cli_addr);
			socket_t new_fd = accept(s->listen_fd, (sockaddr*)&cli_addr, &clilen);

			if (IS_VALID_SOCKET(new_fd))
			{
				set_nonblocking(new_fd);
				set_nodelay(new_fd);

				bool accepted = false;
				for (auto& peer : s->peers)
				{
					if (!peer.connected)
					{
						peer.fd = new_fd;
						peer.connected = true;
						peer.ip = inet_ntoa(cli_addr.sin_addr);
						peer.port = ntohs(cli_addr.sin_port);
						accepted = true;
						Console.WriteLn("UePCB: netplay peer connected from %s:%d", peer.ip.c_str(), peer.port);
						break;
					}
				}

				if (!accepted)
				{
					CLOSE_SOCKET(new_fd);
				}
			}
		}

		// Read incoming network packets
		for (auto& peer : s->peers)
		{
			if (peer.connected && IS_VALID_SOCKET(peer.fd) && FD_ISSET(peer.fd, &readfds))
			{
				u8 buf[MAX_PKT_SIZE];
				int bytes = recv(peer.fd, (char*)buf, sizeof(buf), 0);

				if (bytes > 0)
				{
					std::vector<u8> pkt(buf, buf + bytes);
					std::lock_guard<std::mutex> lk(s->in_lock);
					s->in_q.push_back(std::move(pkt));
					if (s->in_q.size() > 500)
					{
						s->in_q.pop_front(); // Drop oldest on overflow
					}
				}
				else if (bytes == 0 || (bytes < 0 &&
#ifdef _WIN32
					WSAGetLastError() != WSAEWOULDBLOCK
#else
					errno != EWOULDBLOCK && errno != EAGAIN
#endif
				))
				{
					Console.WriteLn("UePCB: peer disconnected %s:%d", peer.ip.c_str(), peer.port);
					CLOSE_SOCKET(peer.fd);
					peer.fd = INVALID_SOCKET;
					peer.connected = false;
				}
			}
		}
	}
}

static void netplay_init(UePcbState* s, int port)
{
#ifdef _WIN32
	WSADATA wsa;
	WSAStartup(MAKEWORD(2, 2), &wsa);
#endif

	s->listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (IS_VALID_SOCKET(s->listen_fd))
	{
		int opt = 1;
		setsockopt(s->listen_fd, SOL_SOCKET, SO_REUSEADDR, (const char*)&opt, sizeof(opt));
		set_nonblocking(s->listen_fd);

		sockaddr_in addr{};
		addr.sin_family = AF_INET;
		addr.sin_addr.s_addr = INADDR_ANY;
		addr.sin_port = htons(7500 + port);

		if (bind(s->listen_fd, (sockaddr*)&addr, sizeof(addr)) == 0)
		{
			listen(s->listen_fd, 4);
			Console.WriteLn("UePCB: netplay HOST listening 0.0.0.0:%d", 7500 + port);
		}
	}

	s->running = true;
	s->net_thread = std::thread(netplay_thread_proc, s);
}

static void netplay_close(UePcbState* s)
{
	s->running = false;
	if (s->net_thread.joinable())
		s->net_thread.join();

	if (IS_VALID_SOCKET(s->listen_fd))
	{
		CLOSE_SOCKET(s->listen_fd);
		s->listen_fd = INVALID_SOCKET;
	}

	for (auto& peer : s->peers)
	{
		if (IS_VALID_SOCKET(peer.fd))
		{
			CLOSE_SOCKET(peer.fd);
			peer.fd = INVALID_SOCKET;
		}
		peer.connected = false;
	}
}

static void peer_send_all(UePcbState* s, const u8* data, size_t len)
{
	for (auto& peer : s->peers)
	{
		if (peer.connected && IS_VALID_SOCKET(peer.fd))
		{
			send(peer.fd, (const char*)data, (int)len, 0);
		}
	}
}

// -----------------------------------------------------------------------------
// USB & Device Logic (Includes OHCI Safe Intercept & Registers)
// -----------------------------------------------------------------------------
static void uepcb_reset(USBDevice* dev)
{
	UePcbState* s = USB_CONTAINER_OF(dev, UePcbState, dev);
	s->tx_accum.clear();
	s->current_rx_chunk.clear();
	s->current_rx_offset = 0;

	std::lock_guard<std::mutex> lk(s->in_lock);
	s->in_q.clear();

	// Reset Default Registers
	std::memset(s->registers, 0, sizeof(s->registers));
	s->registers[0x00] = 0x00; // CTL1
	s->registers[0x01] = 0x00; // CTL2
	s->registers[0x20] = 0x01; // Link active status
}

static void uepcb_handle_destroy(USBDevice* dev)
{
	UePcbState* s = USB_CONTAINER_OF(dev, UePcbState, dev);
	netplay_close(s);
	delete s;
}

static void uepcb_handle_control(USBDevice* dev, USBPacket* p, int request, int value, int index, int length, u8* data)
{
	UePcbState* s = USB_CONTAINER_OF(dev, UePcbState, dev);

	switch (request)
	{
		case DeviceRequest | USB_REQ_GET_STATUS:
			data[0] = 0x00;
			data[1] = 0x00;
			p->actual_length = 2;
			break;

		case DeviceOutRequest | USB_REQ_CLEAR_FEATURE:
			if (value == USB_ENDPOINT_HALT)
				p->actual_length = 0;
			else
				p->status = USB_RET_STALL;
			break;

		case InterfaceRequest | USB_REQ_GET_INTERFACE:
			data[0] = 0x00;
			p->actual_length = 1;
			break;

		case InterfaceOutRequest | USB_REQ_SET_INTERFACE:
			p->actual_length = 0;
			break;

		// AN986 / Pegasus Vendor Requests
		case VendorDeviceRequest | 0xF0: // Read Register
		{
			u8 reg_idx = (u8)(index & 0xFF);
			if (reg_idx < sizeof(s->registers))
			{
				std::memcpy(data, &s->registers[reg_idx], std::min<int>(length, sizeof(s->registers) - reg_idx));
			}
			else if (reg_idx == 0x10) // MAC Read
			{
				std::memcpy(data, s->mac_addr, std::min<int>(length, 6));
			}
			else
			{
				std::memset(data, 0, length);
			}
			p->actual_length = length;
			break;
		}

		case VendorDeviceOutRequest | 0xF1: // Write Register
		{
			u8 reg_idx = (u8)(index & 0xFF);
			if (reg_idx < sizeof(s->registers) && length > 0)
			{
				s->registers[reg_idx] = data[0];
			}
			p->actual_length = length;
			break;
		}

		default:
			p->status = USB_RET_STALL;
			break;
	}
}

static void uepcb_handle_data(USBDevice* dev, USBPacket* p)
{
	UePcbState* s = USB_CONTAINER_OF(dev, UePcbState, dev);
	const u8 ep = p->ep->nr;

	switch (p->pid)
	{
		case USB_TOKEN_OUT:
		{
			// 防範過大封包造成 OHCI DMA 記憶體越界與 ohci_die
			if (p->buffer_size > 2048)
			{
				p->status = USB_RET_STALL;
				return;
			}

			u8 pkt[2048];
			int np = std::min<int>(p->buffer_size, (int)sizeof(pkt));
			usb_packet_copy(p, pkt, np);
			s->tx_accum.insert(s->tx_accum.end(), pkt, pkt + np);

			if (s->tx_accum.size() > 16384)
				s->tx_accum.clear();

			while (s->tx_accum.size() >= 2)
			{
				int ethlen = s->tx_accum[0] | (s->tx_accum[1] << 8);
				int total = 2 + ethlen;
				if (ethlen < 14 || ethlen > 1600)
				{
					s->tx_accum.clear();
					break;
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
			if (ep == 3) // Interrupt EP (Status Notify)
			{
				u8 ready = 0x40; // PHY Status: Link Active
				usb_packet_copy(p, &ready, 1);
				break;
			}

			// ---- 核心修正：安全分配 OHCI DMA 分塊讀取邏輯 ----
			if (s->current_rx_chunk.empty() || s->current_rx_offset >= s->current_rx_chunk.size())
			{
				s->current_rx_chunk.clear();
				s->current_rx_offset = 0;

				std::vector<u8> raw;
				{
					std::lock_guard<std::mutex> lk(s->in_lock);
					if (!s->in_q.empty())
					{
						raw = std::move(s->in_q.front());
						s->in_q.pop_front();
					}
				}

				if (!raw.empty())
					s->current_rx_chunk = std::move(raw);
			}

			if (!s->current_rx_chunk.empty() && s->current_rx_offset < s->current_rx_chunk.size())
			{
				const size_t remain = s->current_rx_chunk.size() - s->current_rx_offset;
				// 嚴格限制單次 USB 寫入不得大於 OHCI Descriptor 緩衝區與 USB 1.1 64B 上限
				const int copyLen = std::min<int>({(int)p->buffer_size, (int)remain, 64});

				usb_packet_copy(p, s->current_rx_chunk.data() + s->current_rx_offset, copyLen);
				s->current_rx_offset += copyLen;
			}
			else
			{
				// 無資料時明確給予 USB_RET_NAK 讓 OHCI 掛起 Descriptor，防止 OHCI 觸發 DMA Error 死鎖
				p->status = USB_RET_NAK;
			}
			break;
		}

		default:
			p->status = USB_RET_STALL;
			break;
	}
}

USBDevice* uepcb_init(int port)
{
	UePcbState* s = new UePcbState();
	std::memset(&s->dev, 0, sizeof(s->dev));

	s->port_num = port;
	s->dev.speed = USB_SPEED_FULL;
	s->dev.klass.reset = uepcb_reset;
	s->dev.klass.handle_control = uepcb_handle_control;
	s->dev.klass.handle_data = uepcb_handle_data;
	s->dev.klass.handle_destroy = uepcb_handle_destroy;

	netplay_init(s, port);

	return &s->dev;
}