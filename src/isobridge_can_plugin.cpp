/**
 * @file isobridge_can_plugin.cpp
 * @brief CAN driver that reaches the ISOBUS through a Teensy ISOBridge over UDP.
 */

#include "isobridge_can_plugin.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <thread>

namespace
{
	constexpr std::uint16_t BRIDGE_PORT = 9200;
	constexpr std::uint8_t MAGIC_A = 'A';
	constexpr std::uint8_t MAGIC_C = 'C';
	constexpr std::uint8_t PROTOCOL_VERSION = 1;

	constexpr std::uint8_t TYPE_DATA = 1;
	constexpr std::uint8_t TYPE_HELLO = 2;
	constexpr std::uint8_t TYPE_HELLO_ACK = 3;
	constexpr std::uint8_t TYPE_BYE = 5;

	constexpr std::size_t HEADER_SIZE = 8;
	constexpr std::size_t FRAME_SIZE = 16;
	constexpr std::size_t HELLO_ACK_BODY_SIZE = 28;
	constexpr std::uint8_t MAX_FRAMES = 88;
	constexpr std::size_t MAX_PACKET = HEADER_SIZE + MAX_FRAMES * FRAME_SIZE;

	constexpr std::uint8_t CLIENT_TABLE_FULL = 0xFF;
	constexpr std::uint32_t EXTENDED_ID_FLAG = 0x80000000UL;
	constexpr std::uint32_t CAN_ID_MASK = 0x1FFFFFFFUL;

	constexpr auto KEEPALIVE_INTERVAL = std::chrono::seconds(1);
	constexpr auto HANDSHAKE_TIMEOUT = std::chrono::seconds(4);
	constexpr auto HELLO_RETRY_INTERVAL = std::chrono::milliseconds(250);
	constexpr auto HANDSHAKE_POLL_INTERVAL = std::chrono::milliseconds(5);
	constexpr auto EMPTY_READ_SLEEP = std::chrono::milliseconds(1);

	void put_u16(std::uint8_t *destination, std::uint16_t value)
	{
		destination[0] = static_cast<std::uint8_t>(value);
		destination[1] = static_cast<std::uint8_t>(value >> 8);
	}

	void put_u32(std::uint8_t *destination, std::uint32_t value)
	{
		for (int i = 0; i < 4; i++)
		{
			destination[i] = static_cast<std::uint8_t>(value >> (8 * i));
		}
	}

	std::uint32_t get_u32(const std::uint8_t *source)
	{
		return static_cast<std::uint32_t>(source[0]) |
		  (static_cast<std::uint32_t>(source[1]) << 8) |
		  (static_cast<std::uint32_t>(source[2]) << 16) |
		  (static_cast<std::uint32_t>(source[3]) << 24);
	}

	bool is_no_data(const boost::system::error_code &error)
	{
		return (boost::asio::error::would_block == error) || (boost::asio::error::try_again == error);
	}

	/// UDP errors that say nothing about this socket. Windows reports the ICMP "port unreachable"
	/// from a rebooting Teensy as a connection reset on the next receive. The stack only opens a
	/// driver once, so giving up on these would lose the CAN bus until the TC is restarted.
	bool is_transient(const boost::system::error_code &error)
	{
		return (boost::asio::error::connection_reset == error) ||
		  (boost::asio::error::connection_refused == error) ||
		  (boost::asio::error::network_unreachable == error) ||
		  (boost::asio::error::host_unreachable == error) ||
		  (boost::asio::error::message_size == error);
	}

	bool has_valid_header(const std::array<std::uint8_t, MAX_PACKET> &packet, std::size_t length)
	{
		return (length >= HEADER_SIZE) && (MAGIC_A == packet[0]) && (MAGIC_C == packet[1]) && (PROTOCOL_VERSION == packet[2]);
	}
}

ISOBridgeCANPlugin::ISOBridgeCANPlugin(const std::string &gatewayAddress) :
  socket(ioContext)
{
	boost::system::error_code error;
	const auto address = boost::asio::ip::make_address_v4(gatewayAddress, error);
	if (error)
	{
		lastError = "Invalid ISOBridge IPv4 address: " + gatewayAddress;
		return;
	}
	gateway = udp::endpoint(address, BRIDGE_PORT);
	addressValid = true;
}

ISOBridgeCANPlugin::~ISOBridgeCANPlugin()
{
	close();
}

std::string ISOBridgeCANPlugin::get_name() const
{
	return "AgOpenGPS ISOBridge UDP";
}

bool ISOBridgeCANPlugin::get_is_valid() const
{
	return valid.load();
}

std::string ISOBridgeCANPlugin::get_last_error() const
{
	std::lock_guard<std::mutex> lock(mutex);
	return lastError;
}

bool ISOBridgeCANPlugin::send_packet_locked(std::uint8_t type, std::uint8_t count, const std::uint8_t *body, std::size_t bodySize)
{
	std::array<std::uint8_t, MAX_PACKET> packet{};
	if (HEADER_SIZE + bodySize > packet.size())
	{
		return false;
	}

	packet[0] = MAGIC_A;
	packet[1] = MAGIC_C;
	packet[2] = PROTOCOL_VERSION;
	packet[3] = type;
	put_u16(&packet[4], txSequence++);
	packet[6] = count;
	packet[7] = 0;
	if ((0 != bodySize) && (nullptr != body))
	{
		std::memcpy(packet.data() + HEADER_SIZE, body, bodySize);
	}

	boost::system::error_code error;
	const auto sent = socket.send_to(boost::asio::buffer(packet.data(), HEADER_SIZE + bodySize), gateway, 0, error);
	if (error || (sent != HEADER_SIZE + bodySize))
	{
		lastError = "ISOBridge UDP send failed: " + error.message();
		return false;
	}
	return true;
}

bool ISOBridgeCANPlugin::send_hello_locked()
{
	// HELLO body v1: client kind, options, two reserved bytes, 16-byte client name.
	// Kind 1 identifies a PC-side application, options 0 disables sender echo.
	std::array<std::uint8_t, 20> body{};
	body[0] = 1;
	body[1] = 0;
	constexpr char CLIENT_NAME[] = "AOG-TaskController";
	std::memcpy(body.data() + 4, CLIENT_NAME, std::min<std::size_t>(16, sizeof(CLIENT_NAME) - 1));

	const bool sent = send_packet_locked(TYPE_HELLO, 0, body.data(), body.size());
	if (sent)
	{
		lastHello = std::chrono::steady_clock::now();
	}
	return sent;
}

void ISOBridgeCANPlugin::send_keepalive_if_due_locked()
{
	// The keepalive is also how the bridge relearns this client after it reboots.
	if (std::chrono::steady_clock::now() - lastHello >= KEEPALIVE_INTERVAL)
	{
		send_hello_locked();
	}
}

void ISOBridgeCANPlugin::close_socket_locked()
{
	boost::system::error_code ignored;
	if (socket.is_open())
	{
		socket.close(ignored);
	}
}

void ISOBridgeCANPlugin::open()
{
	std::lock_guard<std::mutex> lock(mutex);
	if (valid || !addressValid)
	{
		return;
	}

	boost::system::error_code error;
	socket.open(udp::v4(), error);
	if (error)
	{
		lastError = "Cannot open UDP socket: " + error.message();
		return;
	}
	socket.bind(udp::endpoint(udp::v4(), 0), error);
	if (error)
	{
		lastError = "Cannot bind UDP socket: " + error.message();
		close_socket_locked();
		return;
	}
	socket.non_blocking(true, error);
	if (error)
	{
		lastError = "Cannot set UDP socket non-blocking: " + error.message();
		close_socket_locked();
		return;
	}

	// Sized for a full DATA packet: on Windows a datagram bigger than the buffer is an error.
	std::array<std::uint8_t, MAX_PACKET> buffer{};
	const auto deadline = std::chrono::steady_clock::now() + HANDSHAKE_TIMEOUT;
	while (std::chrono::steady_clock::now() < deadline)
	{
		if (!send_hello_locked())
		{
			break;
		}

		const auto retryAt = std::chrono::steady_clock::now() + HELLO_RETRY_INTERVAL;
		while (std::chrono::steady_clock::now() < retryAt)
		{
			udp::endpoint from;
			const auto length = socket.receive_from(boost::asio::buffer(buffer), from, 0, error);
			if (error && !is_no_data(error) && !is_transient(error))
			{
				lastError = "ISOBridge UDP receive failed: " + error.message();
				close_socket_locked();
				return;
			}

			if (!error && (from == gateway) && has_valid_header(buffer, length) &&
			    (TYPE_HELLO_ACK == buffer[3]) && (length >= HEADER_SIZE + HELLO_ACK_BODY_SIZE))
			{
				if (CLIENT_TABLE_FULL == buffer[HEADER_SIZE])
				{
					lastError = "ISOBridge refused client registration: client table full (4 clients).";
					close_socket_locked();
					return;
				}
				valid = true;
				lastError.clear();
				return;
			}
			std::this_thread::sleep_for(HANDSHAKE_POLL_INTERVAL);
		}
	}

	close_socket_locked();
	if (lastError.empty())
	{
		lastError = "No HELLO_ACK from ISOBridge at " + gateway.address().to_string() + ":9200. Check Teensy, Ethernet and UDP firewall.";
	}
}

void ISOBridgeCANPlugin::close()
{
	std::lock_guard<std::mutex> lock(mutex);
	if (socket.is_open() && valid)
	{
		send_packet_locked(TYPE_BYE, 0, nullptr, 0);
	}
	valid = false;
	receivedFrames.clear();
	close_socket_locked();
}

void ISOBridgeCANPlugin::receive_datagram_locked()
{
	std::array<std::uint8_t, MAX_PACKET> packet{};
	udp::endpoint from;
	boost::system::error_code error;
	const auto length = socket.receive_from(boost::asio::buffer(packet), from, 0, error);
	if (is_no_data(error) || is_transient(error))
	{
		return;
	}
	if (error)
	{
		lastError = "ISOBridge UDP receive error: " + error.message();
		valid = false;
		close_socket_locked();
		return;
	}
	if ((from != gateway) || !has_valid_header(packet, length))
	{
		return;
	}

	const std::uint8_t type = packet[3];
	const std::uint8_t count = packet[6];
	if ((TYPE_HELLO_ACK == type) && (length >= HEADER_SIZE + HELLO_ACK_BODY_SIZE))
	{
		// Not fatal: the next keepalive tries again, and a slot may have been freed by then.
		if (CLIENT_TABLE_FULL == packet[HEADER_SIZE])
		{
			lastError = "ISOBridge client table is full.";
		}
		return;
	}

	if ((TYPE_DATA != type) || (count > MAX_FRAMES) || (length != HEADER_SIZE + static_cast<std::size_t>(count) * FRAME_SIZE))
	{
		return;
	}

	for (std::uint8_t i = 0; i < count; i++)
	{
		const std::uint8_t *entry = packet.data() + HEADER_SIZE + static_cast<std::size_t>(i) * FRAME_SIZE;
		const std::uint32_t identifier = get_u32(entry);
		const std::uint8_t dataLength = entry[4];
		if (dataLength > 8)
		{
			continue;
		}

		isobus::CANMessageFrame frame{};
		frame.timestamp_us = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());
		frame.identifier = identifier & ~EXTENDED_ID_FLAG;
		frame.isExtendedFrame = (0 != (identifier & EXTENDED_ID_FLAG));
		frame.channel = 0;
		frame.dataLength = dataLength;
		std::memcpy(frame.data, entry + 8, dataLength);
		receivedFrames.push_back(frame);
	}
}

bool ISOBridgeCANPlugin::read_frame(isobus::CANMessageFrame &canFrame)
{
	{
		std::lock_guard<std::mutex> lock(mutex);
		if (!valid)
		{
			return false;
		}

		send_keepalive_if_due_locked();
		if (receivedFrames.empty())
		{
			receive_datagram_locked();
		}
		if (!receivedFrames.empty())
		{
			canFrame = receivedFrames.front();
			receivedFrames.pop_front();
			return true;
		}
	}

	// The socket is non-blocking and the stack calls this in a tight loop.
	std::this_thread::sleep_for(EMPTY_READ_SLEEP);
	return false;
}

bool ISOBridgeCANPlugin::write_frame(const isobus::CANMessageFrame &canFrame)
{
	std::lock_guard<std::mutex> lock(mutex);
	if (!valid)
	{
		return false;
	}

	send_keepalive_if_due_locked();

	std::array<std::uint8_t, FRAME_SIZE> body{};
	const std::uint32_t identifier = (canFrame.identifier & CAN_ID_MASK) | (canFrame.isExtendedFrame ? EXTENDED_ID_FLAG : 0);
	put_u32(body.data(), identifier);
	body[4] = std::min<std::uint8_t>(canFrame.dataLength, 8);
	std::memcpy(body.data() + 8, canFrame.data, body[4]);
	return send_packet_locked(TYPE_DATA, 1, body.data(), body.size());
}
