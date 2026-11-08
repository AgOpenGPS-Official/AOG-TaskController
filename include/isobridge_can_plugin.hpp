/**
 * @file isobridge_can_plugin.hpp
 * @brief CAN driver that reaches the ISOBUS through a Teensy ISOBridge over UDP.
 */

#pragma once

#include "isobus/hardware_integration/can_hardware_plugin.hpp"

#include <boost/asio.hpp>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>

/// UDP CAN driver for the AgOpenGPS Teensy ISOBridge protocol (AC v1, UDP port 9200).
class ISOBridgeCANPlugin final : public isobus::CANHardwarePlugin
{
public:
	explicit ISOBridgeCANPlugin(const std::string &gatewayAddress);
	~ISOBridgeCANPlugin() override;

	std::string get_name() const override;
	bool get_is_valid() const override;
	void close() override;
	void open() override;
	bool read_frame(isobus::CANMessageFrame &canFrame) override;
	bool write_frame(const isobus::CANMessageFrame &canFrame) override;
	std::string get_last_error() const override;

private:
	using udp = boost::asio::ip::udp;

	bool send_hello_locked();
	bool send_packet_locked(std::uint8_t type, std::uint8_t count, const std::uint8_t *body, std::size_t bodySize);
	void send_keepalive_if_due_locked();
	void receive_datagram_locked();
	void close_socket_locked();

	boost::asio::io_context ioContext;
	udp::socket socket;
	udp::endpoint gateway;
	mutable std::mutex mutex;
	std::deque<isobus::CANMessageFrame> receivedFrames;
	std::atomic_bool valid{ false };
	std::uint16_t txSequence = 0;
	bool addressValid = false;
	std::chrono::steady_clock::time_point lastHello{};
	std::string lastError;
};
