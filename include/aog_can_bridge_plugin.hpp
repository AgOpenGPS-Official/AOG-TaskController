#pragma once
#include "isobus/hardware_integration/can_hardware_plugin.hpp"
#include <boost/asio.hpp>
#include <cstdint>
#include <string>

class AOGCANBridgePlugin : public isobus::CANHardwarePlugin
{
public:
	static constexpr std::uint16_t DEFAULT_PORT = 2234;
	explicit AOGCANBridgePlugin(std::uint16_t port = DEFAULT_PORT);

	std::string get_name() const override;
	bool get_is_valid() const override;
	void open() override;
	void close() override;
	bool read_frame(isobus::CANMessageFrame &frame) override;
	bool write_frame(const isobus::CANMessageFrame &frame) override;

private:
	std::uint16_t port;
	boost::asio::io_context ioContext;
	boost::asio::ip::udp::socket socket_;
	boost::asio::ip::udp::endpoint senderEndpoint;
	bool hasKnownSender = false;
	bool isValid = false;
};
