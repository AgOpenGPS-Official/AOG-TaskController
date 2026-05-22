#include "aog_can_bridge_plugin.hpp"
#include <iostream>

using udp = boost::asio::ip::udp;

static constexpr std::size_t FRAME_SIZE = 13;

AOGCANBridgePlugin::AOGCANBridgePlugin(std::uint16_t port) :
  port(port),
  socket_(ioContext)
{
}

std::string AOGCANBridgePlugin::get_name() const
{
	return "AOG-CAN-Bridge";
}

bool AOGCANBridgePlugin::get_is_valid() const
{
	return isValid;
}

void AOGCANBridgePlugin::open()
{
	try
	{
		socket_.open(udp::v4());
		socket_.set_option(boost::asio::socket_base::reuse_address(true));
		socket_.bind(udp::endpoint(boost::asio::ip::address_v4::any(), port));
		socket_.non_blocking(true);
		isValid = true;
		std::cout << "[AOG-CAN-Bridge] Listening on UDP port " << port << std::endl;
	}
	catch (const std::exception &e)
	{
		std::cout << "[AOG-CAN-Bridge] Failed to open socket: " << e.what() << std::endl;
		isValid = false;
	}
}

void AOGCANBridgePlugin::close()
{
	if (socket_.is_open())
	{
		socket_.close();
	}
	isValid = false;
	hasKnownSender = false;
}

bool AOGCANBridgePlugin::read_frame(isobus::CANMessageFrame &frame)
{
	if (!isValid)
		return false;

	std::uint8_t buf[FRAME_SIZE];
	boost::asio::ip::udp::endpoint remoteEndpoint;
	boost::system::error_code ec;

	std::size_t bytesReceived = socket_.receive_from(boost::asio::buffer(buf, FRAME_SIZE), remoteEndpoint, 0, ec);

	if (ec == boost::asio::error::would_block || ec == boost::asio::error::try_again)
		return false;

	if (ec || bytesReceived != FRAME_SIZE)
		return false;

	senderEndpoint = remoteEndpoint;
	hasKnownSender = true;

	std::uint32_t rawId;
	std::memcpy(&rawId, buf, 4);

	frame.isExtendedFrame = (rawId >> 31) & 1;
	frame.identifier = rawId & 0x1FFFFFFF;
	frame.dataLength = buf[4] <= 8 ? buf[4] : 8;
	std::memcpy(frame.data, buf + 5, 8);
	frame.timestamp_us = 0;

	return true;
}

bool AOGCANBridgePlugin::write_frame(const isobus::CANMessageFrame &frame)
{
	if (!isValid)
		return false;

	std::uint8_t buf[FRAME_SIZE] = {};
	std::uint32_t rawId = frame.identifier | (frame.isExtendedFrame ? 0x80000000U : 0U);
	std::memcpy(buf, &rawId, 4);
	buf[4] = frame.dataLength;
	std::memcpy(buf + 5, frame.data, 8);

	if (!hasKnownSender)
		return true; // No hardware connected yet — succeed silently to keep TC alive

	boost::system::error_code ec;
	socket_.send_to(boost::asio::buffer(buf, FRAME_SIZE), senderEndpoint, 0, ec);

	return !ec;
}
