/**
 * @author Daan Steenbergen
 * @brief The main application class
 * @version 0.1
 * @date 2025-1-20
 *
 * @copyright 2025 Daan Steenbergen
 */
#pragma once

#include <boost/asio.hpp>
#include <memory>
#include <string>

#include "isobus/hardware_integration/can_hardware_plugin.hpp"
#include "isobus/isobus/isobus_speed_distance_messages.hpp"
#include "isobus/isobus/nmea2000_message_interface.hpp"

#include "settings.hpp"
#include "task_controller.hpp"
#include "udp_connections.hpp"

// Utility function to get current timestamp string (HH:MM:SS.mmm)
std::string get_timestamp();

class Application
{
public:
	Application(std::shared_ptr<isobus::CANHardwarePlugin> canDriver);

	bool initialize();
	bool update();
	void stop();

private:
	void send_task_controller_status_message();

	std::shared_ptr<Settings> settings = std::make_shared<Settings>();
	boost::asio::io_context ioContext = boost::asio::io_context();
	std::shared_ptr<UdpConnections> udpConnections = std::make_shared<UdpConnections>(settings, ioContext);

	std::shared_ptr<isobus::CANHardwarePlugin> canDriver;
	std::shared_ptr<MyTCServer> tcServer;
	std::shared_ptr<isobus::InternalControlFunction> tcCF = nullptr;
	std::shared_ptr<isobus::InternalControlFunction> tecuCF = nullptr;
	std::unique_ptr<isobus::SpeedMessagesInterface> speedMessagesInterface;
	std::unique_ptr<isobus::NMEA2000MessageInterface> nmea2000MessageInterface;
	std::uint8_t nmea2000SequenceIdentifier = 0;
	std::uint32_t lastJ1939SpeedTransmit = 0;
	std::uint32_t lastTCStatusTransmit = 0;
	std::int32_t lastSpeedValue = 0;
	bool taskTotalsActive = true;
};
