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
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "isobus/hardware_integration/can_hardware_plugin.hpp"
#include "isobus/isobus/isobus_functionalities.hpp"
#include "isobus/isobus/isobus_speed_distance_messages.hpp"
#include "isobus/isobus/isobus_virtual_terminal_client.hpp"
#include "isobus/isobus/isobus_virtual_terminal_client_update_helper.hpp"
#include "isobus/isobus/nmea2000_message_interface.hpp"

#include "logging_utils.hpp"
#include "settings.hpp"
#include "task_controller.hpp"
#include "udp_connections.hpp"

class Application
{
public:
	Application(std::shared_ptr<isobus::CANHardwarePlugin> canDriver);

	bool initialize();
	bool update();
	void stop();

private:
	void send_task_controller_status_message();

	bool setup_can_hardware();
	bool setup_control_functions();
	void setup_task_controller_server();
	void setup_tecu_interfaces();
	void setup_udp_connections();

	void setup_vt_client();
	void update_vt_client();
	void try_start_vt_client();
	void handle_vt_disconnected();
	void log_vt_capabilities_once();
	void sync_vt_config_once();
	void update_vt_section_map();
	void update_vt_status_strings(bool aogConnected);

	void send_vt_string_if_changed(std::uint16_t objectID, const std::string &value);

	std::shared_ptr<Settings> settings = std::make_shared<Settings>();
	boost::asio::io_context ioContext = boost::asio::io_context();
	std::shared_ptr<UdpConnections> udpConnections = std::make_shared<UdpConnections>(settings, ioContext);

	std::shared_ptr<isobus::CANHardwarePlugin> canDriver;
	std::shared_ptr<MyTCServer> tcServer;
	std::shared_ptr<isobus::InternalControlFunction> tcCF = nullptr;
	std::shared_ptr<isobus::InternalControlFunction> tecuCF = nullptr;
	std::unique_ptr<isobus::SpeedMessagesInterface> speedMessagesInterface;
	std::unique_ptr<isobus::NMEA2000MessageInterface> nmea2000MessageInterface;
	std::unique_ptr<isobus::ControlFunctionFunctionalities> tecuFunctionalities;
	std::unique_ptr<isobus::ControlFunctionFunctionalities> tcFunctionalities;
	std::shared_ptr<isobus::VirtualTerminalClient> vtClient;
	std::unique_ptr<isobus::VirtualTerminalClientUpdateHelper> vtUpdateHelper;
	std::vector<std::uint8_t> vtObjectPool;
	bool vtClientStarted = false;
	bool vtConfigSynced = false;
	bool vtWasConnected = false;
	bool vtConnectionWarningLogged = false;
	bool vtCapabilitiesLogged = false;
	std::uint8_t nmea2000SequenceIdentifier = 0;
	std::uint32_t lastJ1939SpeedTransmit = 0;
	std::uint32_t lastTCStatusTransmit = 0;
	std::int32_t lastSpeedValue = 0;
	std::int32_t lastXteValue = 0;
	std::uint32_t lastDistanceMm = 0;
	std::uint32_t lastAogPacketMs = 0;
	std::uint32_t vtDisconnectedSinceMs = 0;
	std::uint32_t lastVtStatusUpdateMs = 0;
	std::uint32_t lastVtSectionUpdateMs = 0;
	std::map<std::uint16_t, std::string> lastVtStrings;
};
