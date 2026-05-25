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

#include "isobus/hardware_integration/can_hardware_plugin.hpp"
#include "isobus/isobus/can_message.hpp"
#include "isobus/isobus/isobus_functionalities.hpp"
#include "isobus/isobus/isobus_speed_distance_messages.hpp"
#include "isobus/isobus/nmea2000_message_interface.hpp"

#include "logging_utils.hpp"
#include "settings.hpp"
#include "task_controller.hpp"
#include "udp_connections.hpp"

#include <fstream>
#include <map>
#include <mutex>
#include <string>

/// @brief Tracks the connection state of a potential TC client seen on the bus
struct ClientConnectionInfo
{
	std::uint64_t nameFull = 0;
	std::uint8_t address = 0xFF;
	std::string typeString;
	bool workingSetMasterReceived = false;
	bool requestVersionReceived = false;
	bool versionResponseSent = false;
	bool requestVersionSent = false;
	bool clientTaskReceived = false;
	bool registeredAsClient = false;
	std::uint32_t lastWorkingSetMasterMs = 0;
	std::uint32_t lastRequestVersionMs = 0;
	std::uint32_t lastClientTaskMs = 0;
	std::uint32_t firstSeenMs = 0;
};

class Application
{
public:
	Application(std::shared_ptr<isobus::CANHardwarePlugin> canDriver);

	bool initialize();
	bool update();
	void stop();

private:
	void send_task_controller_status_message();
	void send_tc_status_burst();
	void dump_connection_table();
	void update_connection_tracker();

	static void log_can_working_set_master(const isobus::CANMessage &message, void *parent);
	static void log_can_process_data(const isobus::CANMessage &message, void *parent);
	static void log_all_can_messages(const isobus::CANMessage &message, void *parent);

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
	std::uint8_t nmea2000SequenceIdentifier = 0;
	std::uint32_t lastJ1939SpeedTransmit = 0;
	std::uint32_t lastTCStatusTransmit = 0;
	std::int32_t lastSpeedValue = 0;

	// Connection tracking for diagnostics
	std::map<std::uint64_t, ClientConnectionInfo> connectionTracker;
	std::uint32_t lastConnectionTableDumpMs = 0;
	std::uint32_t tcInitializedTimestampMs = 0;
	bool tcStatusBurstSent = false;

	// CAN message log file
	std::ofstream canLogFile;
};
