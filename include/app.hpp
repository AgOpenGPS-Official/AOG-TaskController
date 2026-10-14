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
#include <cstdint>
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
	struct ImplementDetails
	{
		std::string displayName = "No implement";
		std::uint8_t sections = 0;
		std::string widthText;
		std::string boomOffsetText;
	};

	struct ImplementSnapshot
	{
		std::string displayName;
		std::uint8_t sections = 0;
		std::string widthText;
	};

	void send_task_controller_status_message();

	/// @brief Logs AOG's section demand (PGN 229) when it changed, at most every SECTION_DEMAND_LOG_INTERVAL_MS
	void log_section_demand();
	/// @brief Logs when PGN 229 starts and stops, and warns when an implement in automatic is working
	/// while AOG asks for no sections — e.g. AOG's own section Auto button is off after a field change
	void check_section_demand();
	static std::string describe_section_demand(const std::vector<bool> &demand, std::uint8_t numberOfSections);

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
	void send_hardware_message(const std::string &text, std::uint8_t duration, std::uint8_t color);
	ImplementDetails derive_implement_details(ClientState &state) const;

	static constexpr std::uint8_t HW_MSG_ALERT = 0;
	static constexpr std::uint8_t HW_MSG_INFO = 1;

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
	bool canHardwareStarted = false; ///< CANHardwareInterface::start() succeeded, so stop() has a thread to stop
	bool vtConfigSynced = false;
	bool vtWasConnected = false;
	bool vtConnectionWarningLogged = false;
	bool vtCapabilitiesLogged = false;
	std::uint8_t nmea2000SequenceIdentifier = 0;
	std::uint32_t lastJ1939SpeedTransmit = 0;
	std::uint32_t lastTCStatusTransmit = 0;
	std::int32_t lastSpeedValue = 0;
	std::int32_t lastXteValue = 0;
	std::uint8_t gnssFixQuality = 0; ///< AOG fix quality (NMEA 2000 GNSS Method): 0=invalid, 1=GPS, 2=DGPS, 3=PPS, 4=RTK Fix, 5=Float, 6=Estimated, 7=Manual, 8=Simulated
	std::uint32_t lastGnssQualityMs = 0; ///< Timestamp of last PGN 0xD6 fix-quality update (0 = never received)
	static constexpr std::uint32_t GNSS_QUALITY_TIMEOUT_MS = 2000; ///< No PGN 0xD6 for this long = fix quality unknown
	std::uint32_t lastDistanceMm = 0;
	std::uint32_t lastAogPacketMs = 0;

	static constexpr std::uint32_t SECTION_DEMAND_LOG_INTERVAL_MS = 200;
	static constexpr std::uint32_t SECTION_DEMAND_TIMEOUT_MS = 2000; ///< AOG sends PGN 229 every frame while a field is open
	static constexpr std::uint32_t WORKING_WITHOUT_DEMAND_WARNING_MS = 10000;
	std::vector<bool> sectionDemand; ///< Last section states from AOG (PGN 229)
	std::vector<bool> loggedSectionDemand; ///< The section demand last written to the log
	std::uint32_t sectionDemandChangesSinceLog = 0;
	std::uint32_t lastSectionDemandMs = 0; ///< When PGN 229 was last received, 0 = never
	std::uint32_t lastSectionDemandLogMs = 0;
	bool sectionDemandArriving = false;
	std::map<std::uint64_t, std::uint32_t> workingWithoutDemandSinceMs; ///< Per client NAME
	std::map<std::uint64_t, bool> workingWithoutDemandWarned; ///< Per client NAME
	std::uint32_t vtDisconnectedSinceMs = 0;
	std::uint32_t lastVtStatusUpdateMs = 0;
	std::uint32_t lastVtSectionUpdateMs = 0;
	bool tcAddressConflictActive = false;
	bool tecuAddressClaimFailed = false;
	std::map<std::uint64_t, ImplementSnapshot> implementSnapshots;
	std::map<std::uint16_t, std::string> lastVtStrings;
};
