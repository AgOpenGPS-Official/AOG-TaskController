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
#include <set>
#include <string>
#include <vector>

#include "isobus/hardware_integration/can_hardware_plugin.hpp"
#include "isobus/isobus/isobus_functionalities.hpp"
#include "isobus/isobus/isobus_speed_distance_messages.hpp"
#include "isobus/isobus/isobus_virtual_terminal_client.hpp"
#include "isobus/isobus/isobus_virtual_terminal_client_update_helper.hpp"
#include "isobus/isobus/nmea2000_message_interface.hpp"

#include "field_registry.hpp"
#include "guidance_track_context.hpp"
#include "logging_utils.hpp"
#include "settings.hpp"
#include "task_controller.hpp"
#include "tecu_coexistence.hpp"
#include "tractor_facilities.hpp"
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

	struct WorkingWithoutDemand
	{
		std::uint32_t sinceMs = 0;
		bool warned = false;
	};

	void send_task_controller_status_message();

	/// @brief Keeps the TC off the bus (no TC Status, no TC server update) for TC_QUIET_PERIOD_MS
	/// @param reason Logged, so the log shows why clients are about to reconnect
	void start_tc_quiet_period(const std::string &reason);
	bool is_tc_quiet(); ///< Also ends an expired quiet period
	void recover_stale_clients();

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

	/// Our TECU's place among the Tractor ECUs on the bus (ISO 11783-9 4.4.2.1)
	enum class TecuRole
	{
		None, ///< No TECU (disabled, or it could not claim an address)
		Primary, ///< Function instance 0 at address 0xF0: sends everything
		Secondary ///< Another Tractor ECU is on the bus: we only add what it lacks
	};
	/// @brief Create the TECU control function and start its address claim (does not wait for it)
	void start_tecu_claim(TecuRole role, std::uint8_t functionInstance);
	/// @brief Finishes a pending claim, and once a second checks for other Tractor ECUs:
	/// steps down from primary when one appears, and as a secondary sends only what they lack.
	void update_tecu();
	/// @brief Destroy the TECU interfaces and control function, so it can be claimed again with another NAME
	void teardown_tecu();
	/// @brief Start or stop our TECU messages to match the plan, and advertise them in PGN 65033
	void apply_tecu_plan(const TecuMessagePlan &plan);
	void create_speed_messages_interface(const TecuMessagePlan &plan);
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

	bool is_aog_connected() const;

	std::shared_ptr<Settings> settings = std::make_shared<Settings>();
	boost::asio::io_context ioContext = boost::asio::io_context();
	std::shared_ptr<UdpConnections> udpConnections = std::make_shared<UdpConnections>(settings, ioContext);

	std::shared_ptr<isobus::CANHardwarePlugin> canDriver;
	std::shared_ptr<MyTCServer> tcServer;
	std::shared_ptr<isobus::InternalControlFunction> tcCF = nullptr;
	std::shared_ptr<isobus::InternalControlFunction> tecuCF = nullptr;
	std::unique_ptr<isobus::SpeedMessagesInterface> speedMessagesInterface;
	std::unique_ptr<isobus::NMEA2000MessageInterface> nmea2000MessageInterface;
	std::unique_ptr<TractorFacilities> tractorFacilities;
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

	/// Longer than the 6 s a client waits for TC Status before it drops the connection and starts over
	static constexpr std::uint32_t TC_QUIET_PERIOD_MS = 6500;
	/// Minimum time between two quiet periods started for stale clients, so reconnecting clients get time to finish
	static constexpr std::uint32_t STALE_CLIENT_RECOVERY_COOLDOWN_MS = 30000;
	/// A client still stale after this many quiet periods does not reconnect this way; stop disturbing the others
	static constexpr std::uint8_t MAX_STALE_CLIENT_RECOVERIES = 3;
	std::uint32_t tcQuietUntilMs = 0; ///< 0 = not quiet
	std::uint32_t lastStaleClientRecoveryMs = 0;
	std::map<std::uint64_t, std::uint8_t> staleClientRecoveries; ///< Quiet periods started per client NAME
	std::int32_t lastSpeedValue = 0;
	std::int32_t lastXteValue = 0;
	std::uint8_t gnssFixQuality = 0; ///< AOG fix quality (NMEA 2000 GNSS Method): 0=invalid, 1=GPS, 2=DGPS, 3=PPS, 4=RTK Fix, 5=Float, 6=Estimated, 7=Manual, 8=Simulated
	std::uint32_t lastGnssQualityMs = 0; ///< Timestamp of last PGN 0xD6 fix-quality update (0 = never received)
	static constexpr std::uint32_t GNSS_QUALITY_TIMEOUT_MS = 2000; ///< No PGN 0xD6 for this long = fix quality unknown
	static constexpr std::uint32_t AOG_CONNECTION_TIMEOUT_MS = 3000; ///< No AOG packet for this long = disconnected

	// Guidance track context — real data from AOG PGN 0xF4.
	GuidanceTrackProvider trackProvider;
	GuidanceTrackContext currentTrackContext;
	bool aogWasConnectedForTrack = false; ///< Edge-detection for AOG connect/disconnect transitions

	// Field identity — from AOG PGN 0xF3. Folded into the upper 16 bits of DDI 508
	// (see the PGN 0xF4 handling in setup_udp_connections()) so a track's guidance reference
	// line ID is unique across fields, not just within whichever field AOG currently has open.
	FieldRegistry fieldRegistry;
	std::string currentFieldName; ///< Empty when no field is open
	std::uint16_t currentFieldIndex = 0;
	bool hasActiveField = false;

	std::uint32_t lastDistanceMm = 0;
	std::uint32_t lastAogPacketMs = 0;

	/// @brief AOG opened a field: switch section control on, as if AOG's SC button was pressed (if enabled in settings)
	/// @param how What showed the field opening, for the log
	void on_aog_field_opened(const std::string &how);
	/// The field name (PGN 0xF3) and PGN 229 resuming both report one field opening; act on it once
	static constexpr std::uint32_t FIELD_OPEN_DEDUPLICATE_MS = 5000;
	std::uint32_t lastFieldOpenAppliedMs = 0; ///< When on_aog_field_opened() last switched section control on, 0 = never

	bool tractorFacilitiesSentOnPowerUp = false;

	static constexpr std::uint32_t TECU_CLAIM_TIMEOUT_MS = 5000;
	static constexpr std::uint32_t TECU_CHECK_INTERVAL_MS = 1000;
	/// As a secondary, send nothing until the other TECUs answered PGN 65033, or this long has passed
	static constexpr std::uint32_t TECU_PLAN_SETTLE_MS = 2500;
	/// ...but always watch the bus at least this long, for messages they send without advertising
	static constexpr std::uint32_t TECU_PLAN_MIN_SETTLE_MS = 1000;
	static constexpr std::uint8_t SECONDARY_TECU_PREFERRED_ADDRESS = 0x80; ///< Arbitrary range; never 0xF0
	static constexpr std::uint8_t ALL_FUNCTION_INSTANCES = 31; ///< Highest NAME function instance (5 bits)
	isobus::NAME tecuNameTemplate{ 0 }; ///< Our NAME fields, set in setup_control_functions()
	OtherTecuMonitor otherTecus;
	TecuRole tecuRole = TecuRole::None;
	std::uint8_t tecuFunctionInstance = 0;
	std::uint32_t tecuClaimStartedMs = 0;
	std::uint32_t tecuClaimedMs = 0; ///< 0 while the claim is pending
	bool tecuInterfacesReady = false;
	std::uint32_t tecuInterfacesReadyMs = 0;
	std::uint32_t lastTecuCheckMs = 0;
	bool tecuPlanSettled = false;
	TecuMessagePlan tecuPlan; ///< What our TECU sends now
	std::set<std::uint8_t> knownOtherTecuAddresses; ///< To log other TECUs appearing and leaving

	static constexpr std::uint32_t SECTION_DEMAND_LOG_INTERVAL_MS = 200;
	static constexpr std::uint32_t SECTION_DEMAND_TIMEOUT_MS = 2000; ///< AOG sends PGN 229 every frame while a field is open
	static constexpr std::uint32_t WORKING_WITHOUT_DEMAND_WARNING_MS = 10000;
	std::vector<bool> sectionDemand; ///< Last section states from AOG (PGN 229)
	std::vector<bool> loggedSectionDemand; ///< The section demand last written to the log
	std::uint32_t sectionDemandChangesSinceLog = 0;
	std::uint32_t lastSectionDemandMs = 0; ///< When PGN 229 was last received, 0 = never
	std::uint32_t lastSectionDemandLogMs = 0;
	bool sectionDemandArriving = false;
	std::map<std::uint64_t, WorkingWithoutDemand> workingWithoutDemand; ///< Per client NAME
	std::uint32_t vtDisconnectedSinceMs = 0;
	std::uint32_t lastVtStatusUpdateMs = 0;
	std::uint32_t lastVtSectionUpdateMs = 0;
	bool tcAddressConflictActive = false;
	bool tecuAddressClaimFailed = false;
	std::map<std::uint64_t, ImplementSnapshot> implementSnapshots;
	std::map<std::uint16_t, std::string> lastVtStrings;
};
