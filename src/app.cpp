/**
 * @author Daan Steenbergen
 * @brief The main application class
 * @version 0.1
 * @date 2025-1-20
 *
 * @copyright 2025 Daan Steenbergen
 */
#include "app.hpp"

#include "isobus/hardware_integration/available_can_drivers.hpp"
#include "isobus/hardware_integration/can_hardware_interface.hpp"
#include "isobus/isobus/can_general_parameter_group_numbers.hpp"
#include "isobus/isobus/can_network_manager.hpp"
#include "isobus/isobus/isobus_preferred_addresses.hpp"
#include "isobus/isobus/isobus_standard_data_description_indices.hpp"
#include "isobus/isobus/isobus_task_controller_server.hpp"
#include "isobus/utility/system_timing.hpp"

#include "task_controller.hpp"

#include <iomanip>
#include <thread>

using boost::asio::ip::udp;

Application::Application(std::shared_ptr<isobus::CANHardwarePlugin> canDriver) :
  canDriver(canDriver)
{
}

bool Application::initialize()
{
	settings->load();
	if (nullptr == canDriver)
	{
		std::cout << "[" << get_timestamp() << "] Unable to find a CAN driver. Please make sure the selected driver is installed." << std::endl;
		return false;
	}
	isobus::CANHardwareInterface::set_number_of_can_channels(1);
	isobus::CANHardwareInterface::assign_can_channel_frame_handler(0, canDriver);

	if ((!isobus::CANHardwareInterface::start()) || (!canDriver->get_is_valid()))
	{
		std::cout << "[" << get_timestamp() << "] Failed to start CAN hardware interface." << std::endl;
		return false;
	}

	isobus::CANNetworkManager::CANNetwork.get_configuration().set_number_of_packets_per_cts_message(255);

	isobus::NAME ourNAME(0);

	//! Make sure you change these for your device!!!!
	ourNAME.set_arbitrary_address_capable(true);
	ourNAME.set_industry_group(2);
	ourNAME.set_device_class(0);
	ourNAME.set_identity_number(20);
	ourNAME.set_ecu_instance(0);
	ourNAME.set_function_instance(0); // TC #1. If you want to change the TC number, change this.
	ourNAME.set_device_class_instance(0);
	ourNAME.set_manufacturer_code(1407);

	isobus::NAME tcNAME = ourNAME;
	tcNAME.set_function_code(static_cast<std::uint8_t>(isobus::NAME::Function::TaskController));

	isobus::NAME tecuNAME = ourNAME;
	tecuNAME.set_function_code(static_cast<std::uint8_t>(isobus::NAME::Function::TractorECU));
	tecuNAME.set_arbitrary_address_capable(false); // TECU address is fixed
	tecuNAME.set_ecu_instance(0);

	std::cout << "[" << get_timestamp() << "] [Init] Creating Task Controller control function..." << std::endl;
	tcCF = isobus::CANNetworkManager::CANNetwork.create_internal_control_function(tcNAME, 0, isobus::preferred_addresses::IndustryGroup2::TaskController_MappingComputer); // The preferred address for a TC is defined in ISO 11783

	// Wait for TC address claim with bounded wait loop (no async to avoid blocking on destruction)
	// Also implements minimum 250ms delay per J1939-81 section 4.4.4.1
	// CAs with addresses in range 128-247 must wait 250ms after claiming before sending other messages
	static constexpr std::uint32_t MINIMUM_ADDRESS_CLAIM_DELAY_MS = 250;
	static constexpr std::uint32_t MAX_ADDRESS_CLAIM_WAIT_MS = 5000;
	auto tcClaimWaitStart = isobus::SystemTiming::get_timestamp_ms();
	bool tcAddressClaimed = false;

	while (isobus::SystemTiming::get_time_elapsed_ms(tcClaimWaitStart) < MAX_ADDRESS_CLAIM_WAIT_MS)
	{
		isobus::CANNetworkManager::CANNetwork.update();
		if (tcCF->get_address_valid())
		{
			tcAddressClaimed = true;
			break;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}

	if (!tcAddressClaimed)
	{
		std::cout << "[" << get_timestamp() << "] Failed to claim address for TC server. The control function might be invalid." << std::endl;
		return false;
	}

	// Record when the address was actually claimed for the 250ms delay calculation
	auto tcAddressClaimedTime = isobus::SystemTiming::get_timestamp_ms();
	std::cout << "[" << get_timestamp() << "] [Init] TC claimed address " << static_cast<int>(tcCF->get_address()) << std::endl;

	// Print existing ECUs on the bus with their NAMEs
	auto existingControlFunctions = isobus::CANNetworkManager::CANNetwork.get_control_functions(false);
	if (!existingControlFunctions.empty())
	{
		std::cout << "[" << get_timestamp() << "] [Init] Existing ECUs on the bus:" << std::endl;
		for (const auto &cf : existingControlFunctions)
		{
			if (cf && cf->get_address_valid())
			{
				const auto &name = cf->get_NAME();
				std::cout << "[" << get_timestamp() << "] [Init]   Address " << static_cast<int>(cf->get_address())
				          << " - NAME 0x" << std::hex << name.get_full_name() << std::dec
				          << " [Fn:" << static_cast<int>(name.get_function_code())
				          << "/IG:" << static_cast<int>(name.get_industry_group())
				          << "/Cls:" << static_cast<int>(name.get_device_class()) << "]"
				          << " - Mfg:" << name.get_manufacturer_code()
				          << " (" << cf->get_type_string() << ")" << std::endl;
			}
		}
	}

	// Ensure minimum 250ms delay after address claim per J1939-81
	auto tcClaimElapsedMs = isobus::SystemTiming::get_time_elapsed_ms(tcAddressClaimedTime);
	if (tcClaimElapsedMs < MINIMUM_ADDRESS_CLAIM_DELAY_MS)
	{
		auto remainingDelay = MINIMUM_ADDRESS_CLAIM_DELAY_MS - tcClaimElapsedMs;
		std::cout << "[" << get_timestamp() << "] [Init] Waiting " << remainingDelay << "ms after address claim (J1939-81 250ms rule)..." << std::endl;
		// Process CAN messages during the delay to prevent timeouts
		auto delayStart = isobus::SystemTiming::get_timestamp_ms();
		while (isobus::SystemTiming::get_time_elapsed_ms(delayStart) < remainingDelay)
		{
			isobus::CANNetworkManager::CANNetwork.update();
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}

	// Create TECU control function
	// TODO: Should we wait between this and TC?
	// TODO: If there's already a TECU on the bus we should not create ours
	if (tcCF && settings->is_tecu_enabled())
	{ // Only create TECU if TC was created and ECU is enabled
		std::cout << "[" << get_timestamp() << "] [Init] Creating Tractor ECU control function..." << std::endl;
		tecuCF = isobus::CANNetworkManager::CANNetwork.create_internal_control_function(tecuNAME, 0, isobus::preferred_addresses::IndustryGroup2::TractorECU);

		// Wait for TECU address claim with minimum 250ms delay per J1939-81 section 4.4.4.1
		std::cout << "[" << get_timestamp() << "] [Init] Tractor ECU control function created, waiting for address claim..." << std::endl;

		// Update the network manager to process TECU CF claiming
		int tecuClaimAttempts = 0;
		const int MAX_TECU_CLAIM_ATTEMPTS = 50; // 5 seconds max
		while (!tecuCF->get_address_valid() && tecuClaimAttempts < MAX_TECU_CLAIM_ATTEMPTS)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			isobus::CANNetworkManager::CANNetwork.update();
			tecuClaimAttempts++;
		}

		// Check if TECU successfully claimed its FIXED address (128)
		// TECU is non-arbitrary-address-capable and MUST use address 128
		if (tecuCF->get_address_valid() && tecuCF->get_address() == isobus::preferred_addresses::IndustryGroup2::TractorECU)
		{
			// Record when the address was actually claimed for the 250ms delay calculation
			auto tecuAddressClaimedTime = isobus::SystemTiming::get_timestamp_ms();
			std::cout << "[" << get_timestamp() << "] [Init] TECU claimed address " << static_cast<int>(tecuCF->get_address()) << std::endl;

			// Ensure minimum 250ms delay after address claim per J1939-81
			auto tecuClaimElapsedMs = isobus::SystemTiming::get_time_elapsed_ms(tecuAddressClaimedTime);
			if (tecuClaimElapsedMs < MINIMUM_ADDRESS_CLAIM_DELAY_MS)
			{
				auto remainingDelay = MINIMUM_ADDRESS_CLAIM_DELAY_MS - tecuClaimElapsedMs;
				std::cout << "[" << get_timestamp() << "] [Init] Waiting " << remainingDelay << "ms after TECU address claim (J1939-81 250ms rule)..." << std::endl;
				// Process CAN messages during the delay to prevent timeouts
				auto delayStart = isobus::SystemTiming::get_timestamp_ms();
				while (isobus::SystemTiming::get_time_elapsed_ms(delayStart) < remainingDelay)
				{
					isobus::CANNetworkManager::CANNetwork.update();
					std::this_thread::sleep_for(std::chrono::milliseconds(10));
				}
			}
		}
		else
		{
			if (tecuCF->get_address_valid())
			{
				std::cout << "[" << get_timestamp() << "] [Warning] TECU claimed unexpected address " << static_cast<int>(tecuCF->get_address()) << " instead of 128!" << std::endl;
			}
			else
			{
				std::cout << "[" << get_timestamp() << "] [Warning] TECU failed to claim address 128! Another TECU may be on the bus." << std::endl;
			}
			std::cout << "[" << get_timestamp() << "] [Warning] TECU functionality will be disabled." << std::endl;
			tecuCF.reset(); // Release the failed control function
		}
	}

	// Map settings version to TaskControllerVersion enum
	isobus::TaskControllerServer::TaskControllerVersion tcVersionEnum;
	switch (settings->get_tc_version())
	{
		case 0:
			tcVersionEnum = isobus::TaskControllerServer::TaskControllerVersion::DraftInternationalStandard;
			break;
		case 1:
			tcVersionEnum = isobus::TaskControllerServer::TaskControllerVersion::FinalDraftInternationalStandardFirstEdition;
			break;
		case 2:
			tcVersionEnum = isobus::TaskControllerServer::TaskControllerVersion::FirstPublishedEdition;
			break;
		case 3:
			tcVersionEnum = isobus::TaskControllerServer::TaskControllerVersion::SecondEditionDraft;
			break;
		case 4:
		default:
			tcVersionEnum = isobus::TaskControllerServer::TaskControllerVersion::SecondPublishedEdition;
			break;
	}
	std::cout << "[" << get_timestamp() << "] [Init] TC version set to " << static_cast<int>(settings->get_tc_version()) << " ("
	          << static_cast<int>(tcVersionEnum) << ")" << std::endl;

	tcServer = std::make_shared<MyTCServer>(tcCF, tcVersionEnum);
	auto &languageInterface = tcServer->get_language_command_interface();
	languageInterface.set_language_code(settings->get_language_code());
	languageInterface.set_country_code(settings->get_country_code());
	tcServer->initialize();
	tcServer->set_task_totals_active(true); // TODO: make this dynamic based on status in AOG

	// Register CAN callbacks to log WorkingSetMaster and ProcessData for diagnostics
	isobus::CANNetworkManager::CANNetwork.add_any_control_function_parameter_group_number_callback(
	  static_cast<std::uint32_t>(isobus::CANLibParameterGroupNumber::WorkingSetMaster),
	  Application::log_can_working_set_master,
	  this);
	isobus::CANNetworkManager::CANNetwork.add_any_control_function_parameter_group_number_callback(
	  static_cast<std::uint32_t>(isobus::CANLibParameterGroupNumber::ProcessData),
	  Application::log_can_process_data,
	  this);

	tcInitializedTimestampMs = isobus::SystemTiming::get_timestamp_ms();
	std::cout << "[" << get_timestamp() << "] [Init] TC server initialized, sending startup status burst..." << std::endl;

	// Initialize speed and distance messages
	if (tecuCF && tecuCF->get_address_valid())
	{
		std::cout << "[" << get_timestamp() << "] [Init] Creating TECU Control Function Functionalities..." << std::endl;
		tecuFunctionalities = std::make_unique<isobus::ControlFunctionFunctionalities>(tecuCF);
		// Announce as Basic Tractor ECU Server Class 1 (ISO 11783-9 compliance)
		tecuFunctionalities->set_functionality_is_supported(
		  isobus::ControlFunctionFunctionalities::Functionalities::BasicTractorECUServer,
		  2, // Generation 2 (Version 2)
		  true);
		tecuFunctionalities->set_basic_tractor_ECU_server_option_state(
		  isobus::ControlFunctionFunctionalities::BasicTractorECUOptions::Class1NoOptions,
		  true);
		std::cout << "[" << get_timestamp() << "] [Init] TECU announced as Class 1 Tractor ECU (PGN 64654)" << std::endl;

		std::cout << "[" << get_timestamp() << "] [Init] Creating Speed Messages Interface on TECU..." << std::endl;
		speedMessagesInterface = std::make_unique<isobus::SpeedMessagesInterface>(tecuCF, true, true, true, false); //TODO: make configurable whether to send these messages
		speedMessagesInterface->initialize();
		speedMessagesInterface->wheelBasedSpeedTransmitData.set_implement_start_stop_operations_state(isobus::SpeedMessagesInterface::WheelBasedMachineSpeedData::ImplementStartStopOperations::NotAvailable);
		speedMessagesInterface->wheelBasedSpeedTransmitData.set_key_switch_state(isobus::SpeedMessagesInterface::WheelBasedMachineSpeedData::KeySwitchState::NotAvailable);
		speedMessagesInterface->wheelBasedSpeedTransmitData.set_operator_direction_reversed_state(isobus::SpeedMessagesInterface::WheelBasedMachineSpeedData::OperatorDirectionReversed::NotAvailable);
		speedMessagesInterface->machineSelectedSpeedTransmitData.set_speed_source(isobus::SpeedMessagesInterface::MachineSelectedSpeedData::SpeedSource::NavigationBasedSpeed);
		std::cout << "[" << get_timestamp() << "] [Init] Speed Messages Interface created and initialized." << std::endl;

		std::cout << "[" << get_timestamp() << "] [Init] Creating NMEA2000 Message Interface on TECU..." << std::endl;
		nmea2000MessageInterface = std::make_unique<isobus::NMEA2000MessageInterface>(tecuCF, false, false, false, false, false, false, false);
		nmea2000MessageInterface->initialize();
		nmea2000MessageInterface->set_enable_sending_cog_sog_cyclically(true); // TODO: make configurable whether to send these messages
		std::cout << "[" << get_timestamp() << "] [Init] NMEA2000 Message Interface created and initialized." << std::endl;
	}
	else
	{
		if (!settings->is_tecu_enabled())
		{
			std::cout << "[" << get_timestamp() << "] [Info] Tractor ECU disabled in settings, skipping ECU initialization." << std::endl;
		}
		else
		{
			std::cout << "[" << get_timestamp() << "] [Warning] TECU Control Function not available, Speed/NMEA interfaces not created" << std::endl;
		}
	}

	std::cout << "[" << get_timestamp() << "] Task controller server started." << std::endl;

	static std::uint8_t xteSid = 0;
	static std::uint32_t lastXteTransmit = 0;

	auto packetHandler = [this](std::uint8_t src, std::uint8_t pgn, std::span<std::uint8_t> data) {
		if (src == 0x7F && pgn == 0xE5) // 229 - 64 sections PGN
		{
			std::vector<bool> sectionStates;
			for (std::uint8_t j = 0; j < 8; j++)
			{
				for (std::uint8_t i = 0; i < 8; i++)
				{
					sectionStates.push_back(data[j] & (1 << i));
				}
			}
			tcServer->update_section_states(sectionStates);
		}
		else if (src == 0x7F && pgn == 0xF1) // 241 - Section Control
		{
			std::uint8_t sectionControlState = data[0];
			std::cout << "[" << get_timestamp() << "] Received request from AOG to change section control state to " << (sectionControlState == 1 ? "enabled" : "disabled") << std::endl;
			tcServer->update_section_control_enabled(sectionControlState == 1);
		}
		else if (src == 0x7F && pgn == 0xF2) // Process Data
		{
			auto identifier = static_cast<isobus::DataDescriptionIndex>(data[0] | (data[1] << 8));

			std::int32_t value = data[2] | (data[3] << 8) | (data[4] << 16) | (data[5] << 24);
			if (identifier == isobus::DataDescriptionIndex::ActualSpeed)
			{
				lastSpeedValue = value; // Store the full precision value
				std::uint16_t speed = std::abs(value);
				auto direction = value < 0 ? isobus::SpeedMessagesInterface::MachineDirection::Reverse : isobus::SpeedMessagesInterface::MachineDirection::Forward;
				if (speedMessagesInterface)
				{
					speedMessagesInterface->groundBasedSpeedTransmitData.set_machine_direction_of_travel(direction);
					speedMessagesInterface->wheelBasedSpeedTransmitData.set_machine_direction_of_travel(direction);
					speedMessagesInterface->machineSelectedSpeedTransmitData.set_machine_direction_of_travel(direction);

					speedMessagesInterface->groundBasedSpeedTransmitData.set_machine_speed(speed);
					speedMessagesInterface->wheelBasedSpeedTransmitData.set_machine_speed(speed);
					speedMessagesInterface->machineSelectedSpeedTransmitData.set_machine_speed(speed);

					speedMessagesInterface->groundBasedSpeedTransmitData.set_machine_distance(0); // TODO: Implement distance
					speedMessagesInterface->wheelBasedSpeedTransmitData.set_machine_distance(0); // TODO: Implement distance
					speedMessagesInterface->machineSelectedSpeedTransmitData.set_machine_distance(0); // TODO: Implement distance
				}
				if (nmea2000MessageInterface)
				{
					auto &cog_sog_message = nmea2000MessageInterface->get_cog_sog_transmit_message();
					cog_sog_message.set_sequence_id(nmea2000SequenceIdentifier++);
					cog_sog_message.set_speed_over_ground(speed / 10);
					cog_sog_message.set_course_over_ground(0); // TODO: Implement course
					cog_sog_message.set_course_over_ground_reference(isobus::NMEA2000Messages::CourseOverGroundSpeedOverGroundRapidUpdate::CourseOverGroundReference::NotApplicableOrNull);
				}
			}
			else if (identifier == isobus::DataDescriptionIndex::GuidanceLineDeviation)
			{
				std::int32_t xte = value / 1000; // Convert from mm to m
				static const std::uint8_t xteMode = 0b00000001;
				xteSid = xteSid % 253 + 1;

				std::uint8_t status = 0; // TODO: navigation terminated status

				std::array<std::uint8_t, 8> xteData = {
					xteSid, // Sequence ID
					static_cast<std::uint8_t>(xteMode | 0b00110000 | (status == 1 ? 0b00000000 : 0b01000000)), // XTE mode (4 bits) + Reserved (2 bits set to 1) + Navigation Terminated (2 bits)
					static_cast<std::uint8_t>(xte & 0xFF), // XTE LSB
					static_cast<std::uint8_t>((xte >> 8) & 0xFF), // XTE
					static_cast<std::uint8_t>((xte >> 16) & 0xFF), // XTE
					static_cast<std::uint8_t>((xte >> 24) & 0xFF), // XTE MSB
					0xFF, // Reserved byte 1 (all bits set to 1)
					0xFF // Reserved byte 2 (all bits set to 1)
				};
				if (isobus::SystemTiming::time_expired_ms(lastXteTransmit, 1000)) // Transmit every second
				{
					if (isobus::CANNetworkManager::CANNetwork.send_can_message(0x1F903, xteData.data(), xteData.size(), tcCF))
					{
						lastXteTransmit = isobus::SystemTiming::get_timestamp_ms();
					}
				}
			}
			else if (static_cast<std::uint16_t>(identifier) == 597 /*isobus::DataDescriptionIndex::TotalDistance*/ && speedMessagesInterface)
			{
				auto distance = static_cast<std::uint32_t>(value);
				speedMessagesInterface->groundBasedSpeedTransmitData.set_machine_distance(distance);
				speedMessagesInterface->wheelBasedSpeedTransmitData.set_machine_distance(distance);
				speedMessagesInterface->machineSelectedSpeedTransmitData.set_machine_distance(distance);
			}
		}
	};
	udpConnections->set_packet_handler(packetHandler);
	udpConnections->open();

	std::cout << "[" << get_timestamp() << "] UDP connections opened." << std::endl;

	return true;
}

bool Application::update()
{
	static std::uint32_t lastHeartbeatTransmit = 0;

	udpConnections->handle_address_detection();
	udpConnections->handle_incoming_packets();

	tcServer->request_measurement_commands();
	tcServer->update();
	if (tecuFunctionalities)
		tecuFunctionalities->update();
	if (speedMessagesInterface)
		speedMessagesInterface->update();
	if (nmea2000MessageInterface)
		nmea2000MessageInterface->update();

	// Send section control heartbeat to AOG every 100ms (PGN 0xF0, source 0x80)
	// When no clients with sections, send 0 sections as heartbeat so AOG knows TC is alive
	if (isobus::SystemTiming::time_expired_ms(lastHeartbeatTransmit, 100))
	{
		bool anyClientWithSections = false;
		for (auto &client : tcServer->get_clients())
		{
			auto &state = client.second;

			// Skip clients with no sections (e.g., tractors or non-implement devices)
			if (state.get_number_of_sections() == 0)
			{
				continue;
			}

			anyClientWithSections = true;
			std::vector<uint8_t> data = { state.is_section_control_enabled(), state.get_number_of_sections() };

			std::uint8_t sectionIndex = 0;
			while (sectionIndex < state.get_number_of_sections())
			{
				std::uint8_t byte = 0;
				for (std::uint8_t i = 0; i < 8; i++)
				{
					if (sectionIndex < state.get_number_of_sections())
					{
						byte |= (state.get_section_actual_state(sectionIndex) == SectionState::ON) << i;
						sectionIndex++;
					}
				}
				data.push_back(byte);
			}
			udpConnections->send(0x80, 0xF0, data);
		}

		// If no clients with sections, send heartbeat with 0 sections
		if (settings->is_aog_heartbeat_enabled() && !anyClientWithSections)
		{
			std::vector<uint8_t> heartbeatData = { 0, 0 }; // section control disabled, 0 sections
			udpConnections->send(0x80, 0xF0, heartbeatData);
		}

		lastHeartbeatTransmit = isobus::SystemTiming::get_timestamp_ms();
	}

	// Send J1939 PGN 65256 every 100ms (0.1 seconds)
	if (isobus::SystemTiming::time_expired_ms(lastJ1939SpeedTransmit, 100) && tecuCF && tecuCF->get_address_valid())
	{
		std::uint16_t speed_j1939 = (static_cast<std::uint32_t>(std::abs(lastSpeedValue)) * 576u + 312u) / 625u; // mm/s -> (km/h)*256
		std::array<std::uint8_t, 8> j1939_speed_data = {
			0xFF,
			0xFF, // Compass Bearing (SPN 165)
			static_cast<std::uint8_t>(speed_j1939 & 0xFF),
			static_cast<std::uint8_t>((speed_j1939 >> 8) & 0xFF), // Machine Speed MSB (SPN 517)
			0xFF,
			0xFF, // Pitch (SPN 583)
			0xFF,
			0xFF // Altitude (SPN 580)
		};
		if (isobus::CANNetworkManager::CANNetwork.send_can_message(0xFEE8, j1939_speed_data.data(), j1939_speed_data.size(), tecuCF))
		{
			lastJ1939SpeedTransmit = isobus::SystemTiming::get_timestamp_ms();
		}
	}

	// Send startup TC Status burst to help late-joining implements detect the TC
	// ISO 11783-10 allows up to 5 Hz (200ms interval) for TC Status
	// After the burst, AgIsoStack++ handles the periodic 2-second TC Status internally.
	if (!tcStatusBurstSent && tcCF && tcCF->get_address_valid())
	{
		send_tc_status_burst();
	}

	// Update connection tracker state
	update_connection_tracker();

	// Dump connection diagnostics table every 30 seconds
	if (isobus::SystemTiming::time_expired_ms(lastConnectionTableDumpMs, 30000))
	{
		dump_connection_table();
		lastConnectionTableDumpMs = isobus::SystemTiming::get_timestamp_ms();
	}

	return true;
}

void Application::send_task_controller_status_message()
{
	// ISO 11783-10 B.8.1 Task Controller Status message
	// PGN: 0xCB00 (Process Data), Command: 0x0E (Task Controller Status)
	// Transmission rate: 2 seconds, Global destination
	//
	// Byte 1: Bits 4-1 = 0x0E (Command), Bits 8-5 = 0x0F (Element nibble NA)
	// Byte 2: 0xFF (Element number MSB - not available)
	// Bytes 3-4: 0xFFFF (DDI - not available)
	// Byte 5: Status bits
	//   Bit 1 = Task totals active (1 = active, 0 = not active)
	//   Bit 2 = TC busy saving data
	//   Bit 3 = TC busy reading data
	//   Bit 4 = TC busy executing B.6 command
	//   Bit 8 = TC out of memory
	// Byte 6: Source address of client for B.6 command (0 if not applicable)
	// Byte 7: B.6 command being executed (0 if not applicable)
	// Byte 8: Reserved

	std::uint8_t statusByte = 0x00;
	if (tcServer && tcServer->get_task_totals_active())
	{
		statusByte |= 0x01; // Bit 1: Task totals active
	}
	// Bits 2-4 and 8 are always 0 for now (not busy, not out of memory)

	std::array<std::uint8_t, 8> tcStatusData = {
		0xFE, // Byte 1: Command 0x0E + Element nibble 0xF
		0xFF, // Byte 2: Element number MSB (not available)
		0xFF, // Byte 3: DDI LSB (not available)
		0xFF, // Byte 4: DDI MSB (not available)
		statusByte, // Byte 5: TC Status
		0x00, // Byte 6: Client SA for B.6 command (not applicable)
		0x00, // Byte 7: B.6 command being executed (not applicable)
		0xFF // Byte 8: Reserved
	};

	// Send to global destination (0xFF) with priority 3 (ISO 11783-10 requirement)
	const auto transmitAttemptTimestamp = isobus::SystemTiming::get_timestamp_ms();
	bool sendSuccess = isobus::CANNetworkManager::CANNetwork.send_can_message(
	    0xCB00, tcStatusData.data(), tcStatusData.size(), tcCF, nullptr,
	    isobus::CANIdentifier::CANPriority::Priority3);

	if (!sendSuccess)
	{
		std::cout << "[" << get_timestamp() << "] [TC Status] Failed to send TC Status message!" << std::endl;
	}
	else
	{
		std::cout << "[" << get_timestamp() << "] [TC Status] Sent: ID=0x0CBFFF" << std::hex
		          << static_cast<int>(tcCF->get_address()) << std::dec
		          << " Payload=" << std::hex << std::uppercase
		          << std::setfill('0') << std::setw(2) << static_cast<int>(tcStatusData[0]) << " "
		          << std::setw(2) << static_cast<int>(tcStatusData[1]) << " "
		          << std::setw(2) << static_cast<int>(tcStatusData[2]) << " "
		          << std::setw(2) << static_cast<int>(tcStatusData[3]) << " "
		          << std::setw(2) << static_cast<int>(tcStatusData[4]) << " "
		          << std::setw(2) << static_cast<int>(tcStatusData[5]) << " "
		          << std::setw(2) << static_cast<int>(tcStatusData[6]) << " "
		          << std::setw(2) << static_cast<int>(tcStatusData[7]) << std::dec
		          << " [task_totals=" << (statusByte & 0x01) << "]"
		          << std::endl;
	}

	// Update the transmit timestamp for every send attempt so failed sends
	// still respect the minimum 2-second transmit period.
	lastTCStatusTransmit = transmitAttemptTimestamp;
}

void Application::send_tc_status_burst()
{
	// ISO 11783-10 allows TC Status up to 5 Hz (200ms minimum interval).
	// Send a burst of 5 messages at 200ms intervals right after TC initialization
	// to maximize the chance that late-joining implements detect the TC.
	static std::uint8_t burstCount = 0;
	static std::uint32_t lastBurstTransmit = 0;

	if (burstCount == 0)
	{
		lastBurstTransmit = tcInitializedTimestampMs;
	}

	if (isobus::SystemTiming::time_expired_ms(lastBurstTransmit, 200))
	{
		send_task_controller_status_message();
		burstCount++;
		lastBurstTransmit = isobus::SystemTiming::get_timestamp_ms();
		lastTCStatusTransmit = lastBurstTransmit; // Prevent duplicate 2s timer
		std::cout << "[" << get_timestamp() << "] [TC Status] Startup burst message " << static_cast<int>(burstCount) << "/5 sent" << std::endl;

		if (burstCount >= 5)
		{
			tcStatusBurstSent = true;
			burstCount = 0;
			std::cout << "[" << get_timestamp() << "] [TC Status] Startup burst complete" << std::endl;
		}
	}
}

void Application::dump_connection_table()
{
	auto now = isobus::SystemTiming::get_timestamp_ms();
	std::cout << "[" << get_timestamp() << "] === Client Connection Diagnostics Table ===" << std::endl;
	std::cout << std::left << std::setw(18) << "NAME"
	          << std::setw(10) << "Address"
	          << std::setw(8) << "WSM"
	          << std::setw(8) << "ReqVer"
	          << std::setw(8) << "VerSent"
	          << std::setw(8) << "ReqSent"
	          << std::setw(8) << "ClTask"
	          << std::setw(8) << "Reg"
	          << std::setw(12) << "Type"
	          << std::setw(10) << "Age(s)"
	          << std::endl;
	std::cout << std::string(98, '-') << std::endl;

	for (const auto &entry : connectionTracker)
	{
		const auto &info = entry.second;
		std::stringstream nameStream;
		nameStream << "0x" << std::hex << info.nameFull << std::dec;
		std::string nameStr = nameStream.str();

		std::cout << std::left << std::setw(18) << nameStr.substr(0, 17)
		          << std::setw(10) << static_cast<int>(info.address)
		          << std::setw(8) << (info.workingSetMasterReceived ? "Y" : "N")
		          << std::setw(8) << (info.requestVersionReceived ? "Y" : "N")
		          << std::setw(8) << (info.versionResponseSent ? "Y" : "N")
		          << std::setw(8) << (info.requestVersionSent ? "Y" : "N")
		          << std::setw(8) << (info.clientTaskReceived ? "Y" : "N")
		          << std::setw(8) << (info.registeredAsClient ? "Y" : "N")
		          << std::setw(12) << info.typeString.substr(0, 11)
		          << std::setw(10) << static_cast<int>((now - info.firstSeenMs) / 1000)
		          << std::endl;
	}

	if (connectionTracker.empty())
	{
		std::cout << "[" << get_timestamp() << "] No clients seen on the bus yet." << std::endl;
	}
	std::cout << "[" << get_timestamp() << "] === End Connection Table ===" << std::endl;
}

void Application::update_connection_tracker()
{
	auto now = isobus::SystemTiming::get_timestamp_ms();

	// Sync registered clients from tcServer
	if (tcServer)
	{
		for (auto &client : tcServer->get_clients())
		{
			auto nameFull = client.first->get_NAME().get_full_name();
			auto it = connectionTracker.find(nameFull);
			if (it != connectionTracker.end())
			{
				it->second.registeredAsClient = true;
				it->second.address = client.first->get_address();
			}
		}
	}

	// Scan all control functions on the bus and add any we haven't seen yet
	auto allCFs = isobus::CANNetworkManager::CANNetwork.get_control_functions(false);
	for (const auto &cf : allCFs)
	{
		if (cf && cf->get_address_valid())
		{
			auto nameFull = cf->get_NAME().get_full_name();
			if (connectionTracker.find(nameFull) == connectionTracker.end())
			{
				ClientConnectionInfo info;
				info.nameFull = nameFull;
				info.address = cf->get_address();
				info.typeString = cf->get_type_string();
				info.firstSeenMs = now;
				connectionTracker[nameFull] = info;
				std::cout << "[" << get_timestamp() << "] [ConnectionTracker] New CF detected: NAME 0x" << std::hex << nameFull << std::dec
				          << " @ address " << static_cast<int>(cf->get_address())
				          << " (" << info.typeString << ")" << std::endl;
			}
		}
	}
}

void Application::log_can_working_set_master(const isobus::CANMessage &message, void *parent)
{
	if (nullptr == parent)
	{
		return;
	}
	auto *app = static_cast<Application *>(parent);
	auto source = message.get_source_control_function();
	if (nullptr == source)
	{
		return;
	}

	auto now = isobus::SystemTiming::get_timestamp_ms();
	auto nameFull = source->get_NAME().get_full_name();
	auto &info = app->connectionTracker[nameFull];
	info.nameFull = nameFull;
	info.address = source->get_address();
	info.typeString = source->get_type_string();
	if (info.firstSeenMs == 0)
	{
		info.firstSeenMs = now;
	}
	info.workingSetMasterReceived = true;
	info.lastWorkingSetMasterMs = now;

	std::uint8_t numberOfMembers = message.get_data().empty() ? 0 : message.get_data()[0];
	std::cout << "[" << get_timestamp() << "] [CAN] WorkingSetMaster from NAME 0x" << std::hex << nameFull << std::dec
	          << " @ " << static_cast<int>(source->get_address())
	          << " - members=" << static_cast<int>(numberOfMembers) << std::endl;
}

void Application::log_can_process_data(const isobus::CANMessage &message, void *parent)
{
	if (nullptr == parent)
	{
		return;
	}
	auto *app = static_cast<Application *>(parent);
	auto source = message.get_source_control_function();
	if (nullptr == source)
	{
		return;
	}

	const auto &data = message.get_data();
	if (data.empty())
	{
		return;
	}

	std::uint8_t command = data[0] & 0x0F;
	std::uint8_t subcommand = data[0] >> 4;
	auto now = isobus::SystemTiming::get_timestamp_ms();
	auto nameFull = source->get_NAME().get_full_name();
	auto &info = app->connectionTracker[nameFull];
	info.nameFull = nameFull;
	info.address = source->get_address();
	info.typeString = source->get_type_string();
	if (info.firstSeenMs == 0)
	{
		info.firstSeenMs = now;
	}

	// Log RequestVersion (command 0x00, subcommand 0x00)
	if (command == 0x00 && subcommand == 0x00)
	{
		info.requestVersionReceived = true;
		info.lastRequestVersionMs = now;

		// Bidirectional version exchange: if a client asks our version, ask theirs too
		// This makes a V3-reporting TC behave like V4 for clients that expect it
		if (!info.requestVersionSent && app->tcCF && app->tcCF->get_address_valid())
		{
			std::array<std::uint8_t, 8> requestVersionPayload = {
				0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
			};
			if (isobus::CANNetworkManager::CANNetwork.send_can_message(
			      0xCB00, requestVersionPayload.data(), requestVersionPayload.size(), app->tcCF, source))
			{
				info.requestVersionSent = true;
				std::cout << "[" << get_timestamp() << "] [CAN] Sent RequestVersion to NAME 0x" << std::hex << nameFull << std::dec << std::endl;
			}
		}

		std::cout << "[" << get_timestamp() << "] [CAN] RequestVersion from NAME 0x" << std::hex << nameFull << std::dec
		          << " @ " << static_cast<int>(source->get_address()) << std::endl;
	}
	// Log ParameterVersion response (command 0x00, subcommand 0x01) - only if destination is the TC
	else if (command == 0x00 && subcommand == 0x01)
	{
		if (message.get_destination_control_function() == source)
		{
			// This is actually a Version response FROM the TC TO the client,
			// but if we see it going TO a client, it means the TC responded.
			info.versionResponseSent = true;
		}
	}
	// Log ClientTask (command 0x0F)
	else if (command == 0x0F)
	{
		info.clientTaskReceived = true;
		info.lastClientTaskMs = now;
	}
}

void Application::stop()
{
	tcServer->terminate();
	isobus::CANHardwareInterface::stop();
}
