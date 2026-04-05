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
#include "isobus/isobus/can_network_manager.hpp"
#include "isobus/isobus/isobus_preferred_addresses.hpp"
#include "isobus/isobus/isobus_standard_data_description_indices.hpp"
#include "isobus/utility/system_timing.hpp"

#include "task_controller.hpp"

#include <chrono>
#include <iomanip>
#include <sstream>

using boost::asio::ip::udp;

// Utility function to get current timestamp string (HH:MM:SS.mmm)
std::string get_timestamp()
{
	auto now = std::chrono::system_clock::now();
	auto time_t_now = std::chrono::system_clock::to_time_t(now);
	auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

	std::tm localTime;
	localtime_s(&localTime, &time_t_now);

	std::ostringstream oss;
	oss << std::setfill('0') << std::setw(2) << localTime.tm_hour << ":"
	    << std::setfill('0') << std::setw(2) << localTime.tm_min << ":"
	    << std::setfill('0') << std::setw(2) << localTime.tm_sec << "."
	    << std::setfill('0') << std::setw(3) << ms.count();
	return oss.str();
}

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

	// Wait for TC address claim with minimum 250ms delay per J1939-81 section 4.4.4.1
	// CAs with addresses in range 128-247 must wait 250ms after claiming before sending other messages
	static constexpr std::uint32_t MINIMUM_ADDRESS_CLAIM_DELAY_MS = 250;
	auto tcClaimStartTime = isobus::SystemTiming::get_timestamp_ms();
	auto tcAddressClaimedFuture = std::async(std::launch::async, [this]() {
		while (!this->tcCF->get_address_valid())
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
			isobus::CANNetworkManager::CANNetwork.update();
		}
	});

	// If this fails, probably the update thread is not started
	tcAddressClaimedFuture.wait_for(std::chrono::seconds(5));
	if (!tcCF->get_address_valid())
	{
		std::cout << "[" << get_timestamp() << "] Failed to claim address for TC server. The control function might be invalid." << std::endl;
		return false;
	}

	// Ensure minimum 250ms delay after address claim per J1939-81
	auto tcClaimElapsedMs = isobus::SystemTiming::get_time_elapsed_ms(tcClaimStartTime);
	if (tcClaimElapsedMs < MINIMUM_ADDRESS_CLAIM_DELAY_MS)
	{
		auto remainingDelay = MINIMUM_ADDRESS_CLAIM_DELAY_MS - tcClaimElapsedMs;
		std::cout << "[" << get_timestamp() << "] [Init] TC claimed address " << static_cast<int>(tcCF->get_address()) << ", waiting " << remainingDelay << "ms (J1939-81 250ms rule)..." << std::endl;
		// Process CAN messages during the delay to prevent timeouts
		auto delayStart = isobus::SystemTiming::get_timestamp_ms();
		while (isobus::SystemTiming::get_time_elapsed_ms(delayStart) < remainingDelay)
		{
			isobus::CANNetworkManager::CANNetwork.update();
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}
	else
	{
		std::cout << "[" << get_timestamp() << "] [Init] TC claimed address " << static_cast<int>(tcCF->get_address()) << " (250ms delay already satisfied)" << std::endl;
	}

	// Create TECU control function
	// TODO: Should we wait between this and TC?
	// TODO: If there's already a TECU on the bus we should not create ours
	if (tcCF && settings->is_tecu_enabled())
	{ // Only create TECU if TC was created and ECU is enabled
		std::cout << "[" << get_timestamp() << "] [Init] Creating Tractor ECU control function..." << std::endl;
		tecuCF = isobus::CANNetworkManager::CANNetwork.create_internal_control_function(tecuNAME, 0, isobus::preferred_addresses::IndustryGroup2::TractorECU);

		// Wait for TECU address claim with minimum 250ms delay per J1939-81 section 4.4.4.1
		auto tecuClaimStartTime = isobus::SystemTiming::get_timestamp_ms();
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

		// Check if TECU successfully claimed its address
		if (tecuCF->get_address_valid())
		{
			// Ensure minimum 250ms delay after address claim per J1939-81
			auto tecuClaimElapsedMs = isobus::SystemTiming::get_time_elapsed_ms(tecuClaimStartTime);
			if (tecuClaimElapsedMs < MINIMUM_ADDRESS_CLAIM_DELAY_MS)
			{
				auto remainingDelay = MINIMUM_ADDRESS_CLAIM_DELAY_MS - tecuClaimElapsedMs;
				std::cout << "[" << get_timestamp() << "] [Init] Tractor ECU claimed address " << static_cast<int>(tecuCF->get_address()) << ", waiting " << remainingDelay << "ms (J1939-81 250ms rule)..." << std::endl;
				// Process CAN messages during the delay to prevent timeouts
				auto delayStart = isobus::SystemTiming::get_timestamp_ms();
				while (isobus::SystemTiming::get_time_elapsed_ms(delayStart) < remainingDelay)
				{
					isobus::CANNetworkManager::CANNetwork.update();
					std::this_thread::sleep_for(std::chrono::milliseconds(10));
				}
			}
			else
			{
				std::cout << "[" << get_timestamp() << "] [Init] Tractor ECU claimed address " << static_cast<int>(tecuCF->get_address()) << " (250ms delay already satisfied)" << std::endl;
			}
		}
		else
		{
			std::cout << "[" << get_timestamp() << "] [Warning] TECU failed to claim address 128! Another TECU may be on the bus." << std::endl;
			std::cout << "[" << get_timestamp() << "] [Warning] TECU functionality will be disabled." << std::endl;
			tecuCF.reset(); // Release the failed control function
		}
	}

	tcServer = std::make_shared<MyTCServer>(tcCF);
	auto &languageInterface = tcServer->get_language_command_interface();
	languageInterface.set_language_code("en"); // This is the default, but you can change it if you want
	languageInterface.set_country_code("US"); // This is the default, but you can change it if you want
	tcServer->initialize();
	tcServer->set_task_totals_active(true); // TODO: make this dynamic based on status in AOG

	// Initialize speed and distance messages
	if (tecuCF && tecuCF->get_address_valid())
	{
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
	static std::uint32_t lastTECUStatusTransmit = 0;

	udpConnections->handle_address_detection();
	udpConnections->handle_incoming_packets();

	tcServer->request_measurement_commands();
	tcServer->update();
	if (speedMessagesInterface)
		speedMessagesInterface->update();
	if (nmea2000MessageInterface)
		nmea2000MessageInterface->update();

	if (isobus::SystemTiming::time_expired_ms(lastHeartbeatTransmit, 100))
	{
		for (auto &client : tcServer->get_clients())
		{
			auto &state = client.second;

			// Skip clients with no sections (e.g., tractors or non-implement devices)
			if (state.get_number_of_sections() == 0)
			{
				continue;
			}

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

	// Send Task Controller Status message every 2 seconds (ISO 11783-10 B.8.1)
	if (isobus::SystemTiming::time_expired_ms(lastTCStatusTransmit, 2000) && tcCF && tcCF->get_address_valid())
	{
		static bool firstStatusSent = false;
		send_task_controller_status_message();
		if (!firstStatusSent)
		{
			std::cout << "[" << get_timestamp() << "] [TC Status] First TC Status message sent (PGN 0xCB00)" << std::endl;
			firstStatusSent = true;
		}
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
	if (taskTotalsActive)
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

	// Send to global destination (0xFF) - broadcast to all nodes
	// Using 4-arg version: PGN, data, length, source CF (destination is implicit in PGN for broadcast)
	if (isobus::CANNetworkManager::CANNetwork.send_can_message(0xCB00, tcStatusData.data(), tcStatusData.size(), tcCF))
	{
		lastTCStatusTransmit = isobus::SystemTiming::get_timestamp_ms();
	}
	else
	{
		std::cout << "[" << get_timestamp() << "] [TC Status] Failed to send TC Status message!" << std::endl;
	}
}

void Application::stop()
{
	tcServer->terminate();
	isobus::CANHardwareInterface::stop();
}
