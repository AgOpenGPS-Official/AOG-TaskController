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
		std::cout << "Unable to find a CAN driver. Please make sure the selected driver is installed." << std::endl;
		return false;
	}
	isobus::CANHardwareInterface::set_number_of_can_channels(1);
	isobus::CANHardwareInterface::assign_can_channel_frame_handler(0, canDriver);

	if ((!isobus::CANHardwareInterface::start()) || (!canDriver->get_is_valid()))
	{
		std::cout << "Failed to start CAN hardware interface." << std::endl;
		return false;
	}

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

	std::cout << "[Init] Creating Task Controller control function..." << std::endl;
	auto tcCF = isobus::CANNetworkManager::CANNetwork.create_internal_control_function(tcNAME, 0, isobus::preferred_addresses::IndustryGroup2::TaskController_MappingComputer); // The preferred address for a TC is defined in ISO 11783
	auto tcAddressClaimedFuture = std::async(std::launch::async, [&tcCF]() {
		while (!tcCF->get_address_valid())
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			isobus::CANNetworkManager::CANNetwork.update();
		}
	});

	// If this fails, probably the update thread is not started
	tcAddressClaimedFuture.wait_for(std::chrono::seconds(5));
	if (!tcCF->get_address_valid())
	{
		std::cout << "Failed to claim address for TC server. The control function might be invalid." << std::endl;
		return false;
	}

	// Create TECU control function
	// TODO: Should we wait between this and TC?
	// TODO: If there's already a TECU on the bus we should not create ours
	if (tcCF)
	{ // Only create TECU if TC was created
		std::cout << "[Init] Creating Tractor ECU control function..." << std::endl;
		tecuCF = isobus::CANNetworkManager::CANNetwork.create_internal_control_function(tecuNAME, 0, isobus::preferred_addresses::IndustryGroup2::TractorECU);
		std::cout << "[Init] Tractor ECU control function created, waiting 1.5 seconds..." << std::endl;

		// Update the network manager to process TECU CF claiming
		for (int i = 0; i < 15; i++)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			isobus::CANNetworkManager::CANNetwork.update();
		}
	}

	tcServer = std::make_shared<MyTCServer>(tcCF);
	auto &languageInterface = tcServer->get_language_command_interface();
	languageInterface.set_language_code("en"); // This is the default, but you can change it if you want
	languageInterface.set_country_code("US"); // This is the default, but you can change it if you want
	tcServer->initialize();
	tcServer->set_task_totals_active(true); // TODO: make this dynamic based on status in AOG

	// Initialize speed and distance messages
	if (tecuCF)
	{
		std::cout << "[Init] Creating Speed Messages Interface on TECU..." << std::endl;
		speedMessagesInterface = std::make_unique<isobus::SpeedMessagesInterface>(tecuCF, true, true, true, false); //TODO: make configurable whether to send these messages
		speedMessagesInterface->initialize();
		speedMessagesInterface->wheelBasedSpeedTransmitData.set_implement_start_stop_operations_state(isobus::SpeedMessagesInterface::WheelBasedMachineSpeedData::ImplementStartStopOperations::NotAvailable);
		speedMessagesInterface->wheelBasedSpeedTransmitData.set_key_switch_state(isobus::SpeedMessagesInterface::WheelBasedMachineSpeedData::KeySwitchState::NotAvailable);
		speedMessagesInterface->wheelBasedSpeedTransmitData.set_operator_direction_reversed_state(isobus::SpeedMessagesInterface::WheelBasedMachineSpeedData::OperatorDirectionReversed::NotAvailable);
		speedMessagesInterface->machineSelectedSpeedTransmitData.set_speed_source(isobus::SpeedMessagesInterface::MachineSelectedSpeedData::SpeedSource::NavigationBasedSpeed);
		std::cout << "[Init] Speed Messages Interface created and initialized." << std::endl;

		std::cout << "[Init] Creating NMEA2000 Message Interface on TECU..." << std::endl;
		nmea2000MessageInterface = std::make_unique<isobus::NMEA2000MessageInterface>(tecuCF, false, false, false, false, false, false, false);
		nmea2000MessageInterface->initialize();
		nmea2000MessageInterface->set_enable_sending_cog_sog_cyclically(true); // TODO: make configurable whether to send these messages
		std::cout << "[Init] NMEA2000 Message Interface created and initialized." << std::endl;
	}
	else
	{
		std::cout << "[Warning] TECU Control Function not available, Speed/NMEA interfaces not created" << std::endl;
	}

	std::cout << "Task controller server started." << std::endl;

	static std::uint8_t xteSid = 0;
	static std::uint32_t lastXteTransmit = 0;

	auto packetHandler = [this, tcCF](std::uint8_t src, std::uint8_t pgn, std::span<std::uint8_t> data) {
		if (src == 0x7F && pgn == 0xFE) // 254 - Steer Data
		{
			// TODO: hack to get desired section states. probably want to make a new pgn later when we need more than 16 sections
			std::vector<bool> sectionStates;
			for (std::uint8_t i = 0; i < 8; i++)
			{
				sectionStates.push_back(data[6] & (1 << i));
			}
			for (std::uint8_t i = 0; i < 8; i++)
			{
				sectionStates.push_back(data[7] & (1 << i));
			}

			tcServer->update_section_states(sectionStates);
		}
		else if (src == 0x7F && pgn == 0xF1) // 241 - Section Control
		{
			std::uint8_t sectionControlState = data[0];
			std::cout << "Received request from AOG to change section control state to " << (sectionControlState == 1 ? "enabled" : "disabled") << std::endl;
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

	std::cout << "UDP connections opened." << std::endl;

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
	if (isobus::SystemTiming::time_expired_ms(lastJ1939SpeedTransmit, 100) && tecuCF)
	{
		std::uint16_t speed_j1939 = (static_cast<std::uint32_t>(std::abs(lastSpeedValue)) * 576u + 312u) / 625u; // mm/s -> (km/h)*256
		std::array<std::uint8_t, 8> j1939_speed_data = {
			0xFF, 0xFF, // Compass Bearing (SPN 165)
			static_cast<std::uint8_t>(speed_j1939 & 0xFF), // Machine Speed LSB (byte 2)
			static_cast<std::uint8_t>((speed_j1939 >> 8) & 0xFF), // Machine Speed MSB (byte 3)
			0xFF, 0xFF, // Pitch (SPN 583)
			0xFF, 0xFF  // Altitude (SPN 580)
		};
		if (isobus::CANNetworkManager::CANNetwork.send_can_message(0xFEE8, j1939_speed_data.data(), j1939_speed_data.size(), tecuCF))
		{
			lastJ1939SpeedTransmit = isobus::SystemTiming::get_timestamp_ms();
		}
	}

	return true;
}

void Application::stop()
{
	tcServer->terminate();
	isobus::CANHardwareInterface::stop();
}
