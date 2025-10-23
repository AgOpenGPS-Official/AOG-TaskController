/**
 * @author Daan Steenbergen
 * @brief An ISOBUS Task Controller for AgOpenGPS
 * @version 0.1
 * @date 2025-1-20
 *
 * @copyright 2025 Daan Steenbergen
 */
#include "task_controller.hpp"
#include "settings.hpp"

#include "isobus/isobus/isobus_device_descriptor_object_pool_helpers.hpp"
#include "isobus/isobus/isobus_task_controller_server.hpp"

#include <bitset>
#include <fstream>
#include <iostream>

void ClientState::set_number_of_sections(std::uint8_t number)
{
	numberOfSections = number;
	sectionSetpointStates.resize(number);
	sectionActualStates.resize(number);
}

void ClientState::set_section_setpoint_state(std::uint8_t section, std::uint8_t state)
{
	if (section < numberOfSections)
	{
		sectionSetpointStates[section] = state;
	}
}

void ClientState::set_section_actual_state(std::uint8_t section, std::uint8_t state)
{
	if (section < numberOfSections)
	{
		sectionActualStates[section] = state;
	}
}

std::uint8_t ClientState::get_number_of_sections() const
{
	return numberOfSections;
}

std::uint8_t ClientState::get_section_setpoint_state(std::uint8_t section) const
{
	if (section < numberOfSections)
	{
		return sectionSetpointStates[section];
	}
	return SectionState::NOT_INSTALLED;
}

std::uint8_t ClientState::get_section_actual_state(std::uint8_t section) const
{
	if (section < numberOfSections)
	{
		return sectionActualStates[section];
	}
	return SectionState::NOT_INSTALLED;
}

bool ClientState::is_any_section_setpoint_on() const
{
	for (std::uint8_t state : sectionSetpointStates)
	{
		if (state == SectionState::ON)
		{
			return true;
		}
	}
	return false;
}

bool ClientState::get_setpoint_work_state() const
{
	return setpointWorkState;
}

void ClientState::set_setpoint_work_state(bool state)
{
	setpointWorkState = state;
}

bool ClientState::get_actual_work_state() const
{
	return actualWorkState;
}

void ClientState::set_actual_work_state(bool state)
{
	actualWorkState = state;
}

bool ClientState::is_section_control_enabled() const
{
	return isSectionControlEnabled;
}

void ClientState::set_section_control_enabled(bool state)
{
	isSectionControlEnabled = state;
}

isobus::DeviceDescriptorObjectPool &ClientState::get_pool()
{
	return pool;
}

bool ClientState::are_measurement_commands_sent() const
{
	return areMeasurementCommandsSent;
}

void ClientState::mark_measurement_commands_sent()
{
	areMeasurementCommandsSent = true;
}

std::uint16_t ClientState::get_element_number_for_ddi(isobus::DataDescriptionIndex ddi) const
{
	auto it = ddiToElementNumber.find(ddi);
	if (it != ddiToElementNumber.end())
	{
		return it->second;
	}
	std::cout << "Cached element number not found for DDI " << static_cast<int>(ddi) << std::endl;
	return 0;
}

bool ClientState::try_get_element_number_for_ddi(isobus::DataDescriptionIndex ddi, std::uint16_t& elementNumber) const
{
	auto it = ddiToElementNumber.find(ddi);
	if (it != ddiToElementNumber.end())
	{
		elementNumber = it->second;
		return true;
	}
	std::cout << "Cached element number not found for DDI " << static_cast<int>(ddi) << std::endl;
	return false;
}

void ClientState::set_element_number_for_ddi(isobus::DataDescriptionIndex ddi, std::uint16_t elementNumber)
{
	ddiToElementNumber[ddi] = elementNumber;
}

bool ClientState::get_left_tramline_state() const
{
	return leftTramlineState;
}

void ClientState::set_left_tramline_state(bool state)
{
	leftTramlineState = state;
}

bool ClientState::get_right_tramline_state() const
{
    return rightTramlineState;
}

void ClientState::set_right_tramline_state(bool state)
{
    rightTramlineState = state;
}

void ClientState::set_track_number_to_right(std::int32_t track)
{
    trackNumberToRight = track;
}

std::int32_t ClientState::get_track_number_to_right() const
{
    return trackNumberToRight;
}

void ClientState::set_track_number_to_left(std::int32_t track)
{
    trackNumberToLeft = track;
}

std::int32_t ClientState::get_track_number_to_left() const
{
    return trackNumberToLeft;
}

void ClientState::set_unique_ab_reference_id(std::int32_t id)
{
    uniqueABReferenceID = id;
}

std::int32_t ClientState::get_unique_ab_reference_id() const
{
    return uniqueABReferenceID;
}

void ClientState::set_tramline_sequence_number(std::int32_t sequence)
{
    tramlineSequenceNumber = sequence;
}

std::int32_t ClientState::get_tramline_sequence_number() const
{
    return tramlineSequenceNumber;
}

void ClientState::set_tramline_control_level_support(std::uint8_t supportBits)
{
    tramlineControlLevelSupport = supportBits;
}

std::uint8_t ClientState::get_tramline_control_level_support() const
{
    return tramlineControlLevelSupport;
}

void ClientState::set_selected_tramline_control_level(std::uint8_t level)
{
    selectedTramlineControlLevel = level;
}

std::uint8_t ClientState::get_selected_tramline_control_level() const
{
    return selectedTramlineControlLevel;
}

void ClientState::set_track_number(std::uint16_t track)
{
    trackNumber = track;
}

std::uint16_t ClientState::get_track_number() const
{
    return trackNumber;
}

void ClientState::set_last_tramline_control_state_sent(std::uint8_t state)
{
    lastSentTramlineControlState = state;
}

std::uint8_t ClientState::get_last_tramline_control_state_sent() const
{
    return lastSentTramlineControlState;
}

MyTCServer::MyTCServer(std::shared_ptr<isobus::InternalControlFunction> internalControlFunction) :
  TaskControllerServer(internalControlFunction,
                       1, // AOG limits to 1 boom
                       16, // AOG limits to 16 sections of unique width
                       16, // 16 channels for position based control
                       isobus::TaskControllerOptions()
                         .with_implement_section_control(), // We support section control
                       TaskControllerVersion::SecondEditionDraft)
{
}

bool MyTCServer::activate_object_pool(std::shared_ptr<isobus::ControlFunction> partnerCF, ObjectPoolActivationError &, ObjectPoolErrorCodes &, std::uint16_t &, std::uint16_t &)
{
	// Safety check to make sure partnerCF has uploaded a DDOP
	if (uploadedPools.find(partnerCF) == uploadedPools.end())
	{
		return false;
	}

	// Initialize a new client state
	auto state = ClientState();
	// state.get_pool().set_task_controller_compatibility_level(get_active_client(partnerCF)->reportedVersion);
	state.get_pool().set_task_controller_compatibility_level(static_cast<std::uint8_t>(TaskControllerVersion::SecondEditionDraft));

	bool deserialized = false;
	while (!uploadedPools[partnerCF].empty())
	{
		auto binaryPool = uploadedPools[partnerCF].front();
		uploadedPools[partnerCF].pop();
		deserialized = state.get_pool().deserialize_binary_object_pool(binaryPool.data(), static_cast<std::uint32_t>(binaryPool.size()), partnerCF->get_NAME());
	}
	if (deserialized)
	{
		std::cout << "Successfully deserialized device descriptor object pool." << std::endl;

		// Save to NVM
		std::shared_ptr<isobus::task_controller_object::DeviceObject> deviceObject;
		for (std::uint16_t i = 0; i < state.get_pool().size(); i++)
		{
			auto object = state.get_pool().get_object_by_index(i);
			if (object->get_object_type() == isobus::task_controller_object::ObjectTypes::Device)
			{
				deviceObject = std::static_pointer_cast<isobus::task_controller_object::DeviceObject>(object);
				break;
			}
		}
		auto fileName = std::to_string(partnerCF->get_NAME().get_full_name()) + "\\" + std::string(deviceObject->get_localization_label().begin(), deviceObject->get_localization_label().end()) + ".iop";
		std::vector<std::uint8_t> binaryPool;
		if (state.get_pool().generate_binary_object_pool(binaryPool))
		{
			std::ofstream outFile(Settings::get_filename_path(fileName), std::ios::binary);
			if (outFile.is_open())
			{
				outFile.write(reinterpret_cast<const char *>(binaryPool.data()), binaryPool.size());
				outFile.close();
			}
			else
			{
				std::cout << "Unable to save DDOP to NVM. (Failed to open file)" << std::endl;
			}
		}
		else
		{
			std::cout << "Unable to save DDOP to NVM. (Failed to generate binary object pool)" << std::endl;
		}

		auto implement = isobus::DeviceDescriptorObjectPoolHelper::get_implement_geometry(state.get_pool());
		std::uint8_t numberOfSections = 0;

		std::cout << "Implement geometry: " << std::endl;
		std::cout << "Number of booms=" << implement.booms.size() << std::endl;
		for (const auto &boom : implement.booms)
		{
			std::cout << "Boom: id=" << static_cast<int>(boom.elementNumber) << std::endl;
			for (const auto &subBoom : boom.subBooms)
			{
				std::cout << "SubBoom: id=" << static_cast<int>(subBoom.elementNumber) << std::endl;
				for (const auto &section : subBoom.sections)
				{
					numberOfSections++;
					std::cout << "Section: id=" << static_cast<int>(section.elementNumber) << std::endl;
					std::cout << "X Offset: " << section.xOffset_mm.get() << std::endl;
					std::cout << "Y Offset: " << section.yOffset_mm.get() << std::endl;
					std::cout << "Z Offset: " << section.zOffset_mm.get() << std::endl;
					std::cout << "Width: " << section.width_mm.get() << std::endl;
				}
			}
			for (const auto &section : boom.sections)
			{
				numberOfSections++;
				std::cout << "Section: id=" << static_cast<int>(section.elementNumber) << std::endl;
				std::cout << "X Offset: " << section.xOffset_mm.get() << std::endl;
				std::cout << "Y Offset: " << section.yOffset_mm.get() << std::endl;
				std::cout << "Z Offset: " << section.zOffset_mm.get() << std::endl;
				std::cout << "Width: " << section.width_mm.get() << std::endl;
			}
		}
		state.set_number_of_sections(numberOfSections);
	}
	else
	{
		std::cout << "Failed to deserialize device descriptor object pool." << std::endl;
		return false;
	}

	clients[partnerCF] = state;
	return true;
}

bool MyTCServer::change_designator(std::shared_ptr<isobus::ControlFunction>, std::uint16_t, const std::vector<std::uint8_t> &)
{
	return true;
}

bool MyTCServer::deactivate_object_pool(std::shared_ptr<isobus::ControlFunction> partnerCF)
{
	clients.erase(partnerCF);
	uploadedPools.erase(partnerCF);
	return true;
}

bool MyTCServer::delete_device_descriptor_object_pool(std::shared_ptr<isobus::ControlFunction> partnerCF, ObjectPoolDeletionErrors &)
{
	clients.erase(partnerCF);
	uploadedPools.erase(partnerCF);
	return true;
}

bool MyTCServer::get_is_stored_device_descriptor_object_pool_by_structure_label(std::shared_ptr<isobus::ControlFunction>, const std::vector<std::uint8_t> &, const std::vector<std::uint8_t> &)
{
	return false;
}

bool MyTCServer::get_is_stored_device_descriptor_object_pool_by_localization_label(std::shared_ptr<isobus::ControlFunction>, const std::array<std::uint8_t, 7> &)
{
	return false;
}

bool MyTCServer::get_is_enough_memory_available(std::uint32_t)
{
	return true;
}

void MyTCServer::identify_task_controller(std::uint8_t)
{
	// When this is called, the TC is supposed to display its TC number for 3 seconds if possible (which is passed into this function).
	// Your TC's number is your function code + 1, in the range of 1-32.
}

void MyTCServer::on_client_timeout(std::shared_ptr<isobus::ControlFunction> partner)
{
	// Cleanup the client state
	clients.erase(partner);
}

void MyTCServer::on_process_data_acknowledge(std::shared_ptr<isobus::ControlFunction> partner,
                                             std::uint16_t dataDescriptionIndex,
                                             std::uint16_t elementNumber,
                                             std::uint8_t errorCodesFromClient,
                                             ProcessDataCommands processDataCommand)
{
	// This callback lets you know when a client sends a process data acknowledge (PDACK) message to you
	std::cout << "Received process data acknowledge from client " << int(partner->get_address()) << " for DDI " << dataDescriptionIndex << " (" << pdackEntry.to_string() << ") element " << elementNumber << " with error codes " << std::bitset<8>(errorCodesFromClient) << " and command " << static_cast<int>(processDataCommand) << std::endl;
}

bool MyTCServer::on_value_command(std::shared_ptr<isobus::ControlFunction> partner,
                                  std::uint16_t dataDescriptionIndex,
                                  std::uint16_t elementNumber,
                                  std::int32_t processDataValue,
                                  std::uint8_t &errorCodes)
{
	// Human-readable DDI log for incoming value commands
	const auto &entry = isobus::DataDictionary::get_entry(dataDescriptionIndex);
	std::cout << "PD value from client " << int(partner->get_address())
			<< ": DDI " << dataDescriptionIndex << " (" << entry.to_string() << ")"
			<< ", element " << elementNumber << ", value " << processDataValue
			<< " (" << entry.format_value(processDataValue) << ")" << std::endl;

	switch (dataDescriptionIndex)
	{
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::TramlineControlState): // Tramline Control State (DDI 515)
        {
            std::uint8_t bits = static_cast<std::uint8_t>(processDataValue) & 0x03;
            const char *mode = (bits == 0) ? "manual/off" : (bits == 1) ? "automatic/on" : (bits == 2) ? "error" : "undefined";
            std::cout << "Implement Tramline Control State: " << mode << std::endl;
        }
        break;
        case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::TramlineControlLevel): // Tramline Control Level (DDI 505)
        {
            std::uint8_t support = static_cast<std::uint8_t>(processDataValue) & 0x07;
            clients[partner].set_tramline_control_level_support(support);
            bool l1 = (support & 0x01) != 0;
            bool l2 = (support & 0x02) != 0;
            bool l3 = (support & 0x04) != 0;
            std::cout << "Implement Tramline Control Level support: L1=" << (l1 ? "Yes" : "No")
                      << ", L2=" << (l2 ? "Yes" : "No")
                      << ", L3=" << (l3 ? "Yes" : "No") << std::endl;

            // Choose a common level with TC support. TC supports Level 1 only for now.
            constexpr std::uint8_t tcSupportedMask = 0x01; // Level 1
            std::uint8_t chosen = 0; // 0 = No common level
            if ((support & tcSupportedMask) != 0)
            {
                chosen = 1; // Use Level 1
            }

            if (clients[partner].get_selected_tramline_control_level() != chosen)
            {
                clients[partner].set_selected_tramline_control_level(chosen);

                // Send DDI 506 Setpoint Tramline Control Level to inform implement
                std::uint16_t elem = clients[partner].get_element_number_for_ddi(isobus::DataDescriptionIndex::SetpointTramlineControlLevel);
                if (elem != 0)
                {
                    send_set_value(partner, static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointTramlineControlLevel), elem, chosen);
                    std::cout << "Setpoint Tramline Control Level (DDI 506) sent: " << int(chosen) << " (element " << elem << ")" << std::endl;
                }
                else
                {
                    std::cout << "DDI 506 element not found; unable to send Setpoint Tramline Control Level." << std::endl;
                }
            }
        }
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState1_16):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState17_32):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState33_48):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState49_64):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState65_80):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState81_96):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState97_112):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState113_128):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState129_144):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState145_160):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState161_176):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState177_192):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState193_208):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState209_224):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState225_240):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState241_256):
		{
			std::uint8_t sectionIndexOffset = NUMBER_SECTIONS_PER_CONDENSED_MESSAGE * static_cast<std::uint8_t>(dataDescriptionIndex - static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState1_16));

			for (std::uint_fast8_t i = 0; i < NUMBER_SECTIONS_PER_CONDENSED_MESSAGE; i++)
			{
				clients[partner].set_section_actual_state(i + sectionIndexOffset, (processDataValue >> (2 * i)) & 0x03);
			}
		}
		break;

		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SectionControlState):
		{
			clients[partner].set_section_control_enabled(processDataValue == 1);
		}
		break;

		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualWorkState):
		{
			clients[partner].set_setpoint_work_state(processDataValue == 1);
		}
	}

	return true;
}

bool MyTCServer::store_device_descriptor_object_pool(std::shared_ptr<isobus::ControlFunction> partnerCF, const std::vector<std::uint8_t> &binaryPool, bool appendToPool)
{
	if (uploadedPools.find(partnerCF) == uploadedPools.end())
	{
		uploadedPools[partnerCF] = std::queue<std::vector<std::uint8_t>>();
	}
	uploadedPools[partnerCF].push(binaryPool);
	return true;
}

std::map<std::shared_ptr<isobus::ControlFunction>, ClientState> &MyTCServer::get_clients()
{
	return clients;
}

void MyTCServer::request_measurement_commands()
{
	for (auto &client : clients)
	{
		if (!client.second.are_measurement_commands_sent())
		{
			// Find all actual (condensed) work state DDIs and request them to trigger "On Change" and "Time Interval"
			for (std::uint32_t i = 0; i < client.second.get_pool().size(); i++)
			{
				auto object = client.second.get_pool().get_object_by_index(i);
				if (object->get_object_type() == isobus::task_controller_object::ObjectTypes::DeviceProcessData)
				{
					auto processDataObject = std::dynamic_pointer_cast<isobus::task_controller_object::DeviceProcessDataObject>(object);
					if (processDataObject->get_ddi() == static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualWorkState) ||
					    (processDataObject->get_ddi() >= static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState1_16) &&
					     processDataObject->get_ddi() <= static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState241_256)))
					{
						// Loop over all objects to find the elements that are the parents of the actual condensed work state objects
						for (std::uint32_t j = 0; j < client.second.get_pool().size(); j++)
						{
							auto parentObject = client.second.get_pool().get_object_by_index(j);
							if (parentObject->get_object_type() == isobus::task_controller_object::ObjectTypes::DeviceElement)
							{
								auto elementObject = std::dynamic_pointer_cast<isobus::task_controller_object::DeviceElementObject>(parentObject);
								for (std::uint16_t elementObjectChild : elementObject->get_child_object_ids())
								{
									if (elementObjectChild == processDataObject->get_object_id())
									{
										// TODO: This is a bit of a hack, but it works for now
										client.second.set_element_number_for_ddi(static_cast<isobus::DataDescriptionIndex>(processDataObject->get_ddi()), elementObject->get_element_number());

										if (processDataObject->has_trigger_method(isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::OnChange))
										{
											send_change_threshold_measurement_command(client.first, processDataObject->get_ddi(), elementObject->get_element_number(), 1);
										}
										if (processDataObject->has_trigger_method(isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::TimeInterval))
										{
											send_time_interval_measurement_command(client.first, processDataObject->get_ddi(), elementObject->get_element_number(), 1000);
										}
									}
								}
							}
						}
					}
				}
			}

			// Find all section control state DDIs and request them to trigger "On Change"
			for (std::uint32_t i = 0; i < client.second.get_pool().size(); i++)
			{
				auto object = client.second.get_pool().get_object_by_index(i);
				if (object->get_object_type() == isobus::task_controller_object::ObjectTypes::DeviceProcessData)
				{
					auto processDataObject = std::dynamic_pointer_cast<isobus::task_controller_object::DeviceProcessDataObject>(object);
					if (processDataObject->get_ddi() == static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SectionControlState) ||
					    processDataObject->get_ddi() == static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointWorkState) ||
					    (processDataObject->get_ddi() >= static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointCondensedWorkState1_16) &&
					     processDataObject->get_ddi() <= static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointCondensedWorkState241_256)) ||
					     processDataObject->get_ddi() == static_cast<std::uint16_t>(isobus::DataDescriptionIndex::TramlineControlState) || // Tramline Control State (DDI 515)
					     processDataObject->get_ddi() == static_cast<std::uint16_t>(isobus::DataDescriptionIndex::TramlineControlLevel) || // Tramline Control Level (DDI 505)
					     processDataObject->get_ddi() == static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointTramlineControlLevel) || // Setpoint Tramline Control Level (DDI 506)
					     processDataObject->get_ddi() == static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualTrackNumber)) // Actual Track Number (DDI 509)
					{
						// Loop over all objects to find the elements that are the parents of the section control state objects
						for (std::uint32_t j = 0; j < client.second.get_pool().size(); j++)
						{
							auto parentObject = client.second.get_pool().get_object_by_index(j);
							if (parentObject->get_object_type() == isobus::task_controller_object::ObjectTypes::DeviceElement)
							{
								auto elementObject = std::dynamic_pointer_cast<isobus::task_controller_object::DeviceElementObject>(parentObject);
								for (std::uint16_t elementObjectChild : elementObject->get_child_object_ids())
								{
									if (elementObjectChild == processDataObject->get_object_id())
									{
										// TODO: This is a bit of a hack, but it works for now
										client.second.set_element_number_for_ddi(static_cast<isobus::DataDescriptionIndex>(processDataObject->get_ddi()), elementObject->get_element_number());
										const auto &entryB = isobus::DataDictionary::get_entry(processDataObject->get_ddi());
										std::cout << "Mapped DDI " << processDataObject->get_ddi() << " (" << entryB.to_string() << ") to element " 
												<< elementObject->get_element_number() << std::endl;

										if (processDataObject->has_trigger_method(isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::OnChange))
										{
											send_change_threshold_measurement_command(client.first, processDataObject->get_ddi(), elementObject->get_element_number(), 1);
											std::cout << "Subscribed (OnChange) to DDI " << processDataObject->get_ddi() << " (" << entryB.to_string() << ") for element " 
												<< elementObject->get_element_number() << std::endl;
										}
										if (processDataObject->get_ddi() == static_cast<std::uint16_t>(isobus::DataDescriptionIndex::TramlineControlLevel)) // Tramline Control Level (DDI 505)
										{
											// Ask for periodic update so we receive the supported levels at least once
											send_time_interval_measurement_command(client.first, processDataObject->get_ddi(), elementObject->get_element_number(), 5000);
											std::cout << "Requested Tramline Control Level (DDI 505) from element " << elementObject->get_element_number() << std::endl;
										}
									}
								}
							}
						}
					}
				}
			}

			std::cout << "Measurement commands sent." << std::endl;
			client.second.mark_measurement_commands_sent();
		}
	}
}

void MyTCServer::update_section_states(std::vector<bool> &sectionStates)
{
	for (auto &client : clients)
	{
		auto &state = client.second;
		if (!state.is_section_control_enabled())
		{
			// According to standard, the section setpoint states should only be sent when in auto mode
			return;
		}

		bool requiresUpdate = false;
		for (std::uint8_t i = 0; i < state.get_number_of_sections(); i++)
		{
			if ((i % NUMBER_SECTIONS_PER_CONDENSED_MESSAGE == 0) && requiresUpdate)
			{
				// Send the previous 16 sections
				std::uint8_t ddiOffset = (i / NUMBER_SECTIONS_PER_CONDENSED_MESSAGE) - 1;
				send_section_setpoint_states(client.first, ddiOffset);
				requiresUpdate = false;
			}

			if (i < sectionStates.size())
			{
				if (sectionStates[i] != (state.get_section_setpoint_state(i) == SectionState::ON))
				{
					state.set_section_setpoint_state(i, sectionStates[i] ? SectionState::ON : SectionState::OFF);
					requiresUpdate = true;
				}
			}
		}
		if (requiresUpdate)
		{
			std::uint8_t ddiOffset = state.get_number_of_sections() / NUMBER_SECTIONS_PER_CONDENSED_MESSAGE;
			send_section_setpoint_states(client.first, ddiOffset);
		}
	}
}

void MyTCServer::update_section_control_enabled(bool enabled)
{
	for (auto &client : clients)
	{
		if (client.second.is_section_control_enabled() != enabled)
		{
			client.second.set_section_control_enabled(enabled);
			send_section_control_state(client.first, enabled);

			// Reuse SC toggle to drive Tramline Control State (DDI 515) for Level 1/2
            std::uint16_t ctlElem = 0;
            if (client.second.try_get_element_number_for_ddi(isobus::DataDescriptionIndex::TramlineControlState, ctlElem)) // Tramline Control State (DDI 515)
            {
                const std::uint8_t desired = enabled ? 1 : 0; // 01b automatic/on when SC enabled, 00b manual/off when disabled
                if (client.second.get_last_tramline_control_state_sent() != desired)
                {
                    send_set_value(client.first, static_cast<std::uint16_t>(isobus::DataDescriptionIndex::TramlineControlState), ctlElem, desired); // Tramline Control State (DDI 515)
                    client.second.set_last_tramline_control_state_sent(desired);
                }
            }
		}
	}
}

void MyTCServer::update_tramline_states(bool leftTram, bool rightTram)
{
    for (auto &client : clients)
    {
        bool oldLeft = client.second.get_left_tramline_state();
        bool oldRight = client.second.get_right_tramline_state();
        client.second.set_left_tramline_state(leftTram);
        client.second.set_right_tramline_state(rightTram);
        
        // For Level 1 tramline control, we increment track number when both tramlines become active
        bool oldBothActive = oldLeft && oldRight;
        bool newBothActive = leftTram && rightTram;
        
        // When both tramlines become active, trigger the next tramline sequence
        if (!oldBothActive && newBothActive)
        {
            handle_tramline_sequence(client.first);
        }
    }
}

void MyTCServer::handle_tramline_sequence(std::shared_ptr<isobus::ControlFunction> client)
{
    auto &state = clients[client];
    
    // Get the current track number
    std::uint16_t currentTrack = state.get_track_number();
    
    // Check if both tramlines are now active (this is when we increment the track number)
    bool leftTramActive = state.get_left_tramline_state();
    bool rightTramActive = state.get_right_tramline_state();
    if(leftTramActive && rightTramActive) currentTrack++;
    
    // For testing purposes, we'll increment the track number when both tramlines are active
    state.set_track_number(currentTrack);
    
    // Increment the sequence number (wrapping at 2147483647)
    std::int32_t sequenceNumber = state.get_tramline_sequence_number() + 1;
    if (sequenceNumber >= 2147483647) {
        sequenceNumber = 0;
    }
    state.set_tramline_sequence_number(sequenceNumber);
    
    // Hardcode the Unique A-B Guidance Reference Line ID to 0 for now
    std::int32_t uniqueABReferenceID = 0;
    state.set_unique_ab_reference_id(uniqueABReferenceID);
    
    // Calculate track numbers to the left and right (relative to current track)
    std::int32_t trackToLeft = static_cast<std::int32_t>(currentTrack) - 1;
    std::int32_t trackToRight = static_cast<std::int32_t>(currentTrack) + 1;
    state.set_track_number_to_left(trackToLeft);
    state.set_track_number_to_right(trackToRight);
    
    // Send the tramline sequence data to the implement using the correct DDI values
    std::cout << "Tramline sequence triggered:" << std::endl;
    std::cout << "  Sequence Number: " << sequenceNumber << std::endl;
    std::cout << "  Unique A-B Reference ID: " << uniqueABReferenceID << std::endl;
    std::cout << "  Current Track: " << currentTrack << std::endl;
    std::cout << "  Track to Left: " << trackToLeft << std::endl;
    std::cout << "  Track to Right: " << trackToRight << std::endl;
    
    // Send Tramline Sequence Number (DDI 507)
    std::uint16_t elem507 = state.get_element_number_for_ddi(isobus::DataDescriptionIndex::TramlineSequenceNumber);
    if (elem507 != 0) {
        send_set_value(client, static_cast<std::uint16_t>(isobus::DataDescriptionIndex::TramlineSequenceNumber), elem507, sequenceNumber);
        std::cout << "Sent Tramline Sequence Number (DDI 507): " << sequenceNumber << std::endl;
    }
    
    // Send Unique A-B Guidance Reference Line ID (DDI 508)
    std::uint16_t elem508 = state.get_element_number_for_ddi(isobus::DataDescriptionIndex::UniqueABGuidanceReferenceLineID);
    if (elem508 != 0) {
        send_set_value(client, static_cast<std::uint16_t>(isobus::DataDescriptionIndex::UniqueABGuidanceReferenceLineID), elem508, uniqueABReferenceID);
        std::cout << "Sent Unique A-B Guidance Reference Line ID (DDI 508): " << uniqueABReferenceID << std::endl;
    }
    
    // Send Track Number to the Right (DDI 510)
    std::uint16_t elem510 = state.get_element_number_for_ddi(isobus::DataDescriptionIndex::TrackNumberToTheRight);
    if (elem510 != 0) {
        send_set_value(client, static_cast<std::uint16_t>(isobus::DataDescriptionIndex::TrackNumberToTheRight), elem510, trackToRight);
        std::cout << "Sent Track Number to the Right (DDI 510): " << trackToRight << std::endl;
    }
    
    // Send Track Number to the Left (DDI 511)
    std::uint16_t elem511 = state.get_element_number_for_ddi(isobus::DataDescriptionIndex::TrackNumberToTheLeft);
    if (elem511 != 0) {
        send_set_value(client, static_cast<std::uint16_t>(isobus::DataDescriptionIndex::TrackNumberToTheLeft), elem511, trackToLeft);
        std::cout << "Sent Track Number to the Left (DDI 511): " << trackToLeft << std::endl;
    }
    
    // Send Actual Track Number (DDI 509)
    std::uint16_t elem509 = state.get_element_number_for_ddi(isobus::DataDescriptionIndex::ActualTrackNumber);
    if (elem509 != 0) {
        send_set_value(client, static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualTrackNumber), elem509, static_cast<std::int32_t>(currentTrack));
        std::cout << "Sent Actual Track Number (DDI 509): " << currentTrack << std::endl;
    }
}

void MyTCServer::send_section_setpoint_states(std::shared_ptr<isobus::ControlFunction> client, std::uint8_t ddiOffset)
{
	std::uint8_t sectionOffset = ddiOffset * NUMBER_SECTIONS_PER_CONDENSED_MESSAGE;
	std::uint32_t value = 0;
	for (std::uint8_t i = 0; i < NUMBER_SECTIONS_PER_CONDENSED_MESSAGE; i++)
	{
		value |= (clients[client].get_section_setpoint_state(sectionOffset + i) << (2 * i));
	}

	std::uint16_t ddiTarget = static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointCondensedWorkState1_16) + ddiOffset;
	std::uint16_t elementNumber = clients[client].get_element_number_for_ddi(static_cast<isobus::DataDescriptionIndex>(ddiTarget));
	send_set_value(client, ddiTarget, elementNumber, value);

	bool setpointWorkState = clients[client].is_any_section_setpoint_on();
	if ((clients[client].get_setpoint_work_state() != setpointWorkState))
	{
		send_set_value(client, static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointWorkState), clients[client].get_element_number_for_ddi(isobus::DataDescriptionIndex::SetpointWorkState), setpointWorkState ? 1 : 0);
		clients[client].set_setpoint_work_state(setpointWorkState);
	}
}

void MyTCServer::send_section_control_state(std::shared_ptr<isobus::ControlFunction> client, bool enabled)
{
	send_set_value(client, static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SectionControlState), clients[client].get_element_number_for_ddi(isobus::DataDescriptionIndex::SectionControlState), enabled ? 1 : 0);
}


void MyTCServer::set_left_tramline_state(bool state)
{
	for (auto &client : clients)
	{
		client.second.set_left_tramline_state(state);
	}
}

void MyTCServer::set_right_tramline_state(bool state)
{
	for (auto &client : clients)
	{
		client.second.set_right_tramline_state(state);
	}
}
