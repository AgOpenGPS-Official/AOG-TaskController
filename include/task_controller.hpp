/**
 * @author Daan Steenbergen
 * @brief An ISOBUS Task Controller for AgOpenGPS
 * @version 0.1
 * @date 2025-1-20
 *
 * @copyright 2025 Daan Steenbergen
 */

#pragma once

#include "isobus/isobus/isobus_data_dictionary.hpp"
#include "isobus/isobus/isobus_device_descriptor_object_pool.hpp"
#include "isobus/isobus/isobus_standard_data_description_indices.hpp"
#include "isobus/isobus/isobus_task_controller_server.hpp"

#include <cstdint>
#include <map>
#include <queue>

constexpr std::uint8_t NUMBER_SECTIONS_PER_CONDENSED_MESSAGE = 16;

enum SectionState : std::uint8_t
{
	OFF = 0, ///< Section is off
	ON = 1, ///< Section is on
	ERROR_SATE = 2, ///< Section is in an error state
	NOT_INSTALLED = 3 ///< Section is not installed
};

class ClientState
{
public:
	void set_number_of_sections(std::uint8_t number);
	void set_section_setpoint_state(std::uint8_t section, std::uint8_t state);
	void set_section_actual_state(std::uint8_t section, std::uint8_t state);
	std::uint8_t get_number_of_sections() const;
	std::uint8_t get_section_setpoint_state(std::uint8_t section) const;
	std::uint8_t get_section_actual_state(std::uint8_t section) const;
	bool is_any_section_setpoint_on() const;
	bool get_setpoint_work_state() const;
	void set_setpoint_work_state(bool state);
	bool get_actual_work_state() const;
	void set_actual_work_state(bool state);
	bool is_section_control_enabled() const;
	void set_section_control_enabled(bool state);
	isobus::DeviceDescriptorObjectPool &get_pool();
	bool are_measurement_commands_sent() const;
	void mark_measurement_commands_sent();
	std::uint16_t get_element_number_for_ddi(isobus::DataDescriptionIndex ddi) const;
	void set_element_number_for_ddi(isobus::DataDescriptionIndex ddi, std::uint16_t elementNumber);
	// Element work state management these act like master / override for actual sections
	void set_element_work_state(std::uint16_t elementNumber, bool isWorking);
	bool get_element_work_state(std::uint16_t elementNumber, bool &isWorking) const;

private:
	isobus::DeviceDescriptorObjectPool pool; ///< The device descriptor object pool (DDOP) for the TC
	bool areMeasurementCommandsSent = false; ///< Whether or not the measurement commands have been sent
	std::map<isobus::DataDescriptionIndex, std::uint16_t> ddiToElementNumber; ///< Mapping of DDI to element number // TODO: better way to do this?

	std::uint8_t numberOfSections;
	std::vector<std::uint8_t> sectionSetpointStates; // 2 bits per section (0 = off, 1 = on, 2 = error, 3 = not installed)
	std::vector<std::uint8_t> sectionActualStates; // 2 bits per section (0 = off, 1 = on, 2 = error, 3 = not installed)
	bool setpointWorkState = false; ///< The overall work state desired
	bool actualWorkState = false; ///< The overall work state actual
	std::map<std::uint16_t, bool> elementWorkStates; ///< Work state per element (element number -> is working)
	bool isSectionControlEnabled = false; ///< Stores auto vs manual mode setting
};

// Create the task controller server object, this will handle all the ISOBUS communication for us
class MyTCServer : public isobus::TaskControllerServer
{
public:
	MyTCServer(std::shared_ptr<isobus::InternalControlFunction> internalControlFunction);
	bool activate_object_pool(std::shared_ptr<isobus::ControlFunction> partnerCF, ObjectPoolActivationError &, ObjectPoolErrorCodes &, std::uint16_t &, std::uint16_t &) override;
	bool change_designator(std::shared_ptr<isobus::ControlFunction>, std::uint16_t, const std::vector<std::uint8_t> &) override;
	bool deactivate_object_pool(std::shared_ptr<isobus::ControlFunction> partnerCF) override;
	bool delete_device_descriptor_object_pool(std::shared_ptr<isobus::ControlFunction> partnerCF, ObjectPoolDeletionErrors &) override;
	bool get_is_stored_device_descriptor_object_pool_by_structure_label(std::shared_ptr<isobus::ControlFunction>, const std::vector<std::uint8_t> &, const std::vector<std::uint8_t> &) override;
	bool get_is_stored_device_descriptor_object_pool_by_localization_label(std::shared_ptr<isobus::ControlFunction>, const std::array<std::uint8_t, 7> &) override;
	bool get_is_enough_memory_available(std::uint32_t) override;
	void identify_task_controller(std::uint8_t) override;
	void on_client_timeout(std::shared_ptr<isobus::ControlFunction> partner) override;
	void on_process_data_acknowledge(std::shared_ptr<isobus::ControlFunction> partner, std::uint16_t dataDescriptionIndex, std::uint16_t elementNumber, std::uint8_t errorCodesFromClient, ProcessDataCommands processDataCommand) override;
	bool on_value_command(std::shared_ptr<isobus::ControlFunction> partner,
	                      std::uint16_t dataDescriptionIndex,
	                      std::uint16_t elementNumber,
	                      std::int32_t processDataValue,
	                      std::uint8_t &errorCodes) override;
	bool store_device_descriptor_object_pool(std::shared_ptr<isobus::ControlFunction> partnerCF, const std::vector<std::uint8_t> &binaryPool, bool appendToPool) override;
	std::map<std::shared_ptr<isobus::ControlFunction>, ClientState> &get_clients();
	void request_measurement_commands();
	void update_section_states(std::vector<bool> &sectionStates);
	void update_section_control_enabled(bool enabled);

private:
	void send_section_setpoint_states(std::shared_ptr<isobus::ControlFunction> client, std::uint8_t ddiOffset);
	void send_section_control_state(std::shared_ptr<isobus::ControlFunction> client, bool enabled);

	std::map<std::shared_ptr<isobus::ControlFunction>, ClientState> clients;
	std::map<std::shared_ptr<isobus::ControlFunction>, std::queue<std::vector<std::uint8_t>>> uploadedPools;
};
