// Offline test for hydrated DDOP snapshots: builds a small DDOP, hydrates it and reads the result back.
#include "ddop_hydration.hpp"

#include "isobus/isobus/can_constants.hpp"
#include "isobus/isobus/isobus_standard_data_description_indices.hpp"
#include "isobus/utility/iop_file_interface.hpp"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <string>

namespace
{
	using isobus::task_controller_object::DeviceElementObject;
	using isobus::task_controller_object::DeviceProcessDataObject;
	using isobus::task_controller_object::DevicePropertyObject;
	using isobus::task_controller_object::ObjectTypes;
	using Trigger = DeviceProcessDataObject::AvailableTriggerMethods;

	int failures = 0;

	void check(bool condition, const char *description)
	{
		if (!condition)
		{
			std::printf("FAIL: %s\n", description);
			failures++;
		}
	}

	constexpr std::uint16_t OFFSET_X_ID = 10; // on-change, live value known
	constexpr std::uint16_t WIDTH_TOTAL_ID = 11; // on-change but flagged as a total
	constexpr std::uint16_t WORK_STATE_ID = 12; // always excluded
	constexpr std::uint16_t OFFSET_Y_PROPERTY_ID = 13; // device property
	constexpr std::uint16_t OFFSET_Z_ID = 14; // on-change, requested but never answered
	constexpr std::uint16_t MAX_WIDTH_INTERVAL_ID = 15; // no on-change trigger
	constexpr std::uint16_t SETPOINT_ID = 16; // on-change setpoint
	constexpr std::uint16_t SECTION_ELEMENT_ID = 2;
	constexpr std::uint16_t SECTION_ELEMENT_NUMBER = 1;

	std::uint8_t triggers(Trigger trigger)
	{
		return static_cast<std::uint8_t>(trigger);
	}

	isobus::DeviceDescriptorObjectPool build_canonical_pool()
	{
		using isobus::DataDescriptionIndex;
		isobus::DeviceDescriptorObjectPool pool(3);
		pool.add_device("Test sprayer", "1.0", "123", "ABCDEFG", { 'e', 'n', 0x50, 0x00, 0x00, 0x00, 0xFF }, {}, 0);
		pool.add_device_element("Sprayer", 0, 0, DeviceElementObject::Type::Device, 1);
		pool.add_device_element("Section 1", SECTION_ELEMENT_NUMBER, 1, DeviceElementObject::Type::Section, SECTION_ELEMENT_ID);

		pool.add_device_process_data("Offset X", static_cast<std::uint16_t>(DataDescriptionIndex::DeviceElementOffsetX), isobus::NULL_OBJECT_ID, 0, triggers(Trigger::OnChange), OFFSET_X_ID);
		pool.add_device_process_data("Width", static_cast<std::uint16_t>(DataDescriptionIndex::ActualWorkingWidth), isobus::NULL_OBJECT_ID, 0, triggers(Trigger::OnChange) | triggers(Trigger::Total), WIDTH_TOTAL_ID);
		pool.add_device_process_data("Work state", static_cast<std::uint16_t>(DataDescriptionIndex::ActualWorkState), isobus::NULL_OBJECT_ID, 0, triggers(Trigger::OnChange), WORK_STATE_ID);
		pool.add_device_property("Offset Y", 500, static_cast<std::uint16_t>(DataDescriptionIndex::DeviceElementOffsetY), isobus::NULL_OBJECT_ID, OFFSET_Y_PROPERTY_ID);
		pool.add_device_process_data("Offset Z", static_cast<std::uint16_t>(DataDescriptionIndex::DeviceElementOffsetZ), isobus::NULL_OBJECT_ID, 0, triggers(Trigger::OnChange), OFFSET_Z_ID);
		pool.add_device_process_data("Max width", static_cast<std::uint16_t>(DataDescriptionIndex::MaximumWorkingWidth), isobus::NULL_OBJECT_ID, 0, triggers(Trigger::TimeInterval), MAX_WIDTH_INTERVAL_ID);
		pool.add_device_process_data("Setpoint width", static_cast<std::uint16_t>(DataDescriptionIndex::SetpointWorkingWidth), isobus::NULL_OBJECT_ID, 2, triggers(Trigger::OnChange), SETPOINT_ID);

		auto section = std::static_pointer_cast<DeviceElementObject>(pool.get_object_by_id(SECTION_ELEMENT_ID));
		for (std::uint16_t child : { OFFSET_X_ID, WIDTH_TOTAL_ID, WORK_STATE_ID, OFFSET_Y_PROPERTY_ID, OFFSET_Z_ID, MAX_WIDTH_INTERVAL_ID, SETPOINT_ID })
		{
			section->add_reference_to_child_object(child);
		}
		return pool;
	}
} // namespace

int main()
{
	auto canonicalPool = build_canonical_pool();
	std::vector<std::uint8_t> canonicalBinary;
	check(canonicalPool.generate_binary_object_pool(canonicalBinary), "canonical test pool generates");

	// Automatic eligibility
	check(ddop_hydration::is_hydratable(*canonicalPool.get_object_by_id(OFFSET_X_ID)), "on-change offset is hydratable");
	check(!ddop_hydration::is_hydratable(*canonicalPool.get_object_by_id(WIDTH_TOTAL_ID)), "total is not hydratable");
	check(!ddop_hydration::is_hydratable(*canonicalPool.get_object_by_id(WORK_STATE_ID)), "work state is not hydratable");
	check(!ddop_hydration::is_hydratable(*canonicalPool.get_object_by_id(SETPOINT_ID)), "setpoint is not hydratable");
	check(ddop_hydration::is_hydratable(*canonicalPool.get_object_by_id(OFFSET_Y_PROPERTY_ID)), "device property is hydratable");
	check(!ddop_hydration::is_hydratable(*canonicalPool.get_object_by_id(MAX_WIDTH_INTERVAL_ID)), "process data without on-change is not hydratable");

	// Index
	ddop_hydration::ProcessDataIndex index;
	index.build(canonicalPool);
	std::uint16_t objectID = 0;
	check(index.try_get_object_id(static_cast<std::uint16_t>(isobus::DataDescriptionIndex::DeviceElementOffsetX), SECTION_ELEMENT_NUMBER, objectID) && (objectID == OFFSET_X_ID),
	      "index maps (DDI, element) to object ID");

	// Snapshot
	ddop_hydration::SnapshotInput input;
	input.canonicalPoolChunks = { canonicalBinary };
	input.taskControllerCompatibilityLevel = 3;
	input.fileStem = "ddop-hydration-test/test-label";
	input.entries = {
		{ OFFSET_X_ID, static_cast<std::uint16_t>(isobus::DataDescriptionIndex::DeviceElementOffsetX), SECTION_ELEMENT_NUMBER, ddop_hydration::ValueSource::Live, 1234 },
		{ OFFSET_Y_PROPERTY_ID, static_cast<std::uint16_t>(isobus::DataDescriptionIndex::DeviceElementOffsetY), SECTION_ELEMENT_NUMBER, ddop_hydration::ValueSource::Pool, 500 },
		{ OFFSET_Z_ID, static_cast<std::uint16_t>(isobus::DataDescriptionIndex::DeviceElementOffsetZ), SECTION_ELEMENT_NUMBER, ddop_hydration::ValueSource::NoResponse, 0 },
	};

	const auto result = ddop_hydration::write_snapshot(input);
	check(result.success, "snapshot is written");
	if (!result.success)
	{
		std::printf("snapshot error: %s\n", result.error.c_str());
		return 1;
	}
	check(result.patchedObjects == 1, "one object patched");
	check(result.missingValues == 1, "one value missing");
	check(ddop_hydration::is_snapshot_file(result.ddopPath), "snapshot file is recognized as a snapshot");
	check(!ddop_hydration::is_snapshot_file("123/test-label.ddop"), "canonical file is not recognized as a snapshot");

	auto snapshotBinary = isobus::IOPFileInterface::read_iop_file(result.ddopPath);
	isobus::DeviceDescriptorObjectPool snapshotPool(3);
	check(snapshotPool.deserialize_binary_object_pool(snapshotBinary), "snapshot deserializes");

	auto patched = snapshotPool.get_object_by_id(OFFSET_X_ID);
	check(patched && (patched->get_object_type() == ObjectTypes::DeviceProperty), "live process data became a device property");
	if (patched && (patched->get_object_type() == ObjectTypes::DeviceProperty))
	{
		auto property = std::static_pointer_cast<DevicePropertyObject>(patched);
		check(property->get_value() == 1234, "patched value");
		check(property->get_ddi() == static_cast<std::uint16_t>(isobus::DataDescriptionIndex::DeviceElementOffsetX), "patched DDI");
		check(property->get_designator() == "Offset X", "patched designator");
	}
	auto missing = snapshotPool.get_object_by_id(OFFSET_Z_ID);
	check(missing && (missing->get_object_type() == ObjectTypes::DeviceProcessData), "unanswered process data stays process data");

	auto section = std::dynamic_pointer_cast<DeviceElementObject>(snapshotPool.get_object_by_id(SECTION_ELEMENT_ID));
	bool stillReferenced = false;
	if (section)
	{
		for (std::uint16_t child : section->get_child_object_ids())
		{
			stillReferenced = stillReferenced || (child == OFFSET_X_ID);
		}
	}
	check(stillReferenced, "section still references the patched object");

	std::shared_ptr<isobus::task_controller_object::DeviceObject> device;
	for (std::uint16_t i = 0; (i < snapshotPool.size()) && !device; i++)
	{
		auto object = snapshotPool.get_object_by_index(i);
		if (object && (object->get_object_type() == ObjectTypes::Device))
		{
			device = std::static_pointer_cast<isobus::task_controller_object::DeviceObject>(object);
		}
	}
	check(device && (device->get_designator().rfind("SNAP ", 0) == 0), "device designator is prefixed");
	check(device && (device->get_structure_label().rfind("SNAP", 0) == 0), "structure label is altered");

	auto original = canonicalPool.get_object_by_id(OFFSET_X_ID);
	check(original && (original->get_object_type() == ObjectTypes::DeviceProcessData), "canonical pool is untouched");

	std::ifstream metadataFile(result.metadataPath);
	const auto metadata = nlohmann::json::parse(metadataFile, nullptr, false);
	check(!metadata.is_discarded() && metadata.value("notForReupload", false), "sidecar marks the snapshot as not for re-upload");
	check(!metadata.is_discarded() && (metadata["objects"].size() == 3), "sidecar lists every entry");
	metadataFile.close();

	std::error_code cleanupError;
	std::filesystem::remove_all(std::filesystem::path(result.ddopPath).parent_path(), cleanupError);

	std::printf(failures == 0 ? "All ddop hydration checks passed\n" : "%d ddop hydration check(s) failed\n", failures);
	return (failures == 0) ? 0 : 1;
}
