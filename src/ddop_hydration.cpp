/**
 * @brief On-demand "hydrated" DDOP snapshots for debugging and visualization
 */
#include "ddop_hydration.hpp"
#include "settings.hpp"

#include "isobus/isobus/isobus_data_dictionary.hpp"
#include "isobus/isobus/isobus_standard_data_description_indices.hpp"
#include "isobus/utility/system_timing.hpp"

#include <ctime>
#include <fstream>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <sstream>

using json = nlohmann::json;

namespace ddop_hydration
{
	namespace
	{
		using isobus::DataDescriptionIndex;
		using isobus::task_controller_object::DeviceProcessDataObject;
		using isobus::task_controller_object::DevicePropertyObject;
		using isobus::task_controller_object::ObjectTypes;

		constexpr std::size_t MAX_DESIGNATOR_LENGTH = 32; // TC version 3 limit; also valid for version 4

		bool ddi_in_range(std::uint16_t ddi, DataDescriptionIndex first, DataDescriptionIndex last)
		{
			return (ddi >= static_cast<std::uint16_t>(first)) && (ddi <= static_cast<std::uint16_t>(last));
		}

		std::string local_timestamp(const char *format)
		{
			const std::time_t now = std::time(nullptr);
			std::tm localTime{};
#if defined(_WIN32)
			localtime_s(&localTime, &now);
#else
			localtime_r(&now, &localTime);
#endif
			std::ostringstream stream;
			stream << std::put_time(&localTime, format);
			return stream.str();
		}

		std::string to_hex(const std::string &bytes)
		{
			std::ostringstream stream;
			for (unsigned char byte : bytes)
			{
				stream << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
			}
			return stream.str();
		}

		bool write_file(const std::string &path, const char *data, std::size_t size, std::ios::openmode mode)
		{
			std::ofstream file(path, mode | std::ios::trunc);
			if (!file.is_open())
			{
				return false;
			}
			file.write(data, static_cast<std::streamsize>(size));
			return file.good();
		}
	} // namespace

	AllowList AllowList::defaults()
	{
		AllowList allowList;
		allowList.processDataDDIs = {
			static_cast<std::uint16_t>(DataDescriptionIndex::ActualWorkingWidth),
			static_cast<std::uint16_t>(DataDescriptionIndex::DefaultWorkingWidth),
			static_cast<std::uint16_t>(DataDescriptionIndex::MaximumWorkingWidth),
			static_cast<std::uint16_t>(DataDescriptionIndex::DeviceElementOffsetX),
			static_cast<std::uint16_t>(DataDescriptionIndex::DeviceElementOffsetY),
			static_cast<std::uint16_t>(DataDescriptionIndex::DeviceElementOffsetZ),
			static_cast<std::uint16_t>(DataDescriptionIndex::DeviceReferencePointDRPtoGroundDistance),
		};
		return allowList;
	}

	bool AllowList::is_always_excluded(std::uint16_t ddi)
	{
		if ((ddi == static_cast<std::uint16_t>(DataDescriptionIndex::ActualWorkState)) ||
		    (ddi == static_cast<std::uint16_t>(DataDescriptionIndex::SetpointWorkState)) ||
		    (ddi == static_cast<std::uint16_t>(DataDescriptionIndex::SectionControlState)) ||
		    (ddi == static_cast<std::uint16_t>(DataDescriptionIndex::PrescriptionControlState)) ||
		    ddi_in_range(ddi, DataDescriptionIndex::ActualCondensedWorkState1_16, DataDescriptionIndex::ActualCondensedWorkState241_256) ||
		    ddi_in_range(ddi, DataDescriptionIndex::SetpointCondensedWorkState1_16, DataDescriptionIndex::SetpointCondensedWorkState241_256) ||
		    ddi_in_range(ddi, DataDescriptionIndex::CondensedSectionOverrideState1_16, DataDescriptionIndex::CondensedSectionOverrideState241_256))
		{
			return true;
		}

		// Totals and actual rates change continuously, so a snapshot of them is meaningless as structure.
		const std::string &name = isobus::DataDictionary::get_entry(ddi).name;
		const bool isTotal = (name.find("Total") != std::string::npos);
		const bool isActualRate = (name.rfind("Actual", 0) == 0) && (name.find("Rate") != std::string::npos);
		return isTotal || isActualRate;
	}

	bool AllowList::is_eligible(const isobus::task_controller_object::Object &object) const
	{
		switch (object.get_object_type())
		{
			case ObjectTypes::DeviceProperty:
			{
				const auto &property = static_cast<const DevicePropertyObject &>(object);
				return includeDeviceProperties && !is_always_excluded(property.get_ddi());
			}

			case ObjectTypes::DeviceProcessData:
			{
				const auto &processData = static_cast<const DeviceProcessDataObject &>(object);
				// has_trigger_method() is not const in AgIsoStack, so test the bitfield directly
				const std::uint8_t triggers = processData.get_trigger_methods_bitfield();
				return (0 == (triggers & static_cast<std::uint8_t>(DeviceProcessDataObject::AvailableTriggerMethods::Total))) &&
				  (0 != (triggers & static_cast<std::uint8_t>(DeviceProcessDataObject::AvailableTriggerMethods::OnChange))) &&
				  !is_always_excluded(processData.get_ddi()) &&
				  (processDataDDIs.count(processData.get_ddi()) > 0);
			}

			default:
				return false;
		}
	}

	void ShadowValueStore::record(std::uint16_t objectID, std::int32_t value)
	{
		values[objectID] = { value, isobus::SystemTiming::get_timestamp_ms() };
	}

	bool ShadowValueStore::try_get(std::uint16_t objectID, ShadowValue &shadowValue) const
	{
		auto it = values.find(objectID);
		if (it == values.end())
		{
			return false;
		}
		shadowValue = it->second;
		return true;
	}

	void ProcessDataIndex::build(isobus::DeviceDescriptorObjectPool &pool)
	{
		ddiAndElementToObjectID.clear();
		objectIDToElementNumber.clear();

		for (std::uint16_t i = 0; i < pool.size(); i++)
		{
			auto object = pool.get_object_by_index(i);
			if (!object || (object->get_object_type() != ObjectTypes::DeviceElement))
			{
				continue;
			}

			auto element = std::static_pointer_cast<isobus::task_controller_object::DeviceElementObject>(object);
			for (std::uint16_t childID : element->get_child_object_ids())
			{
				auto child = pool.get_object_by_id(childID);
				if (!child)
				{
					continue;
				}

				std::uint16_t ddi;
				if (child->get_object_type() == ObjectTypes::DeviceProcessData)
				{
					ddi = std::static_pointer_cast<DeviceProcessDataObject>(child)->get_ddi();
				}
				else if (child->get_object_type() == ObjectTypes::DeviceProperty)
				{
					ddi = std::static_pointer_cast<DevicePropertyObject>(child)->get_ddi();
				}
				else
				{
					continue;
				}

				ddiAndElementToObjectID.emplace(std::make_pair(ddi, element->get_element_number()), childID);
				objectIDToElementNumber.emplace(childID, element->get_element_number());
			}
		}
	}

	bool ProcessDataIndex::try_get_object_id(std::uint16_t ddi, std::uint16_t elementNumber, std::uint16_t &objectID) const
	{
		auto it = ddiAndElementToObjectID.find(std::make_pair(ddi, elementNumber));
		if (it == ddiAndElementToObjectID.end())
		{
			return false;
		}
		objectID = it->second;
		return true;
	}

	bool ProcessDataIndex::try_get_element_number(std::uint16_t objectID, std::uint16_t &elementNumber) const
	{
		auto it = objectIDToElementNumber.find(objectID);
		if (it == objectIDToElementNumber.end())
		{
			return false;
		}
		elementNumber = it->second;
		return true;
	}

	const char *to_string(ValueSource source)
	{
		switch (source)
		{
			case ValueSource::Pool:
				return "pool";
			case ValueSource::Live:
				return "live";
			case ValueSource::Requested:
				return "requested";
			case ValueSource::NoResponse:
				return "no_response";
			case ValueSource::NotRequestable:
				return "not_requestable";
		}
		return "unknown";
	}

	SnapshotResult write_snapshot(const SnapshotInput &input)
	{
		SnapshotResult result;

		// Deserialize a fresh copy: pools copied through ClientState share their objects with the canonical pool.
		isobus::DeviceDescriptorObjectPool pool;
		pool.set_task_controller_compatibility_level(input.taskControllerCompatibilityLevel);
		for (const auto &chunk : input.canonicalPoolChunks)
		{
			if (!pool.deserialize_binary_object_pool(chunk.data(), static_cast<std::uint32_t>(chunk.size()), isobus::NAME(input.clientName)))
			{
				result.error = "failed to deserialize canonical pool";
				return result;
			}
		}

		std::shared_ptr<isobus::task_controller_object::DeviceObject> device;
		for (std::uint16_t i = 0; (i < pool.size()) && !device; i++)
		{
			auto object = pool.get_object_by_index(i);
			if (object && (object->get_object_type() == ObjectTypes::Device))
			{
				device = std::static_pointer_cast<isobus::task_controller_object::DeviceObject>(object);
			}
		}
		if (!device)
		{
			result.error = "pool has no Device object";
			return result;
		}

		// Mark the snapshot so neither people nor a stored-pool lookup mistake it for the canonical upload.
		const std::string canonicalDesignator = device->get_designator();
		const std::string canonicalStructureLabel = device->get_structure_label();
		device->set_designator((SNAPSHOT_DESIGNATOR_PREFIX + canonicalDesignator).substr(0, MAX_DESIGNATOR_LENGTH));
		const std::string snapshotStructureLabel = "SNAP" + canonicalStructureLabel.substr(0, 3);
		device->set_structure_label(snapshotStructureLabel);

		json objects = json::array();
		for (const auto &entry : input.entries)
		{
			const bool hasValue = (entry.source == ValueSource::Pool) || (entry.source == ValueSource::Live) || (entry.source == ValueSource::Requested);
			bool patched = false;

			if (hasValue && (entry.source != ValueSource::Pool))
			{
				auto object = pool.get_object_by_id(entry.objectID);
				if (object && (object->get_object_type() == ObjectTypes::DeviceProcessData))
				{
					auto processData = std::static_pointer_cast<DeviceProcessDataObject>(object);
					const std::string designator = processData->get_designator();
					const std::uint16_t presentationID = processData->get_device_value_presentation_object_id();

					// The parent element keeps referencing the same object ID, so the hierarchy stays intact.
					pool.remove_object_by_id(entry.objectID);
					patched = pool.add_device_property(designator, entry.value, entry.ddi, presentationID, entry.objectID);
					if (!patched)
					{
						result.error = "failed to replace object " + std::to_string(entry.objectID);
						return result;
					}
					result.patchedObjects++;
				}
			}
			else if (!hasValue)
			{
				result.missingValues++;
			}

			json item = {
				{ "objectId", entry.objectID },
				{ "ddi", entry.ddi },
				{ "ddiName", isobus::DataDictionary::get_entry(entry.ddi).name },
				{ "elementNumber", (entry.elementNumber == SnapshotEntry::NO_ELEMENT) ? json(nullptr) : json(entry.elementNumber) },
				{ "source", to_string(entry.source) },
				{ "value", hasValue ? json(entry.value) : json(nullptr) },
				{ "patchedToDeviceProperty", patched },
			};
			objects.push_back(item);
		}

		std::vector<std::uint8_t> binaryPool;
		if (!pool.generate_binary_object_pool(binaryPool))
		{
			result.error = "failed to generate snapshot pool";
			return result;
		}

		const std::string createdAt = local_timestamp("%Y-%m-%dT%H:%M:%S");
		const std::string snapshotStem = input.fileStem + SNAPSHOT_FILENAME_MARKER + local_timestamp("%Y%m%d-%H%M%S");
		result.ddopPath = Settings::get_filename_path(snapshotStem + ".iop");
		result.metadataPath = Settings::get_filename_path(snapshotStem + ".json");

		std::ostringstream clientName;
		clientName << "0x" << std::hex << std::setw(16) << std::setfill('0') << input.clientName;

		json metadata = {
			{ "kind", "AOG-TaskController hydrated DDOP snapshot" },
			{ "notForReupload", true },
			{ "note",
			  "Derived debugging artifact, not an original upload. Allow-listed DeviceProcessData objects with a known value "
			  "were replaced by DeviceProperty objects with the same object ID, DDI, designator and presentation. "
			  "The Device designator is prefixed with 'SNAP ' and the structure label is altered. Never upload this pool to a TC." },
			{ "createdAt", createdAt },
			{ "clientName", clientName.str() },
			{ "canonicalPool", input.fileStem + ".ddop" },
			{ "canonicalDesignator", canonicalDesignator },
			{ "canonicalStructureLabelHex", to_hex(canonicalStructureLabel) },
			{ "snapshotStructureLabelHex", to_hex(snapshotStructureLabel) },
			{ "taskControllerCompatibilityLevel", input.taskControllerCompatibilityLevel },
			{ "patchedObjects", result.patchedObjects },
			{ "missingValues", result.missingValues },
			{ "objects", objects },
		};
		// Designators come from the implement and are not guaranteed to be valid UTF-8.
		const std::string metadataText = metadata.dump(4, ' ', false, json::error_handler_t::replace);

		if (!write_file(result.ddopPath, reinterpret_cast<const char *>(binaryPool.data()), binaryPool.size(), std::ios::binary))
		{
			result.error = "unable to write " + result.ddopPath;
			return result;
		}
		if (!write_file(result.metadataPath, metadataText.data(), metadataText.size(), std::ios::out))
		{
			result.error = "unable to write " + result.metadataPath;
			return result;
		}

		result.success = true;
		return result;
	}

	bool is_snapshot_file(const std::filesystem::path &path)
	{
		return path.filename().string().find(SNAPSHOT_FILENAME_MARKER) != std::string::npos;
	}
} // namespace ddop_hydration
