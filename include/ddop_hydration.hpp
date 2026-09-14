/**
 * @brief On-demand "hydrated" DDOP snapshots for debugging and visualization
 *
 * A client's canonical DDOP describes structure only: DeviceProcessData objects carry no value,
 * those arrive later as process data value commands. A hydrated snapshot is a derived copy of the
 * canonical pool in which allow-listed DeviceProcessData objects are replaced by DeviceProperty
 * objects (same object ID, DDI, designator and presentation) holding the latest known value, so
 * external tools such as AgIsoDDOPGenerator can show those values.
 *
 * Snapshots are never uploaded to a TC and never replace or modify the canonical pool.
 */

#pragma once

#include "isobus/isobus/can_NAME.hpp"
#include "isobus/isobus/isobus_device_descriptor_object_pool.hpp"

#include <cstdint>
#include <filesystem>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace ddop_hydration
{
	/// @brief Which objects are eligible for hydration. Configurable through settings.json ("ddopHydration").
	struct AllowList
	{
		bool includeDeviceProperties = true; ///< Tier one: DeviceProperty objects (value already in the pool)
		std::set<std::uint16_t> processDataDDIs; ///< Tier two: on-change DeviceProcessData DDIs (geometry/configuration-like)
		std::uint32_t requestWaitSeconds = 7; ///< How long to wait for responses to on-demand value requests

		static constexpr std::uint32_t MIN_REQUEST_WAIT_SECONDS = 1;
		static constexpr std::uint32_t MAX_REQUEST_WAIT_SECONDS = 60;

		/// @brief Working widths, device element offsets and the reference point to ground distance
		static AllowList defaults();

		/// @brief DDIs that are never hydrated regardless of configuration: totals and transient state
		/// such as work states, section control state and actual rates.
		static bool is_always_excluded(std::uint16_t ddi);

		/// @brief Whether a pool object is eligible for hydration under this allow list
		bool is_eligible(const isobus::task_controller_object::Object &object) const;
	};

	/// @brief The latest process data value reported by a client for one object
	struct ShadowValue
	{
		std::int32_t value = 0;
		std::uint32_t timestamp_ms = 0;
	};

	/// @brief Latest reported process data values per object ID for one client.
	/// Not thread-safe on its own; MyTCServer guards it with clientsMutex.
	class ShadowValueStore
	{
	public:
		void record(std::uint16_t objectID, std::int32_t value);
		bool try_get(std::uint16_t objectID, ShadowValue &shadowValue) const;

	private:
		std::map<std::uint16_t, ShadowValue> values;
	};

	/// @brief Maps the (DDI, element number) addressing of process data messages to DDOP object IDs
	class ProcessDataIndex
	{
	public:
		void build(isobus::DeviceDescriptorObjectPool &pool);
		bool try_get_object_id(std::uint16_t ddi, std::uint16_t elementNumber, std::uint16_t &objectID) const;
		bool try_get_element_number(std::uint16_t objectID, std::uint16_t &elementNumber) const;

	private:
		std::map<std::pair<std::uint16_t, std::uint16_t>, std::uint16_t> ddiAndElementToObjectID;
		std::map<std::uint16_t, std::uint16_t> objectIDToElementNumber;
	};

	/// @brief Where a snapshot object's value came from
	enum class ValueSource : std::uint8_t
	{
		Pool, ///< DeviceProperty, value taken from the canonical pool as-is
		Live, ///< Shadow value that was already known when the snapshot was requested
		Requested, ///< Value arrived after an on-demand value request
		NoResponse, ///< Value was requested but did not arrive in time
		NotRequestable ///< DeviceProcessData that no device element references, so it cannot be addressed
	};

	const char *to_string(ValueSource source);

	struct SnapshotEntry
	{
		static constexpr std::uint16_t NO_ELEMENT = 0xFFFF;

		std::uint16_t objectID = 0;
		std::uint16_t ddi = 0;
		std::uint16_t elementNumber = NO_ELEMENT;
		ValueSource source = ValueSource::NoResponse;
		std::int32_t value = 0; ///< Only meaningful for Pool, Live and Requested
	};

	struct SnapshotInput
	{
		std::vector<std::vector<std::uint8_t>> canonicalPoolChunks; ///< The client's DDOP exactly as uploaded
		std::uint8_t taskControllerCompatibilityLevel = 0;
		std::uint64_t clientName = 0;
		std::string fileStem; ///< Relative to the settings directory, e.g. "<NAME>/<label>"; the canonical pool is "<fileStem>.ddop"
		std::vector<SnapshotEntry> entries;
	};

	struct SnapshotResult
	{
		bool success = false;
		std::string ddopPath;
		std::string metadataPath;
		std::size_t patchedObjects = 0;
		std::size_t missingValues = 0;
		std::string error;
	};

	/// @brief Prefix added to the Device designator of a snapshot
	constexpr const char *SNAPSHOT_DESIGNATOR_PREFIX = "SNAP ";
	/// @brief Marker in snapshot file names: "<label>.SNAP-<timestamp>.iop" next to "<label>.ddop"
	constexpr const char *SNAPSHOT_FILENAME_MARKER = ".SNAP-";

	/// @brief Clones the canonical pool, patches in the entries' values and writes the snapshot plus a JSON sidecar
	SnapshotResult write_snapshot(const SnapshotInput &input);

	/// @brief Whether a file is a hydrated snapshot (or its sidecar) rather than a canonical pool.
	/// Anything that ever loads stored pools must skip these.
	bool is_snapshot_file(const std::filesystem::path &path);
} // namespace ddop_hydration
