// Regression test for the stale-upload bug fixed in MyTCServer::store_device_descriptor_object_pool()
// and MyTCServer::activate_object_pool() (see src/task_controller.cpp).
//
// isobus::DeviceDescriptorObjectPool::deserialize_binary_object_pool() never clears its own object
// list before parsing — every object it adds calls remove_object_with_id() first, so re-parsing the
// SAME pool on top of itself is harmless (each object is cleanly replaced). But a client's pool
// upload queue used to accumulate across a timeout-then-reconnect or an aborted upload followed by a restart
// without ever being cleared, and activate_object_pool() looped over every queued chunk and called
// deserialize_binary_object_pool() once per chunk, into the SAME pool object. If the queue ever held
// an earlier, unrelated (or truncated) upload alongside the real one, whatever that earlier upload
// declared under object IDs the real pool never reuses survives the second deserialize call
// uncleaned, because nothing removes an object that the newer pool simply doesn't mention.
//
// This exercises that exact mechanism directly against isobus::DeviceDescriptorObjectPool — the same
// class task_controller.cpp deserializes into — independent of the CAN/TaskControllerServer
// scaffolding MyTCServer needs to actually run.
#include "isobus/isobus/isobus_device_descriptor_object_pool.hpp"

#include <algorithm>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

namespace
{
	using Pool = isobus::DeviceDescriptorObjectPool;
	using Element = isobus::task_controller_object::DeviceElementObject;

	std::vector<std::string> violations;

	void flag(const std::string &reason)
	{
		violations.push_back(reason);
	}

	// A minimal version of the field report's "E2 Seeder" DDOP: a boom (element 2) carrying Section
	// Control State (DDI 160) and a working-width DDI (67), and a section (element 4) carrying its
	// own copy of DDI 67 plus DDI 134 and 135. DDI 160 exists only on the boom.
	bool build_seeder_pool(Pool &pool)
	{
		bool ok = true;
		ok = ok && pool.add_device("E2 Seeder", "1", "SN1", "E2SD", {}, {}, 0x1234);
		ok = ok && pool.add_device_element("Device", 1, 0, Element::Type::Device, 1);
		ok = ok && pool.add_device_element("Boom", 2, 1, Element::Type::Function, 2);
		ok = ok && pool.add_device_element("Section", 4, 2, Element::Type::Section, 4);
		ok = ok && pool.add_device_process_data("Section Control State", 160, 0xFFFF, 0, 0, 160);
		ok = ok && pool.add_device_process_data("Working width (boom)", 67, 0xFFFF, 0, 0, 267);
		ok = ok && pool.add_device_process_data("Working width (section)", 67, 0xFFFF, 0, 0, 467);
		ok = ok && pool.add_device_process_data("Rate", 134, 0xFFFF, 0, 0, 434);
		ok = ok && pool.add_device_process_data("Area", 135, 0xFFFF, 0, 0, 435);
		if (!ok)
		{
			return false;
		}

		auto boom = std::dynamic_pointer_cast<Element>(pool.get_object_by_id(2));
		auto section = std::dynamic_pointer_cast<Element>(pool.get_object_by_id(4));
		if (!boom || !section)
		{
			return false;
		}
		boom->add_reference_to_child_object(160);
		boom->add_reference_to_child_object(267);
		section->add_reference_to_child_object(467);
		section->add_reference_to_child_object(434);
		section->add_reference_to_child_object(435);
		return true;
	}

	// A stale, unrelated pool standing in for "an earlier connection attempt that uploaded a pool
	// and then timed out (or was abandoned and restarted) before
	// activation." Different device, different object IDs, on purpose — the point is that its IDs
	// don't collide with the real pool's, so nothing about a normal remove-then-readd would ever
	// touch it.
	bool build_stale_pool(Pool &pool)
	{
		bool ok = true;
		ok = ok && pool.add_device("Stale Planter", "1", "SN2", "OLD1", {}, {}, 0x9999);
		ok = ok && pool.add_device_element("Device", 1, 0, Element::Type::Device, 900);
		ok = ok && pool.add_device_element("Row unit", 9, 900, Element::Type::Section, 901);
		ok = ok && pool.add_device_process_data("Stale DDI", 999, 0xFFFF, 0, 0, 902);
		if (!ok)
		{
			return false;
		}

		auto rowUnit = std::dynamic_pointer_cast<Element>(pool.get_object_by_id(901));
		if (!rowUnit)
		{
			return false;
		}
		rowUnit->add_reference_to_child_object(902);
		return true;
	}

	// The element numbers whose child list references the object with the given ID — the same
	// "which DeviceElement owns this object" search MyTCServer::request_measurement_commands() does
	// when it maps a DDI's DeviceProcessData object to an element number.
	std::vector<std::uint16_t> find_owning_elements(Pool &pool, std::uint16_t childObjectID)
	{
		std::vector<std::uint16_t> owners;
		for (std::uint32_t i = 0; i < pool.size(); ++i)
		{
			auto element = std::dynamic_pointer_cast<Element>(pool.get_object_by_index(static_cast<std::uint16_t>(i)));
			if (!element)
			{
				continue;
			}
			for (auto id : element->get_child_object_ids())
			{
				if (id == childObjectID)
				{
					owners.push_back(element->get_element_number());
					break;
				}
			}
		}
		return owners;
	}

	std::string join(const std::vector<std::uint16_t> &values)
	{
		std::string result;
		for (auto value : values)
		{
			result += std::to_string(value) + " ";
		}
		return result.empty() ? "(none)" : result;
	}
}

int main()
{
	std::vector<std::uint8_t> staleBytes;
	std::vector<std::uint8_t> seederBytes;
	{
		Pool stale;
		Pool seeder;
		if (!build_stale_pool(stale) || !stale.generate_binary_object_pool(staleBytes))
		{
			flag("failed to build/serialize the stale synthetic pool");
		}
		if (!build_seeder_pool(seeder) || !seeder.generate_binary_object_pool(seederBytes))
		{
			flag("failed to build/serialize the seeder synthetic pool");
		}
	}

	// --- Reproduces the pre-fix bug: deserializing a stale pool and then the real pool into the
	// same pool object, exactly like activate_object_pool()'s old per-chunk loop did whenever more
	// than one blob had been queued for a client. ---
	if (violations.empty())
	{
		Pool combined;
		combined.deserialize_binary_object_pool(staleBytes.data(), static_cast<std::uint32_t>(staleBytes.size()), isobus::NAME(0));
		combined.deserialize_binary_object_pool(seederBytes.data(), static_cast<std::uint32_t>(seederBytes.size()), isobus::NAME(0));

		if (nullptr == combined.get_object_by_id(901))
		{
			flag("pre-fix repro: expected the stale pool's row unit (object 901) to survive "
			     "alongside the real pool — it's gone, so either deserialize_binary_object_pool() "
			     "now resets its object list per call (this repro no longer applies) or something "
			     "else changed. Re-check this test against the current isobus fork.");
		}
	}

	// --- The fix: the server drops unactivated chunks at the start of each client session (version
	// exchange / label queries) and on timeout, and activate_object_pool() concatenates whatever chunks remain
	// into one buffer and deserializes exactly once. With no stale pool queued alongside it, only
	// the real pool's bytes are ever handed to the deserializer. ---
	if (violations.empty())
	{
		Pool activated;
		if (!activated.deserialize_binary_object_pool(seederBytes.data(), static_cast<std::uint32_t>(seederBytes.size()), isobus::NAME(0)))
		{
			flag("fixed-case deserialize of the real pool failed");
		}

		if (nullptr != activated.get_object_by_id(901))
		{
			flag("fixed case: the stale pool's row unit (object 901) leaked into the activated pool");
		}

		auto sectionControlOwners = find_owning_elements(activated, 160);
		if ((1 != sectionControlOwners.size()) || (2 != sectionControlOwners[0]))
		{
			flag("DDI 160 (Section Control State) should map only to element 2, got: " + join(sectionControlOwners));
		}

		auto workingWidthOwners = find_owning_elements(activated, 267);
		auto sectionWidthOwners = find_owning_elements(activated, 467);
		if ((1 != workingWidthOwners.size()) || (2 != workingWidthOwners[0]))
		{
			flag("DDI 67 on the boom should map only to element 2, got: " + join(workingWidthOwners));
		}
		if ((1 != sectionWidthOwners.size()) || (4 != sectionWidthOwners[0]))
		{
			flag("DDI 67 on the section should map only to element 4, got: " + join(sectionWidthOwners));
		}
	}

	// --- Chunked upload: a client may send its DDOP as several Request/Transfer pairs, and the
	// chunk boundaries need not fall on object boundaries (a real implement sent 72/241/2045/2888
	// byte chunks). The server queues every chunk and activate_object_pool() concatenates them
	// before parsing once, so the result must match a single-shot parse of the same bytes, for
	// any split point. ---
	if (violations.empty())
	{
		Pool whole;
		whole.deserialize_binary_object_pool(seederBytes.data(), static_cast<std::uint32_t>(seederBytes.size()), isobus::NAME(0));

		for (std::size_t chunkSize : { std::size_t(1), std::size_t(7), std::size_t(72), std::size_t(241) })
		{
			std::vector<std::vector<std::uint8_t>> chunks;
			for (std::size_t offset = 0; offset < seederBytes.size(); offset += chunkSize)
			{
				const auto end = std::min(seederBytes.size(), offset + chunkSize);
				chunks.emplace_back(seederBytes.begin() + offset, seederBytes.begin() + end);
			}

			std::vector<std::uint8_t> combinedBytes;
			for (const auto &chunk : chunks)
			{
				combinedBytes.insert(combinedBytes.end(), chunk.begin(), chunk.end());
			}

			Pool chunked;
			if (!chunked.deserialize_binary_object_pool(combinedBytes.data(), static_cast<std::uint32_t>(combinedBytes.size()), isobus::NAME(0)))
			{
				flag("chunked upload (chunk size " + std::to_string(chunkSize) + "): concatenated pool failed to deserialize");
			}
			else if (chunked.size() != whole.size())
			{
				flag("chunked upload (chunk size " + std::to_string(chunkSize) + "): got " + std::to_string(chunked.size()) +
				     " objects, a single-shot parse gives " + std::to_string(whole.size()));
			}
		}
	}

	if (!violations.empty())
	{
		std::fprintf(stderr, "FAIL: ddop_reupload_test\n");
		for (const auto &violation : violations)
		{
			std::fprintf(stderr, "  %s\n", violation.c_str());
		}
		return 1;
	}

	std::printf("OK: ddop_reupload_test\n");
	return 0;
}
