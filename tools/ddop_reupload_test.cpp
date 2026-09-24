// Regression test for stale DDOP re-upload (see MyTCServer::activate_object_pool()).
// deserialize_binary_object_pool() never clears the pool first, so parsing a stale queued upload and
// then the real one into the same object leaves the stale pool's non-overlapping objects behind.
// Runs directly against isobus::DeviceDescriptorObjectPool, without the CAN/server scaffolding.
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

	// Minimal "E2 Seeder" DDOP from the field report: boom (element 2) with DDI 160 and 67,
	// section (element 4) with DDI 67, 134 and 135
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

	// An unactivated upload from an earlier, aborted session. Its object IDs deliberately don't
	// collide with the real pool's, so re-parsing the real pool never replaces them.
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

	// Element numbers referencing the given child object, like MyTCServer::request_measurement_commands()
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

	// Pre-fix bug: stale pool then real pool deserialized into the same object
	if (violations.empty())
	{
		Pool combined;
		combined.deserialize_binary_object_pool(staleBytes.data(), static_cast<std::uint32_t>(staleBytes.size()), isobus::NAME(0));
		combined.deserialize_binary_object_pool(seederBytes.data(), static_cast<std::uint32_t>(seederBytes.size()), isobus::NAME(0));

		if (nullptr == combined.get_object_by_id(901))
		{
			flag("pre-fix repro: stale object 901 no longer survives a second deserialize; "
			     "re-check this test against the current isobus library");
		}
	}

	// Fixed: stale chunks are dropped at session start and on timeout, so only the real pool is parsed
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

	// Chunked upload: chunks split mid-object (a real implement sent 72/241/2045/2888 bytes), so the
	// concatenated parse must match a single-shot parse for any split point
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
