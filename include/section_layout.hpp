/**
 * @brief How an implement's sections map onto condensed-DDI owner elements and onto AOG's sections
 *
 * Pure functions, so they can be tested against sample DDOPs without a bus.
 */

#pragma once

#include "element_hierarchy.hpp"
#include "isobus/isobus/isobus_device_descriptor_object_pool.hpp"

#include <cstdint>
#include <cstdlib>
#include <map>
#include <optional>
#include <vector>

namespace section_layout
{
	/// @brief An element that carries condensed section DDIs (e.g. DDI 290 / 161) for a run of sections.
	/// @details ISO 11783-10 numbers the sections of a condensed DDI within the element that carries it.
	/// An implement with several booms (e.g. one Function per product) has one owner per boom, each with
	/// its own section 1.
	struct CondensedOwner
	{
		std::uint16_t elementNumber = 0; ///< Element carrying the condensed DDIs
		std::uint8_t firstSection = 0; ///< Index of its first section in the TC's flat section list
		std::uint8_t count = 0; ///< Number of consecutive sections it owns
	};

	/// @brief Whether an element links a process data object with a DDI in [firstDdi, lastDdi]
	/// @param requireSettable Only count process data the TC may set (needed when the range is DDI 161+, whose
	/// read-only form only reports and cannot be commanded)
	inline bool element_links_ddi(isobus::DeviceDescriptorObjectPool &pool, const isobus::task_controller_object::DeviceElementObject &element, std::uint16_t firstDdi, std::uint16_t lastDdi, bool requireSettable = false)
	{
		using isobus::task_controller_object::DeviceProcessDataObject;
		using isobus::task_controller_object::ObjectTypes;
		for (std::uint16_t childId : element.get_child_object_ids())
		{
			auto child = pool.get_object_by_id(childId);
			if (child && (child->get_object_type() == ObjectTypes::DeviceProcessData))
			{
				const auto processData = std::static_pointer_cast<DeviceProcessDataObject>(child);
				const auto ddi = processData->get_ddi();
				if ((ddi >= firstDdi) && (ddi <= lastDdi) &&
				    (!requireSettable || processData->has_property(DeviceProcessDataObject::PropertiesBit::Settable)))
				{
					return true;
				}
			}
		}
		return false;
	}

	/// @brief Finds, for each section, the nearest element (the section itself or an ancestor) that links a
	/// DDI in [firstDdi, lastDdi], and groups consecutive sections with the same owner.
	/// @param requireSettable Only settable process data makes an owner (see element_links_ddi)
	/// @returns The owners in section order, or an empty vector when a section has no owner or one owner's
	/// sections are not consecutive (callers then keep the single-element behaviour).
	inline std::vector<CondensedOwner> resolve_condensed_owners(isobus::DeviceDescriptorObjectPool &pool,
	                                                            const element_hierarchy::ParentMap &parents,
	                                                            const std::vector<std::uint16_t> &sectionElements,
	                                                            std::uint16_t firstDdi,
	                                                            std::uint16_t lastDdi,
	                                                            bool requireSettable = false)
	{
		using isobus::task_controller_object::DeviceElementObject;
		using isobus::task_controller_object::ObjectTypes;

		std::map<std::uint16_t, std::shared_ptr<DeviceElementObject>> elements;
		for (std::uint16_t i = 0; i < pool.size(); i++)
		{
			auto object = pool.get_object_by_index(i);
			if (object && (object->get_object_type() == ObjectTypes::DeviceElement))
			{
				auto element = std::static_pointer_cast<DeviceElementObject>(object);
				elements[element->get_element_number()] = element;
			}
		}

		std::vector<CondensedOwner> owners;
		for (std::size_t i = 0; i < sectionElements.size(); i++)
		{
			std::optional<std::uint16_t> owner;
			std::uint16_t current = sectionElements[i];
			for (std::size_t depth = 0; depth < 32; depth++) // depth guard against a cyclic pool
			{
				auto found = elements.find(current);
				if ((found != elements.end()) && element_links_ddi(pool, *found->second, firstDdi, lastDdi, requireSettable))
				{
					owner = current;
					break;
				}
				auto parent = parents.find(current);
				if (parent == parents.end())
				{
					break; // Top of the element chain (the device element)
				}
				current = parent->second;
			}

			if (!owner)
			{
				return {};
			}
			if (!owners.empty() && (owners.back().elementNumber == *owner))
			{
				owners.back().count++;
				continue;
			}
			for (const auto &existing : owners)
			{
				if (existing.elementNumber == *owner)
				{
					return {}; // Sections of one owner are not consecutive
				}
			}
			owners.push_back({ *owner, static_cast<std::uint8_t>(i), 1 });
		}
		return owners;
	}

	/// @brief Lateral position of one section, as far as it is known
	struct SectionGeometry
	{
		std::optional<std::int32_t> yOffset_mm; ///< DDI 135, left-/right+
		std::optional<std::int32_t> width_mm; ///< DDI 67 (or 70 / 68 when that is all there is)
	};

	/// @brief Result of grouping sections that cover the same strip of ground
	struct AogSectionMap
	{
		std::vector<std::uint8_t> aogSectionForSection; ///< Implement section -> AOG section
		std::uint8_t numberOfAogSections = 0;
		bool grouped = false; ///< true when at least two sections share an AOG section
	};

	/// @brief Identity mapping: implement section i is AOG section i
	inline AogSectionMap identity_map(std::uint8_t numberOfSections)
	{
		AogSectionMap map;
		for (std::uint8_t i = 0; i < numberOfSections; i++)
		{
			map.aogSectionForSection.push_back(i);
		}
		map.numberOfAogSections = numberOfSections;
		return map;
	}

	/// @brief Puts sections with the same lateral offset and width (within toleranceMm) into one AOG section.
	/// @details AOG sections are numbered in the order their first section appears, so an implement without
	/// overlapping sections keeps the identity mapping. Fore/aft (X) offset is ignored: several dispensers
	/// one behind the other still cover the same strip. Returns the identity mapping unless every section
	/// has a known Y offset and a width above zero.
	inline AogSectionMap group_overlapping_sections(const std::vector<SectionGeometry> &sections, std::int32_t toleranceMm = 10)
	{
		const auto count = static_cast<std::uint8_t>(sections.size());
		for (const auto &section : sections)
		{
			if (!section.yOffset_mm || !section.width_mm || (*section.width_mm <= 0))
			{
				return identity_map(count);
			}
		}

		AogSectionMap map;
		std::vector<std::size_t> representative; // AOG section -> first implement section in it
		for (std::size_t i = 0; i < sections.size(); i++)
		{
			std::optional<std::uint8_t> aogSection;
			for (std::size_t g = 0; g < representative.size(); g++)
			{
				const auto &first = sections[representative[g]];
				// 64-bit differences: two int32 values can be further apart than an int32 holds
				if ((std::llabs(static_cast<long long>(*first.yOffset_mm) - *sections[i].yOffset_mm) <= toleranceMm) &&
				    (std::llabs(static_cast<long long>(*first.width_mm) - *sections[i].width_mm) <= toleranceMm))
				{
					aogSection = static_cast<std::uint8_t>(g);
					break;
				}
			}
			if (!aogSection)
			{
				aogSection = static_cast<std::uint8_t>(representative.size());
				representative.push_back(i);
			}
			map.aogSectionForSection.push_back(*aogSection);
		}
		map.numberOfAogSections = static_cast<std::uint8_t>(representative.size());
		map.grouped = (map.numberOfAogSections < count);
		return map;
	}
} // namespace section_layout
