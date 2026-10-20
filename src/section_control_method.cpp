/**
 * @brief Works out how to switch an implement's sections from its DDOP
 */
#include "section_control_method.hpp"

#include "isobus/isobus/isobus_standard_data_description_indices.hpp"

#include <algorithm>

using isobus::DataDescriptionIndex;
using isobus::task_controller_object::DeviceElementObject;
using isobus::task_controller_object::DeviceProcessDataObject;
using isobus::task_controller_object::ObjectTypes;

namespace
{
	constexpr auto SETPOINT_WORK_STATE = static_cast<std::uint16_t>(DataDescriptionIndex::SetpointWorkState);
	constexpr auto ACTUAL_WORK_STATE = static_cast<std::uint16_t>(DataDescriptionIndex::ActualWorkState);
	constexpr auto SETPOINT_CONDENSED_FIRST = static_cast<std::uint16_t>(DataDescriptionIndex::SetpointCondensedWorkState1_16);
	constexpr auto SETPOINT_CONDENSED_LAST = static_cast<std::uint16_t>(DataDescriptionIndex::SetpointCondensedWorkState241_256);
	constexpr auto ACTUAL_CONDENSED_FIRST = static_cast<std::uint16_t>(DataDescriptionIndex::ActualCondensedWorkState1_16);
	constexpr auto ACTUAL_CONDENSED_LAST = static_cast<std::uint16_t>(DataDescriptionIndex::ActualCondensedWorkState241_256);

	/// One process data object as linked from one device element. A process data object that several
	/// elements link appears once per element; one that no element links does not appear at all.
	struct LinkedProcessData
	{
		std::uint16_t ddi;
		std::uint16_t elementNumber;
		bool settable;
	};

	std::vector<LinkedProcessData> get_linked_process_data(isobus::DeviceDescriptorObjectPool &pool)
	{
		std::vector<LinkedProcessData> links;
		for (std::uint32_t i = 0; i < pool.size(); i++)
		{
			auto object = pool.get_object_by_index(i);
			if (!object || object->get_object_type() != ObjectTypes::DeviceElement)
			{
				continue;
			}
			auto element = std::static_pointer_cast<DeviceElementObject>(object);

			// Read the children by index. Iterating get_child_object_ids() reads past the end of the list:
			// AgIsoStack's DataSpan scales end() by sizeof(T) twice (see #110).
			for (std::uint16_t c = 0; c < element->get_number_child_objects(); c++)
			{
				auto child = pool.get_object_by_id(element->get_child_object_id(c));
				if (!child || child->get_object_type() != ObjectTypes::DeviceProcessData)
				{
					continue;
				}
				auto processData = std::static_pointer_cast<DeviceProcessDataObject>(child);
				links.push_back({ processData->get_ddi(),
				                  element->get_element_number(),
				                  processData->has_property(DeviceProcessDataObject::PropertiesBit::Settable) });
			}
		}
		return links;
	}

	bool is_in(std::uint16_t value, std::uint16_t first, std::uint16_t last)
	{
		return value >= first && value <= last;
	}

	// The element of each section index that links a settable setpointDDI, NULL_OBJECT_ID where none does.
	// Returns an empty list when no section links one.
	std::vector<std::uint16_t> get_per_element_targets(const std::vector<LinkedProcessData> &links,
	                                                   const std::vector<std::uint16_t> &sectionElementNumbers,
	                                                   std::uint16_t setpointDDI)
	{
		std::vector<std::uint16_t> targets;
		bool anyTarget = false;
		for (auto sectionElement : sectionElementNumbers)
		{
			bool linked = std::any_of(links.begin(), links.end(), [&](const LinkedProcessData &link) {
				return link.ddi == setpointDDI && link.settable && link.elementNumber == sectionElement;
			});
			targets.push_back(linked ? sectionElement : isobus::NULL_OBJECT_ID);
			anyTarget = anyTarget || linked;
		}
		return anyTarget ? targets : std::vector<std::uint16_t>();
	}
}

const char *to_string(SectionControlMethod method)
{
	switch (method)
	{
		case SectionControlMethod::CondensedSetpoint:
			return "DDI290";
		case SectionControlMethod::SettableCondensedActual:
			return "DDI161";
		case SectionControlMethod::PerElementSetpointWorkState:
			return "DDI289";
		case SectionControlMethod::PerElementActualWorkState:
			return "DDI141";
		default:
			return "none";
	}
}

bool SectionControlPlan::uses_per_element_control() const
{
	return method == SectionControlMethod::PerElementSetpointWorkState || method == SectionControlMethod::PerElementActualWorkState;
}

std::vector<std::uint16_t> get_section_element_numbers(const isobus::DeviceDescriptorObjectPoolHelper::Implement &implement)
{
	// Same order as the geometry rows in MyTCServer::activate_object_pool(): the index is AOG's section index
	std::vector<std::uint16_t> elements;
	for (const auto &boom : implement.booms)
	{
		for (const auto &subBoom : boom.subBooms)
		{
			for (const auto &section : subBoom.sections)
			{
				elements.push_back(section.elementNumber);
			}
		}
		for (const auto &section : boom.sections)
		{
			elements.push_back(section.elementNumber);
		}
	}
	return elements;
}

SectionControlPlan resolve_section_control(isobus::DeviceDescriptorObjectPool &pool, const std::vector<std::uint16_t> &sectionElementNumbers)
{
	SectionControlPlan plan;
	if (sectionElementNumbers.empty())
	{
		return plan; // Nothing to switch, e.g. a tractor
	}

	const auto links = get_linked_process_data(pool);
	auto isSection = [&sectionElementNumbers](std::uint16_t elementNumber) {
		return std::find(sectionElementNumbers.begin(), sectionElementNumbers.end(), elementNumber) != sectionElementNumbers.end();
	};

	// A global Setpoint Work State lives above the sections. On a section element it is a per-element setpoint.
	for (const auto &link : links)
	{
		if (link.ddi == SETPOINT_WORK_STATE && link.settable && !isSection(link.elementNumber))
		{
			plan.hasSetpointWorkState = true;
			plan.setpointWorkStateElement = link.elementNumber;
			break;
		}
	}

	// Setpoint condensed work state (DDI 290+). The first linking element wins if several link the same DDI.
	for (const auto &link : links)
	{
		if (is_in(link.ddi, SETPOINT_CONDENSED_FIRST, SETPOINT_CONDENSED_LAST))
		{
			plan.condensedTargets.emplace(link.ddi, link.elementNumber);
		}
	}
	if (!plan.condensedTargets.empty())
	{
		plan.method = SectionControlMethod::CondensedSetpoint;
		return plan;
	}

	// Actual condensed work state (DDI 161+) declared settable, the fallback for implements without setpoint DDIs
	for (const auto &link : links)
	{
		if (is_in(link.ddi, ACTUAL_CONDENSED_FIRST, ACTUAL_CONDENSED_LAST) && link.settable)
		{
			plan.condensedTargets.emplace(link.ddi, link.elementNumber);
		}
	}
	if (!plan.condensedTargets.empty())
	{
		plan.method = SectionControlMethod::SettableCondensedActual;
		return plan;
	}

	// Per-element: Setpoint Work State on each section, or else a settable Actual Work State on each section
	plan.perElementTargets = get_per_element_targets(links, sectionElementNumbers, SETPOINT_WORK_STATE);
	if (!plan.perElementTargets.empty())
	{
		plan.method = SectionControlMethod::PerElementSetpointWorkState;
		plan.perElementSetpointDDI = SETPOINT_WORK_STATE;
		return plan;
	}
	plan.perElementTargets = get_per_element_targets(links, sectionElementNumbers, ACTUAL_WORK_STATE);
	if (!plan.perElementTargets.empty())
	{
		plan.method = SectionControlMethod::PerElementActualWorkState;
		plan.perElementSetpointDDI = ACTUAL_WORK_STATE;
	}
	return plan;
}
