#pragma once

#include <map>
#include <optional>
#include <set>
#include <vector>
#include "isobus/isobus/isobus_device_descriptor_object_pool.hpp"

// Some implements place DDI 141 on a dedicated function owning one section.
// This supplies actual feedback only; it never changes the setpoint protocol.
class SectionWorkStateFeedback
{
public:
	void configure(isobus::DeviceDescriptorObjectPool &pool, const std::vector<std::uint16_t> &sections)
	{
		using namespace isobus::task_controller_object;
		mappings.assign(sections.size(), Mapping{});
		samples.clear();
		std::map<std::uint16_t, std::shared_ptr<DeviceElementObject>> byId;
		std::map<std::uint16_t, std::shared_ptr<DeviceElementObject>> byNumber;
		for (std::uint32_t i = 0; i < pool.size(); ++i)
		{
			auto element = std::dynamic_pointer_cast<DeviceElementObject>(pool.get_object_by_index(i));
			if (element)
			{
				byId[element->get_object_id()] = element;
				byNumber[element->get_element_number()] = element;
			}
		}
		auto reportsWorkState = [&pool](const std::shared_ptr<DeviceElementObject> &element) {
			for (auto id : element->get_child_object_ids())
			{
				auto dpd = std::dynamic_pointer_cast<DeviceProcessDataObject>(pool.get_object_by_id(id));
				if (dpd && dpd->get_ddi() == 141)
					return true;
			}
			return false;
		};
		for (std::size_t index = 0; index < sections.size(); ++index)
		{
			auto found = byNumber.find(sections[index]);
			if (found == byNumber.end() || found->second->get_type() != DeviceElementObject::Type::Section)
				continue;
			auto section = found->second;
			auto candidate = section;
			if (!reportsWorkState(candidate))
			{
				auto parent = byId.find(section->get_parent_object());
				if (parent == byId.end() || parent->second->get_type() != DeviceElementObject::Type::Function)
					continue;
				candidate = parent->second;
				std::size_t childSections = 0;
				for (const auto &entry : byId)
					if (entry.second->get_type() == DeviceElementObject::Type::Section &&
					    entry.second->get_parent_object() == candidate->get_object_id())
						++childSections;
				// A boom-wide work bit cannot stand in for individual section feedback.
				if (childSections != 1 || !reportsWorkState(candidate))
					continue;
			}
			auto &mapping = mappings[index];
			mapping.feedback = candidate->get_element_number();
			std::set<std::uint16_t> visited;
			auto ancestor = section;
			while (ancestor)
			{
				if (!visited.insert(ancestor->get_object_id()).second)
				{
					mapping.feedback.reset();
					break;
				}
				mapping.ancestors.push_back(ancestor->get_element_number());
				auto parent = byId.find(ancestor->get_parent_object());
				ancestor = parent == byId.end() ? nullptr : parent->second;
			}
		}
	}

	void update(std::uint16_t element, bool working, std::uint32_t now)
	{
		samples[element] = { working, now };
	}

	void mark_direct_state(std::size_t section)
	{
		if (section < mappings.size())
			mappings[section].directState = true;
	}

	std::optional<std::uint8_t> get(std::size_t section, std::uint32_t now) const
	{
		if (section >= mappings.size())
			return std::nullopt;
		const auto &mapping = mappings[section];
		if (!mapping.feedback || mapping.directState)
			return std::nullopt;
		auto sample = samples.find(*mapping.feedback);
		if (sample == samples.end() || static_cast<std::uint32_t>(now - sample->second.time) >= 3000)
			return 0;
		for (auto ancestor : mapping.ancestors)
		{
			auto state = samples.find(ancestor);
			if (state != samples.end() && !state->second.working)
				return 0;
		}
		return sample->second.working ? 1 : 0;
	}

private:
	struct Mapping
	{
		std::optional<std::uint16_t> feedback;
		std::vector<std::uint16_t> ancestors;
		bool directState = false;
	};
	struct Sample
	{
		bool working;
		std::uint32_t time;
	};
	std::vector<Mapping> mappings;
	std::map<std::uint16_t, Sample> samples;
};
