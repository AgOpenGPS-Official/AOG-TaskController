#pragma once

#include "isobus/isobus/isobus_device_descriptor_object_pool.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>

// Parent lookup for the device elements of a device descriptor object pool (DDOP).
namespace element_hierarchy
{
	using ParentMap = std::map<std::uint16_t, std::uint16_t>; ///< Element number -> parent element number

	/// @brief Maps each device element number to the element number of its parent element.
	/// @details ISO 11783-10 keeps the hierarchy in each element's parent object ID; an element's
	/// child list only references its process data and properties. Elements whose parent is the
	/// Device object, or an object that is not a device element, have no entry.
	/// The pool does not change after activation, so build this once and reuse it.
	inline ParentMap build_parent_map(isobus::DeviceDescriptorObjectPool &pool)
	{
		using isobus::task_controller_object::DeviceElementObject;
		using isobus::task_controller_object::ObjectTypes;

		ParentMap parents;
		for (std::uint16_t i = 0; i < pool.size(); i++)
		{
			auto object = pool.get_object_by_index(i);
			if (!object || (object->get_object_type() != ObjectTypes::DeviceElement))
			{
				continue;
			}

			auto element = std::static_pointer_cast<DeviceElementObject>(object);
			auto parentObject = pool.get_object_by_id(element->get_parent_object());
			if (!parentObject || (parentObject->get_object_type() != ObjectTypes::DeviceElement))
			{
				continue;
			}

			auto parent = std::static_pointer_cast<DeviceElementObject>(parentObject);
			if (parent->get_element_number() != element->get_element_number())
			{
				parents[element->get_element_number()] = parent->get_element_number();
			}
		}
		return parents;
	}

	/// @brief Returns true if the predicate holds for the element or any of its ancestors.
	/// @details Visits at most parents.size() + 1 elements, so a malformed pool with a parent
	/// cycle cannot loop forever.
	template<typename Predicate>
	bool any_in_chain(const ParentMap &parents, std::uint16_t elementNumber, Predicate &&predicate)
	{
		for (std::size_t visited = 0; visited <= parents.size(); visited++)
		{
			if (predicate(elementNumber))
			{
				return true;
			}

			auto parent = parents.find(elementNumber);
			if (parent == parents.end())
			{
				return false;
			}
			elementNumber = parent->second;
		}
		return false;
	}
} // namespace element_hierarchy
