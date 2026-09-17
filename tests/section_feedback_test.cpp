#include "section_work_state_feedback.hpp"

#include <array>
#include <iostream>
#include <stdexcept>

using Element = isobus::task_controller_object::DeviceElementObject;

static void check(bool condition)
{
	if (!condition)
		throw std::runtime_error("section feedback regression");
}

static std::shared_ptr<Element> element(isobus::DeviceDescriptorObjectPool &pool, std::uint16_t id)
{
	auto result = std::dynamic_pointer_cast<Element>(pool.get_object_by_id(id));
	check(result != nullptr);
	return result;
}

// Synthetic DDOP: object IDs deliberately differ from process-data element numbers.
static isobus::DeviceDescriptorObjectPool fixture()
{
	isobus::DeviceDescriptorObjectPool pool(3);
	check(pool.add_device("Synthetic sprayer", "1", "", "TEST", {}, {}, 0));
	check(pool.add_device_element("Device", 1, 0, Element::Type::Device, 1));
	check(pool.add_device_element("Boom", 2, 1, Element::Type::Function, 2));
	check(pool.add_device_process_data("Actual work state", 141, 0xFFFF, 0, 0, 500));
	for (std::uint16_t i = 0; i < 9; ++i)
	{
		check(pool.add_device_element("Function", 10 + i, 2, Element::Type::Function, 30 + i));
		check(pool.add_device_element("Section", 100 + i, 30 + i, Element::Type::Section, 100 + i));
		element(pool, 30 + i)->add_reference_to_child_object(500);
	}
	return pool;
}

int main()
{
	auto pool = fixture();
	SectionWorkStateFeedback feedback;
	const std::vector<std::uint16_t> sections{ 100, 101, 102, 103, 104, 105, 106, 107, 108 };
	feedback.configure(pool, sections);
	check(feedback.get(1, 100) == 0);
	feedback.update(11, true, 100);
	check(feedback.get(1, 100) == 1);
	check(feedback.get(0, 100) == 0);
	check(feedback.get(2, 100) == 0);
	feedback.update(2, false, 100);
	check(feedback.get(1, 100) == 0);
	feedback.update(2, true, 101);
	check(feedback.get(1, 101) == 1);
	check(feedback.get(1, 3099) == 1);
	check(feedback.get(1, 3100) == 0);
	feedback.update(11, false, 3101);
	check(feedback.get(1, 3101) == 0);
	feedback.update(11, true, 3102);
	feedback.mark_direct_state(1);
	check(!feedback.get(1, 3102).has_value());
	feedback.configure(pool, sections);
	check(feedback.get(1, 3102) == 0);
	for (std::uint16_t i = 0; i < 9; ++i)
		feedback.update(10 + i, true, 0xFFFFFFF0);
	for (std::size_t i = 0; i < 9; ++i)
		check(feedback.get(i, 0x00000010) == 1);
	check(!feedback.get(9, 0x10).has_value());

	check(pool.add_device_element("Shared section", 200, 30, Element::Type::Section, 200));
	feedback.configure(pool, { 100 });
	feedback.update(10, true, 100);
	check(!feedback.get(0, 100).has_value());

	pool = fixture();
	element(pool, 31)->set_parent_object(101);
	feedback.configure(pool, { 101 });
	feedback.update(11, true, 100);
	check(!feedback.get(0, 100).has_value());

	pool = fixture();
	element(pool, 100)->set_parent_object(999);
	feedback.configure(pool, { 100, 999 });
	check(!feedback.get(0, 100).has_value());
	check(!feedback.get(1, 100).has_value());

	pool = fixture();
	element(pool, 100)->add_reference_to_child_object(500);
	feedback.configure(pool, { 100 });
	feedback.update(10, true, 100);
	check(feedback.get(0, 100) == 0);
	feedback.update(100, true, 101);
	check(feedback.get(0, 101) == 1);
	std::cout << "PASS: synthetic DDOP mapping, freshness, master OFF, precedence and invalid hierarchy\n";
}
