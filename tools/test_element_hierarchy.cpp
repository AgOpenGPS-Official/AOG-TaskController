/**
 * @brief Unit tests for element_hierarchy, the parent lookup behind ClientState::is_element_or_parent_off().
 *
 * Returns 0 when every assertion passes, 1 otherwise.
 */

#include "element_hierarchy.hpp"

#include <cstdio>
#include <set>

using Element = isobus::task_controller_object::DeviceElementObject;

static int failures = 0;

static void check(bool condition, const char *label)
{
	if (!condition)
	{
		std::fprintf(stderr, "  FAIL: %s\n", label);
		++failures;
	}
}

// Device -> root element 0 -> boom 1 -> sections 2 and 3. As in a real pool, the hierarchy is only
// in the parent object IDs: no element lists another element as a child.
static bool build_boom_pool(isobus::DeviceDescriptorObjectPool &pool)
{
	bool ok = pool.add_device("Sprayer", "1", "SN1", "SPRAY", {}, {}, 0x1234);
	ok = ok && pool.add_device_element("Root", 0, 0, Element::Type::Device, 1);
	ok = ok && pool.add_device_element("Section 2", 2, 10, Element::Type::Section, 20);
	ok = ok && pool.add_device_element("Section 3", 3, 10, Element::Type::Section, 21);
	ok = ok && pool.add_device_element("Boom", 1, 1, Element::Type::Function, 10);
	ok = ok && pool.add_device_process_data("Actual Work State", 141, 0xFFFF, 0, 0, 30);
	// Parent points at process data, not an element: such an element has no parent entry.
	ok = ok && pool.add_device_element("Orphan", 7, 30, Element::Type::Unit, 40);
	return ok;
}

static void test_parents_come_from_parent_object_ids()
{
	isobus::DeviceDescriptorObjectPool pool;
	check(build_boom_pool(pool), "boom pool builds");
	auto parents = element_hierarchy::build_parent_map(pool);

	check(parents.size() == 3, "only elements under another element get an entry");
	check(parents.count(2) && parents.at(2) == 1, "section 2 -> boom 1");
	check(parents.count(3) && parents.at(3) == 1, "section 3 -> boom 1");
	check(parents.count(1) && parents.at(1) == 0, "boom 1 -> root 0");
	check(!parents.count(0), "root element's parent is the Device object, not an element");
	check(!parents.count(7), "element whose parent is not an element has no entry");
}

static void test_chain_walks_ancestors_only()
{
	isobus::DeviceDescriptorObjectPool pool;
	check(build_boom_pool(pool), "boom pool builds");
	auto parents = element_hierarchy::build_parent_map(pool);

	std::set<std::uint16_t> visited;
	element_hierarchy::any_in_chain(parents, 2, [&](std::uint16_t element) {
		visited.insert(element);
		return false;
	});
	check(visited == std::set<std::uint16_t>{ 2, 1, 0 }, "section 2 visits itself, boom and root");

	auto isOff = [](std::set<std::uint16_t> off) {
		return [off](std::uint16_t element) { return off.count(element) > 0; };
	};
	check(element_hierarchy::any_in_chain(parents, 2, isOff({ 0 })), "root off turns section 2 off");
	check(element_hierarchy::any_in_chain(parents, 2, isOff({ 1 })), "boom off turns section 2 off");
	check(!element_hierarchy::any_in_chain(parents, 2, isOff({ 3 })), "sibling section off leaves section 2 on");
	check(!element_hierarchy::any_in_chain(parents, 1, isOff({ 2 })), "child section off leaves boom on");
	check(element_hierarchy::any_in_chain(parents, 99, isOff({ 99 })), "unknown element still checks itself");
}

static void test_parent_cycle_terminates()
{
	isobus::DeviceDescriptorObjectPool pool;
	bool ok = pool.add_device("Broken", "1", "SN1", "BRKN", {}, {}, 0x1234);
	ok = ok && pool.add_device_element("A", 5, 51, Element::Type::Function, 50);
	ok = ok && pool.add_device_element("B", 6, 50, Element::Type::Function, 51);
	check(ok, "cyclic pool builds");
	auto parents = element_hierarchy::build_parent_map(pool);

	int calls = 0;
	bool result = element_hierarchy::any_in_chain(parents, 5, [&](std::uint16_t) {
		++calls;
		return false;
	});
	check(!result, "cycle with nothing off reports on");
	check(calls <= 3, "cycle is cut off after parents.size() + 1 elements");
}

int main()
{
	test_parents_come_from_parent_object_ids();
	test_chain_walks_ancestors_only();
	test_parent_cycle_terminates();

	if (failures != 0)
	{
		std::fprintf(stderr, "%d check(s) failed\n", failures);
		return 1;
	}
	std::printf("All element hierarchy tests passed\n");
	return 0;
}
