/**
 * @brief Unit tests for section_layout: condensed-DDI owners per boom and grouping of overlapping sections.
 *
 * The pools model the two layouts an RDS Artemis seeder controller uploads for three dispensers on one
 * working width: three Functions with one Section each, or ("linked") one Function with three Sections.
 *
 * Returns 0 when every assertion passes, 1 otherwise.
 */

#include "section_layout.hpp"

#include "isobus/isobus/isobus_device_descriptor_object_pool_helpers.hpp"

#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iterator>

using Element = isobus::task_controller_object::DeviceElementObject;
using section_layout::SectionGeometry;

static int failures = 0;

static void check(bool condition, const char *label)
{
	if (!condition)
	{
		std::fprintf(stderr, "  FAIL: %s\n", label);
		++failures;
	}
}

static constexpr std::uint16_t DDI_SETPOINT_CONDENSED_1_16 = 290;
static constexpr std::uint16_t DDI_SETPOINT_CONDENSED_241_256 = 305;

static std::shared_ptr<Element> element_by_id(isobus::DeviceDescriptorObjectPool &pool, std::uint16_t objectId)
{
	return std::static_pointer_cast<Element>(pool.get_object_by_id(objectId));
}

// Device element 0 (object 1); Functions as elements 2.. (objects 3..); Sections as elements 6.. (objects 100..).
// Each Function links DDI 290 / 161, so each is a condensed owner. sectionsPerFunction lists the Sections per Function.
static bool build_seeder_pool(isobus::DeviceDescriptorObjectPool &pool, const std::vector<int> &sectionsPerFunction)
{
	bool ok = pool.add_device("Seeder", "1", "0", "SEED", {}, {}, 0);
	ok = ok && pool.add_device_element("Device", 0, 0, Element::Type::Device, 1);
	ok = ok && pool.add_device_process_data("Sections Set", DDI_SETPOINT_CONDENSED_1_16, 0xFFFF, 0x02, 0, 504);
	ok = ok && pool.add_device_process_data("Sections Actual", 161, 0xFFFF, 0x01, 0x09, 503);

	std::uint16_t sectionElement = 6;
	std::uint16_t sectionObject = 100;
	for (std::size_t f = 0; f < sectionsPerFunction.size(); f++)
	{
		const auto functionElement = static_cast<std::uint16_t>(2 + f);
		const auto functionObject = static_cast<std::uint16_t>(3 + f);
		ok = ok && pool.add_device_element("Seeder", functionElement, 1, Element::Type::Function, functionObject);
		if (!ok)
		{
			return false;
		}
		element_by_id(pool, functionObject)->add_reference_to_child_object(503);
		element_by_id(pool, functionObject)->add_reference_to_child_object(504);
		for (int s = 0; s < sectionsPerFunction[f]; s++)
		{
			ok = ok && pool.add_device_element("Section", sectionElement++, functionObject, Element::Type::Section, sectionObject++);
		}
	}
	return ok;
}

static std::vector<std::uint16_t> section_elements(std::size_t count)
{
	std::vector<std::uint16_t> elements;
	for (std::size_t i = 0; i < count; i++)
	{
		elements.push_back(static_cast<std::uint16_t>(6 + i));
	}
	return elements;
}

static std::vector<section_layout::CondensedOwner> owners_of(isobus::DeviceDescriptorObjectPool &pool, std::size_t sections)
{
	return section_layout::resolve_condensed_owners(pool, element_hierarchy::build_parent_map(pool), section_elements(sections), DDI_SETPOINT_CONDENSED_1_16, DDI_SETPOINT_CONDENSED_241_256);
}

static void test_one_owner_per_function()
{
	isobus::DeviceDescriptorObjectPool pool;
	check(build_seeder_pool(pool, { 1, 1, 1 }), "three-function pool builds");
	const auto owners = owners_of(pool, 3);
	check(owners.size() == 3, "three Functions with their own DDI 290 give three owners");
	for (std::size_t i = 0; (i < owners.size()) && (i < 3); i++)
	{
		check(owners[i].elementNumber == 2 + i, "owner is the Function element");
		check(owners[i].firstSection == i, "owner starts at its own section");
		check(owners[i].count == 1, "owner has one section");
	}
}

static void test_linked_layout_has_one_owner()
{
	isobus::DeviceDescriptorObjectPool pool;
	check(build_seeder_pool(pool, { 3 }), "linked pool builds");
	const auto owners = owners_of(pool, 3);
	check(owners.size() == 1, "one Function with three Sections gives one owner");
	check(!owners.empty() && (owners[0].elementNumber == 2) && (owners[0].count == 3), "the one owner has all three sections");
}

static void test_mixed_section_counts()
{
	isobus::DeviceDescriptorObjectPool pool;
	check(build_seeder_pool(pool, { 1, 2 }), "two-function pool builds");
	const auto owners = owners_of(pool, 3);
	check(owners.size() == 2, "two Functions give two owners");
	check((owners.size() == 2) && (owners[1].firstSection == 1) && (owners[1].count == 2), "second owner has sections 2-3");
}

static void test_owner_on_device_element()
{
	// A single-boom sprayer that puts DDI 290 on the device element: one owner, element 0
	isobus::DeviceDescriptorObjectPool pool;
	bool ok = pool.add_device("Sprayer", "1", "0", "SPRAY", {}, {}, 0);
	ok = ok && pool.add_device_element("Device", 0, 0, Element::Type::Device, 1);
	ok = ok && pool.add_device_process_data("Sections Set", DDI_SETPOINT_CONDENSED_1_16, 0xFFFF, 0x02, 0, 504);
	ok = ok && pool.add_device_element("Boom", 1, 1, Element::Type::Function, 3);
	ok = ok && pool.add_device_element("Section", 6, 3, Element::Type::Section, 100);
	ok = ok && pool.add_device_element("Section", 7, 3, Element::Type::Section, 101);
	check(ok, "sprayer pool builds");
	element_by_id(pool, 1)->add_reference_to_child_object(504);
	const auto owners = owners_of(pool, 2);
	check((owners.size() == 1) && (owners[0].elementNumber == 0) && (owners[0].count == 2), "DDI 290 on the device element owns every section");
}

static void test_missing_owner_falls_back()
{
	isobus::DeviceDescriptorObjectPool pool;
	bool ok = pool.add_device("Bare", "1", "0", "BARE", {}, {}, 0);
	ok = ok && pool.add_device_element("Device", 0, 0, Element::Type::Device, 1);
	ok = ok && pool.add_device_element("Section", 6, 1, Element::Type::Section, 100);
	check(ok, "bare pool builds");
	check(owners_of(pool, 1).empty(), "no element with DDI 290 gives no owners");
}

static void test_read_only_actual_is_not_an_owner()
{
	// The seeder pool links DDI 161 read-only (properties 0x01): it reports, but cannot be commanded
	isobus::DeviceDescriptorObjectPool pool;
	check(build_seeder_pool(pool, { 1, 1, 1 }), "three-function pool builds");
	const auto parents = element_hierarchy::build_parent_map(pool);
	check(section_layout::resolve_condensed_owners(pool, parents, section_elements(3), 161, 176).size() == 3, "any linked DDI 161 counts when settability is not asked for");
	check(section_layout::resolve_condensed_owners(pool, parents, section_elements(3), 161, 176, true).empty(), "a read-only DDI 161 is not a command target");
	check(section_layout::resolve_condensed_owners(pool, parents, section_elements(3), DDI_SETPOINT_CONDENSED_1_16, DDI_SETPOINT_CONDENSED_241_256, true).size() == 3, "a settable DDI 290 still is");
}

static void test_grouping()
{
	using section_layout::group_overlapping_sections;

	auto same = group_overlapping_sections({ { 0, 3000 }, { 0, 3000 }, { 5, 3004 } });
	check(same.numberOfAogSections == 1 && same.grouped, "three dispensers on one width are one AOG section");
	check(same.aogSectionForSection == std::vector<std::uint8_t>({ 0, 0, 0 }), "all three map to AOG section 1");

	auto sideBySide = group_overlapping_sections({ { -1000, 1000 }, { 0, 1000 }, { 1000, 1000 } });
	check(!sideBySide.grouped && sideBySide.numberOfAogSections == 3, "side-by-side sections stay separate");
	check(sideBySide.aogSectionForSection == std::vector<std::uint8_t>({ 0, 1, 2 }), "side-by-side keeps the identity mapping");

	auto twoProducts = group_overlapping_sections({ { -500, 1000 }, { 500, 1000 }, { -500, 1000 }, { 500, 1000 } });
	check(twoProducts.numberOfAogSections == 2, "two products over two sections give two AOG sections");
	check(twoProducts.aogSectionForSection == std::vector<std::uint8_t>({ 0, 1, 0, 1 }), "each product's section follows its strip");

	auto unknown = group_overlapping_sections({ { 0, 3000 }, { std::nullopt, 3000 } });
	check(!unknown.grouped && unknown.numberOfAogSections == 2, "an unknown offset keeps the identity mapping");

	auto noWidth = group_overlapping_sections({ { 0, 0 }, { 0, 0 } });
	check(!noWidth.grouped && noWidth.numberOfAogSections == 2, "zero widths (unfilled DDOP) keep the identity mapping");

	auto differentWidth = group_overlapping_sections({ { 0, 3000 }, { 0, 1500 } });
	check(!differentWidth.grouped, "same centre but different width stays separate");

	// Offsets at the ends of the int32 range differ by more than an int32 holds
	auto extremes = group_overlapping_sections({ { INT32_MIN, 3000 }, { INT32_MAX, 3000 }, { 0, INT32_MAX }, { 0, 1 } });
	check(!extremes.grouped && extremes.numberOfAogSections == 4, "int32 extremes do not wrap around into one AOG section");
}

// Prints the owners and the DDOP's fixed geometry for a captured binary DDOP, in the TC's section order
static int describe_ddop(const char *path)
{
	std::ifstream file(path, std::ios::binary);
	std::vector<std::uint8_t> data((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
	isobus::DeviceDescriptorObjectPool pool;
	pool.set_task_controller_compatibility_level(3); // As MyTCServer::activate_object_pool() does
	if (data.empty() || !pool.deserialize_binary_object_pool(data.data(), static_cast<std::uint32_t>(data.size()), isobus::NAME(0)))
	{
		std::fprintf(stderr, "%s: cannot read DDOP\n", path);
		return 1;
	}
	std::vector<std::uint16_t> sections;
	std::vector<SectionGeometry> geometry;
	const auto implement = isobus::DeviceDescriptorObjectPoolHelper::get_implement_geometry(pool);
	auto add = [&](const isobus::DeviceDescriptorObjectPoolHelper::Section &section) {
		sections.push_back(section.elementNumber);
		geometry.push_back({ section.yOffset_mm.exists() ? std::optional<std::int32_t>(section.yOffset_mm.get()) : std::nullopt,
		                     section.width_mm.exists() ? std::optional<std::int32_t>(section.width_mm.get()) : std::nullopt });
	};
	for (const auto &boom : implement.booms)
	{
		for (const auto &subBoom : boom.subBooms)
		{
			for (const auto &section : subBoom.sections)
			{
				add(section);
			}
		}
		for (const auto &section : boom.sections)
		{
			add(section);
		}
	}
	std::printf("%s: %zu section(s)\n", path, sections.size());
	for (std::size_t i = 0; i < sections.size(); i++)
	{
		std::printf("  section %zu: element %u\n", i + 1, sections[i]);
	}
	const auto owners = section_layout::resolve_condensed_owners(pool, element_hierarchy::build_parent_map(pool), sections, DDI_SETPOINT_CONDENSED_1_16, DDI_SETPOINT_CONDENSED_241_256);
	std::printf("  %zu condensed owner(s)%s\n", owners.size(), owners.size() > 1 ? ", one setpoint each" : "");
	for (const auto &owner : owners)
	{
		std::printf("    element %u: sections %d-%d\n", owner.elementNumber, owner.firstSection + 1, owner.firstSection + owner.count);
	}
	const auto map = section_layout::group_overlapping_sections(geometry);
	std::printf("  fixed geometry gives %d AOG section(s)%s\n", map.numberOfAogSections, map.grouped ? " (grouped)" : "");
	return 0;
}

int main(int argc, char **argv)
{
	if (argc > 1)
	{
		int result = 0;
		for (int i = 1; i < argc; i++)
		{
			result |= describe_ddop(argv[i]);
		}
		return result;
	}

	test_one_owner_per_function();
	test_linked_layout_has_one_owner();
	test_mixed_section_counts();
	test_owner_on_device_element();
	test_missing_owner_falls_back();
	test_read_only_actual_is_not_an_owner();
	test_grouping();

	if (failures != 0)
	{
		std::fprintf(stderr, "%d check(s) failed\n", failures);
		return 1;
	}
	std::printf("section_layout: all checks passed\n");
	return 0;
}
