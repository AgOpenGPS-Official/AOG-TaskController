/**
 * @brief Checks the section control method the TC picks for every DDOP in tools/testdata/implements.
 *
 * The expected method is the section_control column of manifest.csv. A method only counts when it
 * also leaves something to send, and the MIDI 3 pools are checked down to the target elements.
 *
 * Usage: test_section_control <path to tools/testdata/implements>
 * Returns 0 when every check passes, 1 otherwise.
 */

#include "section_control_method.hpp"

#include "isobus/isobus/isobus_task_controller_server.hpp"

#include <cstdio>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

static int failures = 0;

static void check(bool condition, const std::string &label)
{
	if (!condition)
	{
		std::fprintf(stderr, "  FAIL: %s\n", label.c_str());
		++failures;
	}
}

// One CSV record per line; fields may be quoted and contain commas.
static std::vector<std::string> split_csv_line(const std::string &line)
{
	std::vector<std::string> fields(1);
	bool quoted = false;
	for (std::size_t i = 0; i < line.size(); i++)
	{
		char c = line[i];
		if (quoted && c == '"' && i + 1 < line.size() && line[i + 1] == '"')
		{
			fields.back() += '"';
			i++;
		}
		else if (c == '"')
		{
			quoted = !quoted;
		}
		else if (c == ',' && !quoted)
		{
			fields.emplace_back();
		}
		else if (c != '\r')
		{
			fields.back() += c;
		}
	}
	return fields;
}

static bool load_pool(const std::string &path, isobus::DeviceDescriptorObjectPool &pool)
{
	std::ifstream file(path, std::ios::binary);
	std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
	// Same compatibility level as MyTCServer::activate_object_pool()
	pool.set_task_controller_compatibility_level(static_cast<std::uint8_t>(isobus::TaskControllerServer::TaskControllerVersion::SecondEditionDraft));
	return !bytes.empty() && pool.deserialize_binary_object_pool(bytes.data(), static_cast<std::uint32_t>(bytes.size()));
}

static std::string describe(const SectionControlPlan &plan)
{
	std::ostringstream text;
	for (const auto &target : plan.condensedTargets)
	{
		text << " " << target.first << "->" << target.second;
	}
	if (plan.uses_per_element_control())
	{
		text << " " << plan.perElementSetpointDDI << "->";
		for (std::size_t i = 0; i < plan.perElementTargets.size(); i++)
		{
			text << (i ? "," : "") << plan.perElementTargets[i];
		}
	}
	if (plan.hasSetpointWorkState)
	{
		text << " 289(global)->" << plan.setpointWorkStateElement;
	}
	return text.str();
}

// True when the plan leaves the TC something to write for the method it picked
static bool has_targets(const SectionControlPlan &plan)
{
	switch (plan.method)
	{
		case SectionControlMethod::CondensedSetpoint:
		case SectionControlMethod::SettableCondensedActual:
			return !plan.condensedTargets.empty();
		case SectionControlMethod::PerElementSetpointWorkState:
		case SectionControlMethod::PerElementActualWorkState:
			for (auto element : plan.perElementTargets)
			{
				if (element != isobus::NULL_OBJECT_ID)
					return true;
			}
			return false;
		default:
			return true;
	}
}

static void check_midi3_targets(const std::string &file, const SectionControlPlan &plan)
{
	const std::string base = "sprayer/ME_SPRITZE_SPR_MIDI_3_11sec";
	const std::vector<std::uint16_t> sections = { 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20 };
	const std::map<std::uint16_t, std::uint16_t> ddi290OnBoom = { { 290, 2 } };
	const std::map<std::uint16_t, std::uint16_t> ddi161OnBoom = { { 161, 2 } };

	if (file == base + ".ddop")
	{
		check(plan.condensedTargets == ddi290OnBoom, file + ": DDI 290 on element 2");
	}
	else if (file == base + "_oldSC_CWS_synthetic.ddop")
	{
		check(plan.condensedTargets == ddi161OnBoom, file + ": settable DDI 161 on element 2");
	}
	else if (file == base + "_newSC_perElement_synthetic.ddop")
	{
		check(plan.perElementSetpointDDI == 289 && plan.perElementTargets == sections, file + ": DDI 289 on elements 10..20");
	}
	else if (file == base + "_oldSC_perElement_synthetic.ddop")
	{
		check(plan.perElementSetpointDDI == 141 && plan.perElementTargets == sections, file + ": DDI 141 on elements 10..20");
	}
	else
	{
		return;
	}
	// DDI 289 is unlinked (condensed modes) or only on the sections (per-element modes): never a global target
	check(!plan.hasSetpointWorkState, file + ": no global DDI 289");
}

int main(int argc, char **argv)
{
	if (argc < 2)
	{
		std::fprintf(stderr, "Usage: %s <path to tools/testdata/implements>\n", argv[0]);
		return 1;
	}
	const std::string root = argv[1];
	std::ifstream manifest(root + "/manifest.csv");
	check(manifest.is_open(), "open " + root + "/manifest.csv");

	std::string line;
	std::getline(manifest, line);
	auto header = split_csv_line(line);
	std::size_t fileColumn = header.size(), methodColumn = header.size();
	for (std::size_t i = 0; i < header.size(); i++)
	{
		if (header[i] == "file")
			fileColumn = i;
		if (header[i] == "section_control")
			methodColumn = i;
	}
	check(fileColumn < header.size() && methodColumn < header.size(), "manifest has file and section_control columns");

	int pools = 0;
	while (std::getline(manifest, line) && fileColumn < header.size() && methodColumn < header.size())
	{
		auto fields = split_csv_line(line);
		if (fields.size() < header.size())
			continue;
		const auto &file = fields[fileColumn];
		const auto &expected = fields[methodColumn];
		pools++;

		isobus::DeviceDescriptorObjectPool pool;
		if (!load_pool(root + "/" + file, pool))
		{
			check(false, file + ": failed to load");
			continue;
		}
		auto sections = get_section_element_numbers(isobus::DeviceDescriptorObjectPoolHelper::get_implement_geometry(pool));
		auto plan = resolve_section_control(pool, sections);

		bool ok = (expected == to_string(plan.method)) && has_targets(plan);
		std::printf("%-4s %-70s expected %-6s got %-6s%s\n", ok ? "ok" : "FAIL", file.c_str(), expected.c_str(), to_string(plan.method), describe(plan).c_str());
		check(expected == to_string(plan.method), file + ": expected " + expected + ", got " + to_string(plan.method));
		check(has_targets(plan), file + ": " + to_string(plan.method) + " picked but nothing to send");
		check_midi3_targets(file, plan);
	}
	check(pools > 0, "manifest lists at least one pool");

	if (failures != 0)
	{
		std::fprintf(stderr, "%d check(s) failed\n", failures);
		return 1;
	}
	std::printf("All %d pools resolve to their expected section control method\n", pools);
	return 0;
}
