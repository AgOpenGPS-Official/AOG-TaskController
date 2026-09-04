#include "field_registry.hpp"

#include "logging_utils.hpp"
#include "settings.hpp"

#include <fstream>
#include <iostream>
#include <limits>

namespace
{
	constexpr char REGISTRY_FILE_NAME[] = "field_registry.csv";
}

FieldRegistry::FieldRegistry()
{
	filePath = Settings::get_filename_path(REGISTRY_FILE_NAME);
	load();
}

void FieldRegistry::load()
{
	std::ifstream in(filePath);
	if (!in.is_open())
	{
		return; // No registry yet - fields will be indexed fresh as they're seen.
	}

	std::string line;
	std::uint32_t highestIndex = 0;
	bool haveAny = false;
	while (std::getline(in, line))
	{
		if (!line.empty() && line.back() == '\r')
		{
			line.pop_back();
		}
		const auto commaPos = line.find(',');
		if (commaPos == std::string::npos || commaPos == 0)
		{
			continue; // Malformed line - skip it rather than aborting the whole load.
		}

		try
		{
			const unsigned long parsedIndex = std::stoul(line.substr(0, commaPos));
			if (parsedIndex > std::numeric_limits<std::uint16_t>::max())
			{
				continue;
			}
			const auto index = static_cast<std::uint16_t>(parsedIndex);
			const std::string name = line.substr(commaPos + 1);
			nameToIndex[name] = index;
			haveAny = true;
			if (static_cast<std::uint32_t>(index) > highestIndex)
			{
				highestIndex = index;
			}
		}
		catch (const std::exception &)
		{
			continue; // Non-numeric index - skip this line.
		}
	}

	if (haveAny)
	{
		nextIndex = (highestIndex < std::numeric_limits<std::uint16_t>::max()) ? static_cast<std::uint16_t>(highestIndex + 1) : std::numeric_limits<std::uint16_t>::max();
		nextIndexExhausted = (highestIndex >= std::numeric_limits<std::uint16_t>::max());
	}

	std::cout << "[" << get_timestamp() << "] [FieldRegistry] Loaded " << nameToIndex.size() << " field(s) from " << filePath << std::endl;
}

void FieldRegistry::append_entry(const std::string &fieldName, std::uint16_t index)
{
	std::ofstream out(filePath, std::ios::app);
	if (!out.is_open())
	{
		std::cout << "[" << get_timestamp() << "] [FieldRegistry] Failed to persist field '" << fieldName << "' (could not open " << filePath << " for append)" << std::endl;
		return;
	}
	out << index << ',' << fieldName << '\n';
}

std::uint16_t FieldRegistry::get_or_assign_index(const std::string &fieldName)
{
	auto it = nameToIndex.find(fieldName);
	if (it != nameToIndex.end())
	{
		return it->second;
	}

	if (nextIndexExhausted)
	{
		std::cout << "[" << get_timestamp() << "] [FieldRegistry] Field index space exhausted (65536 fields already registered); "
		          << "reusing the last index for '" << fieldName << "' instead of assigning a new one." << std::endl;
		return nextIndex;
	}

	const std::uint16_t assigned = nextIndex;
	nameToIndex[fieldName] = assigned;
	if (assigned == std::numeric_limits<std::uint16_t>::max())
	{
		nextIndexExhausted = true;
	}
	else
	{
		++nextIndex;
	}

	append_entry(fieldName, assigned);
	std::cout << "[" << get_timestamp() << "] [FieldRegistry] Assigned index " << assigned << " to field '" << fieldName << "'" << std::endl;
	return assigned;
}
