#include "isobus/isobus/can_constants.hpp"
#include "isobus/isobus/can_stack_logger.hpp"
#include "isobus/isobus/isobus_virtual_terminal_objects.hpp"
#include "isobus/isobus/isobus_virtual_terminal_working_set_base.hpp"
#include "isobus/utility/iop_file_interface.hpp"

#include <cstdio>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace
{
	class ObjectPool : public isobus::VirtualTerminalWorkingSetBase
	{
	public:
		bool load(const std::string &path)
		{
			auto data = isobus::IOPFileInterface::read_iop_file(path);
			return !data.empty() && parse_iop_into_objects(data.data(), static_cast<std::uint32_t>(data.size()));
		}
	};

	std::vector<std::string> violations;

	void flag(std::uint16_t objectID, const std::string &reason)
	{
		violations.push_back("object " + std::to_string(objectID) + ": " + reason);
	}

	// The parser repairs malformed attributes as it reads them — an out-of-range OutputNumber format
	// byte just becomes exponential — so the finished object looks clean and its log holds the only
	// evidence. Caveat: all warnings are fatal here, including two routine Auxiliary Type 1 notices.
	class FailOnParserComplaint : public isobus::CANStackLogger
	{
	public:
		void sink_CAN_stack_log(LoggingLevel level, const std::string &logText) override
		{
			if (level >= LoggingLevel::Warning)
			{
				violations.push_back("parser: " + logText);
			}
		}
	};

	// get_is_valid() skips child IDs that resolve to nothing instead of rejecting them, and never
	// examines a WorkingSet's active mask, so those dangling references reach the VT unreported.
	void check_reference(std::uint16_t objectID,
	                     const char *label,
	                     std::uint16_t referencedID,
	                     const std::map<std::uint16_t, std::shared_ptr<isobus::VTObject>> &tree)
	{
		if ((isobus::NULL_OBJECT_ID != referencedID) && (0 == tree.count(referencedID)))
		{
			flag(objectID,
			     std::string(label) + " references object " + std::to_string(referencedID) +
			       " which is not in the pool");
		}
	}

	template<typename T>
	void check_min_max(std::uint16_t objectID, const char *label, const std::shared_ptr<isobus::VTObject> &object)
	{
		auto typed = std::static_pointer_cast<T>(object);
		if ((typed->get_value() < typed->get_min_value()) || (typed->get_value() > typed->get_max_value()))
		{
			flag(objectID,
			     std::string(label) + " value " + std::to_string(typed->get_value()) + " outside [" +
			       std::to_string(typed->get_min_value()) + ", " + std::to_string(typed->get_max_value()) + "]");
		}
	}
}

int main(int argc, char **argv)
{
	if (2 != argc)
	{
		std::fprintf(stderr, "usage: iop_validator <path to .iop>\n");
		return 2;
	}

	FailOnParserComplaint parserLog;
	isobus::CANStackLogger::set_can_stack_logger_sink(&parserLog);

	ObjectPool pool;
	if (!pool.load(argv[1]))
	{
		std::fprintf(stderr, "FAIL: could not read or parse %s\n", argv[1]);
		return 1;
	}

	const auto &tree = pool.get_object_tree();
	for (const auto &entry : tree)
	{
		if (nullptr == entry.second)
		{
			continue;
		}

		if (!entry.second->get_is_valid(tree))
		{
			flag(entry.first, "failed object pool structural validation");
		}

		for (std::uint16_t i = 0; i < entry.second->get_number_children(); i++)
		{
			check_reference(entry.first, "child", entry.second->get_child_id(i), tree);
		}

		if (auto workingSet = std::dynamic_pointer_cast<isobus::WorkingSet>(entry.second))
		{
			check_reference(entry.first, "active mask", workingSet->get_active_mask(), tree);
		}

		switch (entry.second->get_object_type())
		{
			case isobus::VirtualTerminalObjectType::InputBoolean:
			{
				auto typed = std::static_pointer_cast<isobus::InputBoolean>(entry.second);
				if (typed->get_value() > 1)
				{
					flag(entry.first, "InputBoolean value " + std::to_string(typed->get_value()) + " is not 0 or 1");
				}
			}
			break;

			case isobus::VirtualTerminalObjectType::InputList:
			case isobus::VirtualTerminalObjectType::OutputList:
			{
				auto typed = std::static_pointer_cast<isobus::ListVTObject>(entry.second);
				const auto itemCount = typed->get_number_children();
				if ((0 != itemCount) && (0xFF != typed->get_value()) && (typed->get_value() >= itemCount))
				{
					flag(entry.first,
					     "list value " + std::to_string(typed->get_value()) + " selects item beyond the " +
					       std::to_string(itemCount) + " present");
				}
			}
			break;

			case isobus::VirtualTerminalObjectType::InputNumber:
			{
				auto typed = std::static_pointer_cast<isobus::InputNumber>(entry.second);
				if ((typed->get_value() < typed->get_minimum_value()) || (typed->get_value() > typed->get_maximum_value()))
				{
					flag(entry.first,
					     "InputNumber value " + std::to_string(typed->get_value()) + " outside [" +
					       std::to_string(typed->get_minimum_value()) + ", " +
					       std::to_string(typed->get_maximum_value()) + "]");
				}
			}
			break;

			case isobus::VirtualTerminalObjectType::OutputMeter:
				check_min_max<isobus::OutputMeter>(entry.first, "OutputMeter", entry.second);
				break;

			case isobus::VirtualTerminalObjectType::OutputLinearBarGraph:
				check_min_max<isobus::OutputLinearBarGraph>(entry.first, "OutputLinearBarGraph", entry.second);
				break;

			case isobus::VirtualTerminalObjectType::OutputArchedBarGraph:
				check_min_max<isobus::OutputArchedBarGraph>(entry.first, "OutputArchedBarGraph", entry.second);
				break;

			default:
				break;
		}
	}

	if (!violations.empty())
	{
		std::fprintf(stderr, "FAIL: %s\n", argv[1]);
		for (const auto &violation : violations)
		{
			std::fprintf(stderr, "  %s\n", violation.c_str());
		}
		return 1;
	}

	std::printf("OK: %s, %zu objects\n", argv[1], tree.size());
	return 0;
}
