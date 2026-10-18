/**
 * @brief Works out how to switch an implement's sections from its DDOP
 *
 * Kept free of the TC server and the CAN stack, so tests can run it against stored DDOPs.
 */

#pragma once

#include "isobus/isobus/isobus_device_descriptor_object_pool.hpp"
#include "isobus/isobus/isobus_device_descriptor_object_pool_helpers.hpp"

#include <cstdint>
#include <map>
#include <vector>

/// The section control methods, in the order the TC prefers them
enum class SectionControlMethod : std::uint8_t
{
	None, ///< The TC cannot switch the sections
	CondensedSetpoint, ///< DDI 290+ (Setpoint Condensed Work State), actual states on DDI 161+
	SettableCondensedActual, ///< DDI 161+ (Actual Condensed Work State) written as the setpoint
	PerElementSetpointWorkState, ///< DDI 289 on each section element, actual state on DDI 141 of the same element
	PerElementActualWorkState ///< DDI 141 on each section element written as the setpoint
};

/// @brief Short name of a method, as used in logs and in tools/testdata/implements/manifest.csv
const char *to_string(SectionControlMethod method);

/// @brief What to write, and to which element, to switch an implement's sections
struct SectionControlPlan
{
	SectionControlMethod method = SectionControlMethod::None;

	/// Condensed methods: element number per condensed DDI (290+ or 161+), one entry per group of 16 sections
	std::map<std::uint16_t, std::uint16_t> condensedTargets;

	/// Per-element methods: the setpoint DDI (289 or 141) and the element to write it to for each
	/// section index. isobus::NULL_OBJECT_ID where that section does not link a settable setpoint.
	std::uint16_t perElementSetpointDDI = 0;
	std::vector<std::uint16_t> perElementTargets;

	/// Global Setpoint Work State (DDI 289) on an element that is not a section, if the implement has one
	bool hasSetpointWorkState = false;
	std::uint16_t setpointWorkStateElement = 0;

	bool uses_per_element_control() const;
};

/// @brief Flattens the implement geometry into section element numbers, in AOG's section order
std::vector<std::uint16_t> get_section_element_numbers(const isobus::DeviceDescriptorObjectPoolHelper::Implement &implement);

/// @brief Picks the section control method and its targets.
/// Only process data that a device element links counts: some implements send every object of their
/// template and leave the unused ones unreferenced.
/// @param[in] pool The client's deserialized DDOP
/// @param[in] sectionElementNumbers The section element numbers in AOG's section order
SectionControlPlan resolve_section_control(isobus::DeviceDescriptorObjectPool &pool, const std::vector<std::uint16_t> &sectionElementNumbers);
