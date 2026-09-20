#pragma once

#include <cstdint>
#include <optional>
#include <vector>

// Measurement subscriptions only. Never enqueue section-control setpoints here.
class MeasurementSubscriptionQueue
{
public:
	enum class Trigger
	{
		OnChange,
		TimeInterval
	};
	struct Command
	{
		std::uint16_t ddi;
		std::uint16_t element;
		Trigger trigger;
		unsigned attempts = 0;
		std::uint32_t lastAttempt = 0;
		bool pending = true;
	};

	void initialize(std::uint32_t now)
	{
		commands.clear();
		initialized = true;
		started = lastSent = now;
	}

	void add(std::uint16_t ddi, std::uint16_t element, Trigger trigger)
	{
		for (const auto &command : commands)
			if (command.ddi == ddi && command.element == element && command.trigger == trigger)
				return;
		commands.push_back({ ddi, element, trigger });
	}

	std::optional<Command> next(std::uint32_t now)
	{
		if (!initialized || elapsed(now, started) < 1000 || elapsed(now, lastSent) < 100)
			return std::nullopt;
		for (auto &command : commands)
		{
			if (!command.pending || command.attempts >= 4)
				continue;
			if (command.attempts != 0 && elapsed(now, command.lastAttempt) < 1000)
				continue;
			command.pending = false;
			command.lastAttempt = now;
			++command.attempts;
			lastSent = now;
			return command;
		}
		return std::nullopt;
	}

	// The installed stack reports the PDACK opcode, not its acknowledged opcode.
	// Retry only already-sent, advertised subscriptions for this exact DDI/element.
	bool rejected(std::uint16_t ddi, std::uint16_t element)
	{
		bool queued = false;
		for (auto &command : commands)
		{
			if (command.ddi == ddi && command.element == element && command.attempts > 0 && command.attempts < 4)
			{
				command.pending = true;
				queued = true;
			}
		}
		return queued;
	}

	void send_failed(const Command &failed)
	{
		for (auto &command : commands)
			if (command.ddi == failed.ddi && command.element == failed.element && command.trigger == failed.trigger && command.attempts < 4)
				command.pending = true;
	}

	void received(std::uint16_t ddi, std::uint16_t element)
	{
		for (auto &command : commands)
			if (command.ddi == ddi && command.element == element && command.attempts > 0)
				command.pending = false;
	}

private:
	static std::uint32_t elapsed(std::uint32_t now, std::uint32_t then)
	{
		return now - then;
	}
	std::vector<Command> commands;
	bool initialized = false;
	std::uint32_t started = 0;
	std::uint32_t lastSent = 0;
};
