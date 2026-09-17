#include <cassert>
#include <iostream>
#include <limits>
#include "measurement_subscription_queue.hpp"

#ifdef NDEBUG
#error "Measurement queue tests require assertions in every configuration"
#endif

using Q = MeasurementSubscriptionQueue;
int main()
{
	Q q;
	assert(!q.next(5000));
	q.initialize(0);
	q.add(141, 10, Q::Trigger::OnChange);
	q.add(141, 10, Q::Trigger::OnChange);
	q.add(141, 10, Q::Trigger::TimeInterval);
	q.add(141, 11, Q::Trigger::TimeInterval);
	assert(!q.next(999));
	auto first = q.next(1000);
	assert(first && first->element == 10 && first->trigger == Q::Trigger::OnChange && first->attempts == 1);
	assert(!q.next(1099));
	assert(q.rejected(141, 10));
	assert(!q.rejected(141, 999));
	assert(!q.rejected(290, 10));
	auto second = q.next(1100);
	assert(second && second->trigger == Q::Trigger::TimeInterval);
	q.received(141, 10);
	auto third = q.next(1200);
	assert(third && third->element == 11);
	assert(!q.next(6000));

	// A receive cancels retries but never cancels an unsent interval subscription.
	q.initialize(0);
	q.add(141, 10, Q::Trigger::OnChange);
	q.add(141, 10, Q::Trigger::TimeInterval);
	assert(q.next(1000));
	q.received(141, 10);
	assert(q.next(1100)->trigger == Q::Trigger::TimeInterval);

	q.initialize(0);
	q.add(141, 10, Q::Trigger::TimeInterval);
	for (unsigned attempt = 1; attempt <= 4; ++attempt)
	{
		auto command = q.next(attempt * 1000);
		assert(command && command->attempts == attempt);
		assert(q.rejected(141, 10) == (attempt < 4));
		assert(!q.next(attempt * 1000 + 999));
	}
	assert(!q.next(999999));

	q.initialize(0);
	q.add(141, 12, Q::Trigger::TimeInterval);
	auto failed = q.next(1000);
	q.send_failed(*failed);
	assert(!q.next(1999));
	assert(q.next(2000)->attempts == 2);

	const auto start = std::numeric_limits<std::uint32_t>::max() - 500;
	q.initialize(start);
	q.add(141, 10, Q::Trigger::TimeInterval);
	assert(!q.next(start + 999));
	assert(q.next(start + 1000));
	Q independent;
	assert(!independent.rejected(141, 10));
	q.initialize(1234);
	assert(!q.next(6000));
	std::cout << "PASS: delay, pacing, dedup, targeted retry, retry cap, data cancellation, send failure, rollover, reset, isolation\n";
}
