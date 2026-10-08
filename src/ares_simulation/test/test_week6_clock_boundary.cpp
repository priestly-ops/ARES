#include <chrono>
#include <limits>
#include <stdexcept>

#include "ares_simulation/week6_clock_boundary.hpp"
#include "gtest/gtest.h"

namespace
{

rosgraph_msgs::msg::Clock clock_message(std::int32_t sec, std::uint32_t nanosec)
{
  rosgraph_msgs::msg::Clock message;
  message.clock.sec = sec;
  message.clock.nanosec = nanosec;
  return message;
}

using ares_simulation::ClockBoundaryState;
using ares_simulation::clock_output_period;
using namespace std::chrono_literals;

TEST(ClockBoundaryState, RejectsInvalidRate)
{
  EXPECT_THROW(clock_output_period(0.0), std::invalid_argument);
  EXPECT_THROW(clock_output_period(-1.0), std::invalid_argument);
  EXPECT_THROW(
    clock_output_period(std::numeric_limits<double>::infinity()),
    std::invalid_argument);
  EXPECT_EQ(clock_output_period(100.0), 10ms);
}

TEST(ClockBoundaryState, TimerTickForwardsLatestGenuineTimestamp)
{
  ClockBoundaryState state;
  ASSERT_TRUE(state.ingest(clock_message(1, 1000000)));
  auto first = state.poll();
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(first->clock.sec, 1);
  EXPECT_EQ(first->clock.nanosec, 1000000U);

  ASSERT_TRUE(state.ingest(clock_message(1, 2000000)));
  ASSERT_TRUE(state.ingest(clock_message(1, 9000000)));
  auto second = state.poll();
  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(second->clock.sec, 1);
  EXPECT_EQ(second->clock.nanosec, 9000000U);
  EXPECT_EQ(state.output_count(), 2U);
}

TEST(ClockBoundaryState, NeverPublishesDuplicateOrBackwardTime)
{
  ClockBoundaryState state;
  ASSERT_TRUE(state.ingest(clock_message(5, 0)));
  ASSERT_TRUE(state.poll().has_value());

  EXPECT_FALSE(state.ingest(clock_message(5, 0)));
  EXPECT_FALSE(state.ingest(clock_message(4, 999000000)));
  EXPECT_FALSE(state.poll().has_value());
  EXPECT_EQ(state.duplicate_drop_count(), 1U);
  EXPECT_EQ(state.backward_drop_count(), 1U);

  ASSERT_TRUE(state.ingest(clock_message(5, 10000000)));
  auto resumed = state.poll();
  ASSERT_TRUE(resumed.has_value());
  EXPECT_EQ(resumed->clock.sec, 5);
  EXPECT_EQ(resumed->clock.nanosec, 10000000U);
}

TEST(ClockBoundaryState, RejectsRegressionBeforeFirstPublish)
{
  ClockBoundaryState state;
  ASSERT_TRUE(state.ingest(clock_message(2, 20000000)));
  EXPECT_FALSE(state.ingest(clock_message(2, 19000000)));
  auto output = state.poll();
  ASSERT_TRUE(output.has_value());
  EXPECT_EQ(output->clock.nanosec, 20000000U);
  EXPECT_EQ(state.backward_drop_count(), 1U);
}

TEST(ClockBoundaryState, HundredHertzTicksBoundHighRateInput)
{
  ClockBoundaryState state;
  std::uint64_t outputs = 0;
  for (std::uint32_t index = 0; index <= 500; ++index) {
    const auto elapsed = std::chrono::milliseconds(index * 2U);
    const auto elapsed_ms = static_cast<std::uint32_t>(elapsed.count());
    ASSERT_TRUE(state.ingest(clock_message(
        static_cast<std::int32_t>(elapsed_ms / 1000U),
        (elapsed_ms % 1000U) * 1000000U)));
    if (index % 5U == 0U) {
      outputs += state.poll().has_value() ? 1U : 0U;
    }
  }
  // The initial genuine sample is forwarded immediately, followed by one
  // sample per 10 ms. The 100 intervals in one second are exactly 100 Hz.
  EXPECT_EQ(outputs, 101U);
  EXPECT_EQ(state.output_count(), outputs);
}

}  // namespace
