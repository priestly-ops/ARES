#ifndef ARES_SIMULATION__WEEK6_CLOCK_BOUNDARY_HPP_
#define ARES_SIMULATION__WEEK6_CLOCK_BOUNDARY_HPP_

#include <chrono>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <optional>
#include <stdexcept>

#include "rosgraph_msgs/msg/clock.hpp"

namespace ares_simulation
{

inline std::chrono::nanoseconds clock_output_period(double output_rate_hz)
{
  if (!std::isfinite(output_rate_hz) || !(output_rate_hz > 0.0)) {
    throw std::invalid_argument("output_rate_hz must be finite and positive");
  }
  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / output_rate_hz));
  if (period.count() <= 0) {
    throw std::invalid_argument("output_rate_hz is too high");
  }
  return period;
}

class ClockBoundaryState
{
public:
  bool ingest(const rosgraph_msgs::msg::Clock & message)
  {
    const auto stamp_ns = to_nanoseconds(message);
    std::lock_guard<std::mutex> lock(mutex_);
    ++input_count_;
    const bool backward =
      (last_published_stamp_ns_ && stamp_ns < *last_published_stamp_ns_) ||
      (latest_stamp_ns_ && stamp_ns < *latest_stamp_ns_);
    if (backward) {
      ++backward_drop_count_;
      return false;
    }
    if (last_published_stamp_ns_ && stamp_ns == *last_published_stamp_ns_) {
      ++duplicate_drop_count_;
      return false;
    }
    latest_ = message;
    latest_stamp_ns_ = stamp_ns;
    return true;
  }

  std::optional<rosgraph_msgs::msg::Clock> poll()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!latest_) {
      return std::nullopt;
    }
    const auto output = latest_;
    last_published_stamp_ns_ = latest_stamp_ns_;
    latest_.reset();
    latest_stamp_ns_.reset();
    ++output_count_;
    return output;
  }

  std::uint64_t input_count() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return input_count_;
  }

  std::uint64_t output_count() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_count_;
  }

  std::uint64_t backward_drop_count() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return backward_drop_count_;
  }

  std::uint64_t duplicate_drop_count() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return duplicate_drop_count_;
  }

  std::optional<std::int64_t> last_published_stamp_ns() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_published_stamp_ns_;
  }

private:
  static std::int64_t to_nanoseconds(
    const rosgraph_msgs::msg::Clock & message)
  {
    if (message.clock.nanosec >= 1000000000U) {
      throw std::invalid_argument("clock nanosec must be below one second");
    }
    return static_cast<std::int64_t>(message.clock.sec) * 1000000000LL +
           static_cast<std::int64_t>(message.clock.nanosec);
  }

  mutable std::mutex mutex_;
  std::optional<rosgraph_msgs::msg::Clock> latest_;
  std::optional<std::int64_t> latest_stamp_ns_;
  std::optional<std::int64_t> last_published_stamp_ns_;
  std::uint64_t input_count_{0};
  std::uint64_t output_count_{0};
  std::uint64_t backward_drop_count_{0};
  std::uint64_t duplicate_drop_count_{0};
};

}  // namespace ares_simulation

#endif  // ARES_SIMULATION__WEEK6_CLOCK_BOUNDARY_HPP_
