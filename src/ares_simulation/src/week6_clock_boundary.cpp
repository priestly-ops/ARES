#include <chrono>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include "ares_simulation/week6_clock_boundary.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "std_msgs/msg/string.hpp"

namespace ares_simulation
{

class Week6ClockBoundary : public rclcpp::Node
{
public:
  Week6ClockBoundary()
  : Node("week6_clock_boundary"),
    input_topic_(declare_parameter<std::string>(
        "input_topic", "/ares/week6/raw_clock")),
    output_topic_(declare_parameter<std::string>("output_topic", "/clock")),
    status_topic_(declare_parameter<std::string>(
        "status_topic", "/ares/week6/clock_boundary_status")),
    output_rate_hz_(declare_parameter<double>("output_rate_hz", 100.0)),
    started_(std::chrono::steady_clock::now())
  {
    if (input_topic_ == output_topic_) {
      throw std::invalid_argument(
              "input_topic and output_topic must differ to prevent duplicate /clock ownership");
    }
    publisher_ = create_publisher<rosgraph_msgs::msg::Clock>(
      output_topic_, rclcpp::ClockQoS());
    subscription_ = create_subscription<rosgraph_msgs::msg::Clock>(
      input_topic_, rclcpp::ClockQoS(),
      [this](const rosgraph_msgs::msg::Clock::SharedPtr message) {
        state_.ingest(*message);
    });
    publish_timer_ = create_wall_timer(clock_output_period(output_rate_hz_), [this]() {
      const auto output = state_.poll();
      if (output) {
        publisher_->publish(*output);
      }
    });
    status_publisher_ = create_publisher<std_msgs::msg::String>(status_topic_, 1);
    status_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
      publish_status();
    });
    RCLCPP_INFO(
      get_logger(),
      "Week 6 clock boundary: %s -> %s, maximum %.3f Hz",
      input_topic_.c_str(), output_topic_.c_str(), output_rate_hz_);
  }

  ~Week6ClockBoundary() override
  {
    const auto elapsed = elapsed_seconds();
    RCLCPP_INFO(
      get_logger(),
      "Week 6 clock boundary stopped: input=%llu output=%llu "
      "backward_drops=%llu duplicate_drops=%llu elapsed=%.3fs",
      static_cast<unsigned long long>(state_.input_count()),
      static_cast<unsigned long long>(state_.output_count()),
      static_cast<unsigned long long>(state_.backward_drop_count()),
      static_cast<unsigned long long>(state_.duplicate_drop_count()), elapsed);
  }

private:
  double elapsed_seconds() const
  {
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now() - started_).count();
  }

  void publish_status()
  {
    const auto elapsed = elapsed_seconds();
    const auto inputs = state_.input_count();
    const auto outputs = state_.output_count();
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(6)
           << "{\"input_topic\":\"" << input_topic_
           << "\",\"output_topic\":\"" << output_topic_
           << "\",\"target_rate_hz\":" << output_rate_hz_
           << ",\"source_mean_hz\":"
           << (elapsed > 0.0 ? static_cast<double>(inputs) / elapsed : 0.0)
           << ",\"output_mean_hz\":"
           << (elapsed > 0.0 ? static_cast<double>(outputs) / elapsed : 0.0)
           << ",\"input_count\":" << inputs
           << ",\"output_count\":" << outputs
           << ",\"backward_drop_count\":" << state_.backward_drop_count()
           << ",\"duplicate_drop_count\":" << state_.duplicate_drop_count();
    const auto stamp = state_.last_published_stamp_ns();
    if (stamp) {
      stream << ",\"last_published_stamp_ns\":" << *stamp;
    } else {
      stream << ",\"last_published_stamp_ns\":null";
    }
    stream << '}';
    std_msgs::msg::String status;
    status.data = stream.str();
    status_publisher_->publish(status);
  }

  const std::string input_topic_;
  const std::string output_topic_;
  const std::string status_topic_;
  const double output_rate_hz_;
  ClockBoundaryState state_;
  const std::chrono::steady_clock::time_point started_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr publisher_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace ares_simulation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ares_simulation::Week6ClockBoundary>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("week6_clock_boundary"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
