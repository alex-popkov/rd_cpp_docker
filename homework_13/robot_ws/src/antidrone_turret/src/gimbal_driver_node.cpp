#include <cstdint>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "antidrone_turret/msg/gimbal_command.hpp"

using GimbalCommand = antidrone_turret::msg::GimbalCommand;

namespace {

constexpr auto kGimbalTopic = "/gimbal/cmd";

const char* direction_to_string(std::int8_t direction)
{
  switch (direction) {
    case GimbalCommand::UP: {
      return "UP";
    }
    case GimbalCommand::DOWN: {
      return "DOWN";
    }
    case GimbalCommand::CENTER: {
      return "CENTER";
    }
  }

  return "UNKNOWN";
}
}  // namespace

class GimbalDriverNode final : public rclcpp::Node {
public:
  GimbalDriverNode()
    : Node("gimbal_driver_node")
  {
    subscription_ = create_subscription<GimbalCommand>(kGimbalTopic, 10, [this](const GimbalCommand& msg) { on_command(msg); });

    RCLCPP_INFO(get_logger(), "gimbal_driver_node listening on %s", kGimbalTopic);
  }

private:
  void on_command(const GimbalCommand& msg)
  {
    RCLCPP_INFO(get_logger(),
                "gimbal_driver_node got: direction=%s target_y=%.1f error_y=%.1f",
                direction_to_string(msg.direction),
                msg.target_y,
                msg.error_y);
  }

  rclcpp::Subscription<GimbalCommand>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GimbalDriverNode>());
  rclcpp::shutdown();

  return 0;
}