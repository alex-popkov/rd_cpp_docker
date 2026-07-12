#include <cstdint>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "antidrone_turret/msg/servo_command.hpp"

using ServoCommand = antidrone_turret::msg::ServoCommand;

namespace {
constexpr auto kServoTopic = "/servo/cmd";

const char* direction_to_string(std::int8_t direction)
{
  switch (direction) {
    case ServoCommand::LEFT: {
      return "LEFT";
    }
    case ServoCommand::RIGHT: {
      return "RIGHT";
    }
    case ServoCommand::CENTER: {
      return "CENTER";
    }
  }

  return "UNKNOWN";
}
}  // namespace

class YawServoDriverNode final : public rclcpp::Node {
public:
  YawServoDriverNode()
    : Node("yaw_servo_driver_node")
  {
    subscription_ = create_subscription<ServoCommand>(kServoTopic, 10, [this](const ServoCommand& msg) { on_command(msg); });

    RCLCPP_INFO(get_logger(), "yaw_servo_driver_node listening on %s", kServoTopic);
  }

private:
  void on_command(const ServoCommand& msg)
  {
    RCLCPP_INFO(get_logger(),
                "yaw_servo_driver_node got: direction=%s target_x=%.1f error_x=%.1f",
                direction_to_string(msg.direction),
                msg.target_x,
                msg.error_x);
  }

  rclcpp::Subscription<ServoCommand>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<YawServoDriverNode>());
  rclcpp::shutdown();

  return 0;
}