#include <cstdint>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "antidrone_turret/turret_controller.hpp"

#include "antidrone_turret/msg/target.hpp"
#include "antidrone_turret/msg/actuator_status.hpp"
#include "antidrone_turret/msg/turret_status.hpp"
#include "antidrone_turret/msg/gimbal_command.hpp"
#include "antidrone_turret/msg/servo_command.hpp"
#include "antidrone_turret/srv/trigger_actuator.hpp"

using Turret = antidrone_turret::msg::TurretStatus;

namespace {

constexpr auto kTargetTopic = "/perception/target";
constexpr auto kActuatorStatusTopic = "/actuator/status";
constexpr auto kTurretStatusTopic = "/turret/status";
constexpr auto kGimbalTopic = "/gimbal/cmd";
constexpr auto kServoTopic = "/servo/cmd";
constexpr auto kTriggerService = "/actuator/trigger";

std::int8_t to_msg_direction(turret_controller::ServoDirections direction)
{
  using Servo = antidrone_turret::msg::ServoCommand;
  switch (direction) {
    case turret_controller::ServoDirections::LEFT: {
      return Servo::LEFT;
    }
    case turret_controller::ServoDirections::RIGHT: {
      return Servo::RIGHT;
    }
    case turret_controller::ServoDirections::CENTER: {
      return Servo::CENTER;
    }
  }

  return Servo::CENTER;
}

std::int8_t to_msg_direction(turret_controller::GimbalDirections direction)
{
  using Gimbal = antidrone_turret::msg::GimbalCommand;
  switch (direction) {
    case turret_controller::GimbalDirections::UP: {
      return Gimbal::UP;
    }
    case turret_controller::GimbalDirections::DOWN: {
      return Gimbal::DOWN;
    }
    case turret_controller::GimbalDirections::CENTER: {
      return Gimbal::CENTER;
    }
  }

  return Gimbal::CENTER;
}

std::uint8_t to_msg_target_state(turret_controller::TargetStates state)
{
  switch (state) {
    case turret_controller::TargetStates::TARGET_NONE: {
      return Turret::TARGET_NONE;
    }
    case turret_controller::TargetStates::TARGET_LOW_CONFIDENCE: {
      return Turret::TARGET_LOW_CONFIDENCE;
    }
    case turret_controller::TargetStates::TARGET_LOCKED: {
      return Turret::TARGET_LOCKED;
    }
  }

  return Turret::TARGET_NONE;
}

std::uint8_t to_msg_action(turret_controller::ActionStates action)
{
  switch (action) {
    case turret_controller::ActionStates::ACTION_IDLE: {
      return Turret::ACTION_IDLE;
    }
    case turret_controller::ActionStates::ACTION_TRACK: {
      return Turret::ACTION_TRACK;
    }
  }
  return Turret::ACTION_IDLE;
}

std::uint8_t to_msg_trigger_state(turret_controller::TriggerStates state)
{
  switch (state) {
    case turret_controller::TriggerStates::TRIGGER_SKIP: {
      return Turret::TRIGGER_SKIP;
    }
    case turret_controller::TriggerStates::TRIGGER_REQUESTED: {
      return Turret::TRIGGER_REQUESTED;
    }
    case turret_controller::TriggerStates::TRIGGER_RELOADING: {
      return Turret::TRIGGER_RELOADING;
    }
  }

  return Turret::TRIGGER_SKIP;
}
}  // namespace

class TurretControllerNode final : public rclcpp::Node {
  using Target = antidrone_turret::msg::Target;
  using ActuatorStatus = antidrone_turret::msg::ActuatorStatus;
  using TurretStatusMsg = antidrone_turret::msg::TurretStatus;
  using GimbalCommandMsg = antidrone_turret::msg::GimbalCommand;
  using ServoCommandMsg = antidrone_turret::msg::ServoCommand;
  using TriggerActuator = antidrone_turret::srv::TriggerActuator;

public:
  TurretControllerNode()
    : Node("turret_controller_node")
  {
    confidence_threshold_ = declare_parameter<double>("confidence_threshold", 0.80);
    max_distance_m_ = declare_parameter<double>("max_distance_m", 30.0);

    turret_status_pub_ = create_publisher<TurretStatusMsg>(kTurretStatusTopic, 10);
    gimbal_pub_ = create_publisher<GimbalCommandMsg>(kGimbalTopic, 10);
    servo_pub_ = create_publisher<ServoCommandMsg>(kServoTopic, 10);

    trigger_client_ = create_client<TriggerActuator>(kTriggerService);

    actuator_status_sub_ =
      create_subscription<ActuatorStatus>(kActuatorStatusTopic, 10, [this](const ActuatorStatus& msg) { on_actuator_status(msg); });

    target_sub_ = create_subscription<Target>(kTargetTopic, 10, [this](const Target& msg) { on_target(msg); });

    RCLCPP_INFO(get_logger(), "t ready: confidence_threshold=%.2f max_distance_m=%.1f", confidence_threshold_, max_distance_m_);
  }

private:
  void on_actuator_status(const ActuatorStatus& msg)
  {
    this->actuator_ready_ = msg.state == ActuatorStatus::READY;
    if (this->actuator_ready_) {
      this->trigger_pending_ = false;
    }
  }

  void on_target(const Target& msg)
  {
    const auto actuator_state = (this->actuator_ready_ && !this->trigger_pending_) ? turret_controller::ActuatorStatuses::READY
                                                                                   : turret_controller::ActuatorStatuses::RELOADING;

    const auto status = turret_controller::build_turret_status(msg.visible,
                                                               msg.confidence,
                                                               msg.distance_m,
                                                               static_cast<float>(this->confidence_threshold_),
                                                               static_cast<float>(this->max_distance_m_),
                                                               actuator_state);

    publish_turret_status(status);

    if (status.action == turret_controller::ActionStates::ACTION_TRACK) {
      this->publish_servo(turret_controller::make_servo_command(msg.x));
      this->publish_gimbal(turret_controller::make_gimbal_command(msg.y));
    }

    if (status.trigger_state == turret_controller::TriggerStates::TRIGGER_REQUESTED) {
      this->request_trigger(msg.confidence, msg.distance_m);
    }
  }

  void publish_turret_status(const turret_controller::TurretStatus& status)
  {
    auto msg = TurretStatusMsg{};
    msg.target_state = to_msg_target_state(status.target_state);
    msg.action = to_msg_action(status.action);
    msg.trigger_state = to_msg_trigger_state(status.trigger_state);
    msg.confidence = status.confidence;
    msg.distance_m = status.distance_m;
    turret_status_pub_->publish(msg);
  }

  void publish_servo(const turret_controller::ServoCommand& cmd)
  {
    auto msg = ServoCommandMsg{};
    msg.direction = to_msg_direction(cmd.direction);
    msg.target_x = cmd.target_x;
    msg.error_x = cmd.error_x;
    servo_pub_->publish(msg);
  }

  void publish_gimbal(const turret_controller::GimbalCommand& cmd)
  {
    auto msg = GimbalCommandMsg{};
    msg.direction = to_msg_direction(cmd.direction);
    msg.target_y = cmd.target_y;
    msg.error_y = cmd.error_y;
    gimbal_pub_->publish(msg);
  }

  void request_trigger(float confidence, float distance_m)
  {
    if (!this->trigger_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "trigger service not available yet, skipping request");
      return;
    }

    this->trigger_pending_ = true;

    auto request = std::make_shared<TriggerActuator::Request>();
    request->confidence = confidence;
    request->distance_m = distance_m;

    this->trigger_client_->async_send_request(request, [this](rclcpp::Client<TriggerActuator>::SharedFuture future) {
      const auto response = future.get();
      RCLCPP_INFO(
        get_logger(), "trigger response accepted=%s trigger_count=%u", response->accepted ? "true" : "false", response->trigger_count);
    });
  }

  double confidence_threshold_{0.80};
  double max_distance_m_{30.0};
  bool actuator_ready_{false};
  bool trigger_pending_{false};

  rclcpp::Publisher<TurretStatusMsg>::SharedPtr turret_status_pub_;
  rclcpp::Publisher<GimbalCommandMsg>::SharedPtr gimbal_pub_;
  rclcpp::Publisher<ServoCommandMsg>::SharedPtr servo_pub_;
  rclcpp::Client<TriggerActuator>::SharedPtr trigger_client_;
  rclcpp::Subscription<ActuatorStatus>::SharedPtr actuator_status_sub_;
  rclcpp::Subscription<Target>::SharedPtr target_sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurretControllerNode>());
  rclcpp::shutdown();

  return 0;
}