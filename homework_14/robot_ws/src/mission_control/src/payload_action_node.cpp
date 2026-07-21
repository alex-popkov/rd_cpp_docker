#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "underground_world/msg/enemy_down.hpp"
#include "underground_world/srv/payload_trigger.hpp"

namespace {
using underground_world::msg::EnemyDown;
using underground_world::srv::PayloadTrigger;

constexpr auto kTriggerService = "/payload/trigger";
constexpr auto kEnemyDownTopic = "/payload/enemy_down";
}  // namespace

class PayloadActionNode final : public rclcpp::Node {
public:
  PayloadActionNode()
    : Node("payload_action_driver_node")
  {
    const auto qos = rclcpp::QoS{10};
    enemy_down_pub_ = create_publisher<EnemyDown>(kEnemyDownTopic, qos);

    service_ = create_service<PayloadTrigger>(
      kTriggerService, [this](const std::shared_ptr<PayloadTrigger::Request> request, std::shared_ptr<PayloadTrigger::Response> response) {
        on_trigger(request, response);
      });

    RCLCPP_INFO(get_logger(), "payload_action ready, serving %s", kTriggerService);
  }

private:
  void on_trigger(const std::shared_ptr<PayloadTrigger::Request> request, std::shared_ptr<PayloadTrigger::Response> response)
  {
    EnemyDown msg;
    msg.contact_id = request->contact_id;
    msg.x = request->x;
    msg.y = request->y;
    enemy_down_pub_->publish(msg);

    response->accepted = true;
    response->reason = "engaged";

    RCLCPP_INFO(get_logger(), "trigger contact_id=%d at (%d,%d) -> enemy_down published", request->contact_id, request->x, request->y);
  }

  rclcpp::Publisher<EnemyDown>::SharedPtr enemy_down_pub_;
  rclcpp::Service<PayloadTrigger>::SharedPtr service_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PayloadActionNode>());
  rclcpp::shutdown();

  return 0;
}
