#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "underground_world/msg/enemy_down.hpp"
#include "underground_world/msg/local_scan.hpp"
#include "underground_world/msg/move_command.hpp"
#include "underground_world/msg/student_status.hpp"
#include "underground_world/srv/payload_trigger.hpp"

namespace {
using underground_world::msg::LocalScan;
using underground_world::msg::MoveCommand;
using underground_world::msg::StudentStatus;
using underground_world::srv::PayloadTrigger;

constexpr auto kScanTopic = "/robot/local_scan";
constexpr auto kMoveTopic = "/robot/cmd_move";
constexpr auto kStatusTopic = "/student/status";
constexpr auto kTriggerService = "/payload/trigger";
}  // namespace

class MissionExplorerNode final : public rclcpp::Node {
public:
  MissionExplorerNode()
    : Node("mission_explorer_driver_node")
  {
    const auto qos = rclcpp::QoS{10};

    move_pub_ = create_publisher<MoveCommand>(kMoveTopic, qos);
    status_pub_ = create_publisher<StudentStatus>(kStatusTopic, qos);
    trigger_client_ = create_client<PayloadTrigger>(kTriggerService);

    scan_sub_ = create_subscription<LocalScan>(kScanTopic, qos, [this](const LocalScan& msg) { on_scan(msg); });

    publish_status(StudentStatus::EXPLORING);
    RCLCPP_INFO(get_logger(), "mission_explorer ready");
  }

private:
  void on_scan(const LocalScan& scan)
  {
    // TODO(step 4): оновити мапу, обробити контакти, обрати наступний крок.
    RCLCPP_INFO(get_logger(), "scan robot=(%d,%d) cells=%zu", scan.robot_x, scan.robot_y, scan.cells.size());
  }

  void publish_status(std::uint8_t state)
  {
    StudentStatus msg;
    msg.state = state;
    status_pub_->publish(msg);
  }

  rclcpp::Publisher<MoveCommand>::SharedPtr move_pub_;
  rclcpp::Publisher<StudentStatus>::SharedPtr status_pub_;
  rclcpp::Client<PayloadTrigger>::SharedPtr trigger_client_;
  rclcpp::Subscription<LocalScan>::SharedPtr scan_sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionExplorerNode>());
  rclcpp::shutdown();

  return 0;
}