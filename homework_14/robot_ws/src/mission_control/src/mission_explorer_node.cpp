#include <memory>
#include <array>
#include <map>
#include <optional>
#include <set>
#include <utility>

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
using Cell = std::pair<int, int>;

struct Step {
  int dx;
  int dy;
  std::uint8_t dir;
};

constexpr auto kScanTopic = "/robot/local_scan";
constexpr auto kMoveTopic = "/robot/cmd_move";
constexpr auto kStatusTopic = "/student/status";
constexpr auto kTriggerService = "/payload/trigger";

bool is_passable(const std::string& cell_type)
{
  return cell_type == "." || cell_type == "S" || cell_type == "x";
}

std::uint8_t direction_to(const Cell& from, const Cell& to)
{
  const int dx = to.first - from.first;
  const int dy = to.second - from.second;
  if (dx == 1)
    return MoveCommand::RIGHT;
  if (dx == -1)
    return MoveCommand::LEFT;
  if (dy == 1)
    return MoveCommand::DOWN;
  return MoveCommand::UP;  // dy == -1
}

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
    if (done_) {
      return;
    }

    const Cell robot{scan.robot_x, scan.robot_y};
    bool active_contact_visible = false;

    for (const auto& cell : scan.cells) {
      known_[{cell.x, cell.y}] = cell.cell_type;

      if (cell.cell_type == "S") {
        start_ = Cell{cell.x, cell.y};
      }
      if (cell.cell_type == "C") {
        active_contact_visible = true;
      }
    }

    if (active_contact_visible) {
      for (const auto& cell : scan.cells) {
        if (cell.cell_type != "C") {
          continue;
        }
        if (engaged_ids_.insert(cell.contact_id).second) {
          send_trigger(cell.contact_id, cell.x, cell.y);
        }
      }
      publish_status(StudentStatus::ENGAGING);

      return;
    }

    RCLCPP_INFO(get_logger(), "scan robot=(%d,%d) cells=%zu", scan.robot_x, scan.robot_y, scan.cells.size());
  }

  void publish_status(std::uint8_t state)
  {
    StudentStatus msg;
    msg.state = state;
    status_pub_->publish(msg);
  }

  void send_trigger(int contact_id, int x, int y)
  {
    auto request = std::make_shared<PayloadTrigger::Request>();
    request->contact_id = contact_id;
    request->x = x;
    request->y = y;

    // Асинхронно, БЕЗ блокуючого очікування відповіді.
    trigger_client_->async_send_request(request, [this, contact_id](rclcpp::Client<PayloadTrigger>::SharedFuture future) {
      const auto response = future.get();
      RCLCPP_INFO(get_logger(),
                  "trigger contact_id=%d accepted=%s reason=%s",
                  contact_id,
                  response->accepted ? "true" : "false",
                  response->reason.c_str());
    });
  }

  rclcpp::Publisher<MoveCommand>::SharedPtr move_pub_;
  rclcpp::Publisher<StudentStatus>::SharedPtr status_pub_;
  rclcpp::Client<PayloadTrigger>::SharedPtr trigger_client_;
  rclcpp::Subscription<LocalScan>::SharedPtr scan_sub_;
  std::map<Cell, std::string> known_;
  std::map<Cell, Cell> parent_;
  std::set<Cell> visited_;
  std::set<int> engaged_ids_;
  std::optional<Cell> start_;
  bool done_ = false;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionExplorerNode>());
  rclcpp::shutdown();

  return 0;
}