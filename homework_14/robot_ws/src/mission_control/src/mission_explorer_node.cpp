#include <memory>
#include <array>
#include <map>
#include <optional>
#include <set>
#include <utility>
#include <vector>
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

constexpr std::array<Step, 4> kSteps = {{
  {0, -1, MoveCommand::UP},
  {0, 1, MoveCommand::DOWN},
  {-1, 0, MoveCommand::LEFT},
  {1, 0, MoveCommand::RIGHT},
}};

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
  if (dx == 1) {
    return MoveCommand::RIGHT;
  }

  if (dx == -1) {
    return MoveCommand::LEFT;
  }

  if (dy == 1) {
    return MoveCommand::DOWN;
  }

  return MoveCommand::UP;
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

    startup_timer_ = create_wall_timer(std::chrono::milliseconds{1000}, [this]() { kick_if_no_scan(); });

    publish_status(StudentStatus::EXPLORING);
    RCLCPP_INFO(get_logger(), "mission_explorer ready");
  }

private:
  void on_scan(const LocalScan& scan)
  {
    if (done_) {
      return;
    }

    started_ = true;

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

    // dfs
    visited_.insert(robot);
    if (path_stack_.empty()) {
      path_stack_.push_back(robot);
    }

    // 1) Спроба піти "вглиб": перший невідвіданий прохідний сусід.
    for (const auto& step : kSteps) {
      const Cell next{robot.first + step.dx, robot.second + step.dy};
      if (visited_.count(next)) {
        continue;
      }
      const auto it = known_.find(next);
      if (it == known_.end() || !is_passable(it->second)) {
        continue;
      }
      path_stack_.push_back(next);
      publish_status(StudentStatus::EXPLORING);
      publish_move(step.dir);

      return;
    }

    // 2) Глухий кут -> backtrack: знімаємо поточну клітинку зі стека.
    path_stack_.pop_back();
    if (path_stack_.empty()) {
      // Зняли стартову і йти нема куди => все досліджено, і ми вже в S.
      publish_status(StudentStatus::DONE);
      done_ = true;
      RCLCPP_INFO(get_logger(), "exploration complete, back at start");

      return;
    }

    publish_status(StudentStatus::RETURNING);
    publish_move(direction_to(robot, path_stack_.back()));

    RCLCPP_INFO(get_logger(), "scan robot=(%d,%d) cells=%zu", scan.robot_x, scan.robot_y, scan.cells.size());
  }

  void publish_status(std::uint8_t state)
  {
    StudentStatus msg;
    msg.state = state;
    status_pub_->publish(msg);
  }

  void publish_move(std::uint8_t direction)
  {
    MoveCommand msg;
    msg.direction = direction;
    move_pub_->publish(msg);
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

  void kick_if_no_scan()
  {
    if (started_) {
      startup_timer_->cancel();
      return;
    }
    MoveCommand msg;
    msg.direction = 255;
    move_pub_->publish(msg);
    RCLCPP_WARN(get_logger(), "no initial scan yet, sending wake-up kick");
  }

  rclcpp::Publisher<MoveCommand>::SharedPtr move_pub_;
  rclcpp::Publisher<StudentStatus>::SharedPtr status_pub_;
  rclcpp::Client<PayloadTrigger>::SharedPtr trigger_client_;
  rclcpp::Subscription<LocalScan>::SharedPtr scan_sub_;
  std::map<Cell, std::string> known_;
  std::vector<Cell> path_stack_;
  std::set<Cell> visited_;
  std::set<int> engaged_ids_;
  std::optional<Cell> start_;
  bool done_ = false;
  bool started_ = false;
  rclcpp::TimerBase::SharedPtr startup_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionExplorerNode>());
  rclcpp::shutdown();

  return 0;
}