#include <chrono>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "rclcpp/rclcpp.hpp"

#include "mowgli_behavior/action_nodes.hpp"
#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_behavior/condition_nodes.hpp"

#include "mowgli_interfaces/msg/emergency.hpp"
#include "mowgli_interfaces/msg/power.hpp"
#include "mowgli_interfaces/msg/status.hpp"
#include "mowgli_interfaces/srv/high_level_control.hpp"

using namespace std::chrono_literals;

namespace mowgli_behavior {

// ---------------------------------------------------------------------------
// BehaviorTreeNode
// ---------------------------------------------------------------------------

class BehaviorTreeNode : public rclcpp::Node {
public:
  explicit BehaviorTreeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{})
  : rclcpp::Node("mowgli_behavior_node", options)
  {
    // ------------------------------------------------------------------
    // Shared context
    // ------------------------------------------------------------------
    context_ = std::make_shared<BTContext>();
    context_->node = shared_from_this();  // set after construction via init()

    RCLCPP_INFO(get_logger(), "Initializing mowgli_behavior_node");
  }

  /// Must be called once after construction (shared_from_this() is valid).
  void init()
  {
    context_->node = shared_from_this();

    setupSubscribers();
    setupServiceServer();
    setupBehaviorTree();
    setupTimer();

    RCLCPP_INFO(get_logger(), "mowgli_behavior_node ready");
  }

private:
  // ------------------------------------------------------------------
  // ROS2 infrastructure
  // ------------------------------------------------------------------

  void setupSubscribers()
  {
    using namespace mowgli_interfaces::msg;

    status_sub_ = create_subscription<Status>(
      "/hardware_bridge/status", 10,
      [this](const Status::SharedPtr msg) {
        context_->latest_status = *msg;
      });

    emergency_sub_ = create_subscription<Emergency>(
      "/hardware_bridge/emergency", 10,
      [this](const Emergency::SharedPtr msg) {
        context_->latest_emergency = *msg;
      });

    power_sub_ = create_subscription<Power>(
      "/hardware_bridge/power", 10,
      [this](const Power::SharedPtr msg) {
        context_->latest_power = *msg;

        // Derive battery_percent from voltage.
        // Simple linear approximation: 25.2 V = 100 %, 21.0 V = 0 %
        // (4S LiPo: 4 cells × 4.2 V max / 3.5 V cutoff)
        constexpr float V_MAX  = 25.2f;
        constexpr float V_MIN  = 21.0f;
        const float clamped    = std::clamp(msg->v_battery, V_MIN, V_MAX);
        context_->battery_percent =
          100.0f * (clamped - V_MIN) / (V_MAX - V_MIN);
      });

    RCLCPP_DEBUG(get_logger(), "Topic subscribers created");
  }

  void setupServiceServer()
  {
    using HighLevelControl = mowgli_interfaces::srv::HighLevelControl;

    high_level_control_srv_ = create_service<HighLevelControl>(
      "~/high_level_control",
      [this](
        const HighLevelControl::Request::SharedPtr req,
        HighLevelControl::Response::SharedPtr resp)
      {
        RCLCPP_INFO(get_logger(),
          "HighLevelControl: received command=%u", req->command);
        context_->current_command = req->command;
        resp->success = true;
      });

    RCLCPP_DEBUG(get_logger(), "~/high_level_control service server created");
  }

  void setupBehaviorTree()
  {
    // Register all custom nodes
    mowgli_behavior::registerAllNodes(factory_);

    // Resolve tree file path
    std::string tree_file = declare_parameter<std::string>("tree_file", "");

    if (tree_file.empty()) {
      try {
        const std::string pkg_share =
          ament_index_cpp::get_package_share_directory("mowgli_behavior");
        tree_file = pkg_share + "/trees/main_tree.xml";
      } catch (const std::exception& ex) {
        throw std::runtime_error(
          std::string("Cannot locate default tree file: ") + ex.what());
      }
    }

    if (!std::filesystem::exists(tree_file)) {
      throw std::runtime_error("Tree file not found: " + tree_file);
    }

    RCLCPP_INFO(get_logger(), "Loading behavior tree from: %s", tree_file.c_str());

    // Build blackboard and store shared context
    blackboard_ = BT::Blackboard::create();
    blackboard_->set("context", context_);

    tree_ = factory_.createTreeFromFile(tree_file, blackboard_);

    // Attach a console logger so state transitions are visible in the terminal.
    logger_ = std::make_unique<BT::StdCoutLogger>(tree_);

    RCLCPP_INFO(get_logger(), "Behavior tree loaded successfully");
  }

  void setupTimer()
  {
    // 10 Hz tick rate
    tick_timer_ = create_wall_timer(100ms, [this]() { tickTree(); });
  }

  void tickTree()
  {
    try {
      const BT::NodeStatus status = tree_.tickOnce();

      if (status == BT::NodeStatus::FAILURE) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Behavior tree root returned FAILURE");
      }
    } catch (const std::exception& ex) {
      RCLCPP_ERROR(get_logger(), "Exception during tree tick: %s", ex.what());
    }
  }

  // ------------------------------------------------------------------
  // Data members
  // ------------------------------------------------------------------

  std::shared_ptr<BTContext> context_;

  // Subscribers
  rclcpp::Subscription<mowgli_interfaces::msg::Status>::SharedPtr    status_sub_;
  rclcpp::Subscription<mowgli_interfaces::msg::Emergency>::SharedPtr emergency_sub_;
  rclcpp::Subscription<mowgli_interfaces::msg::Power>::SharedPtr     power_sub_;

  // Service server
  rclcpp::Service<mowgli_interfaces::srv::HighLevelControl>::SharedPtr high_level_control_srv_;

  // BehaviorTree.CPP
  BT::BehaviorTreeFactory          factory_;
  BT::Blackboard::Ptr              blackboard_;
  BT::Tree                         tree_;
  std::unique_ptr<BT::StdCoutLogger> logger_;

  // Tick timer
  rclcpp::TimerBase::SharedPtr tick_timer_;
};

}  // namespace mowgli_behavior

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<mowgli_behavior::BehaviorTreeNode>();

  // init() must be called after the node is managed by a shared_ptr so that
  // shared_from_this() is valid.
  node->init();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
