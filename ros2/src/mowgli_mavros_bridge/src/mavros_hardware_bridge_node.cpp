#include "mavros_hardware_bridge_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
namespace mowgli_mavros_bridge
{

using namespace std::chrono_literals;

MavrosHardwareBridgeNode::MavrosHardwareBridgeNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("hardware_bridge", options)
{
  status_publish_rate_hz_ = declare_parameter<double>("status_publish_rate_hz", 10.0);
  manual_control_enabled_ = declare_parameter<bool>("manual_control_enabled", false);
  manual_control_linear_scale_ = declare_parameter<double>("manual_control_linear_scale", 1000.0);
  manual_control_yaw_scale_ = declare_parameter<double>("manual_control_yaw_scale", 1000.0);
  blade_control_enabled_ = declare_parameter<bool>("blade_control_enabled", false);
  charging_feedback_enabled_ = declare_parameter<bool>("charging_feedback_enabled", false);
  emergency_mode_ = declare_parameter<std::string>("emergency_mode", "HOLD");
  emergency_disarm_ = declare_parameter<bool>("emergency_disarm", true);
  rain_detected_ = declare_parameter<bool>("rain_detected_default", false);
  esc_power_ = declare_parameter<bool>("esc_power_default", true);
  raspberry_pi_power_ = declare_parameter<bool>("raspberry_pi_power_default", true);

  create_publishers();
  create_subscriptions();
  create_services();
  create_clients();
  create_timers();

  if (!manual_control_enabled_)
  {
    RCLCPP_WARN(get_logger(),
                "manual_control_enabled=false: /cmd_vel commands will be ignored until MAVROS "
                "manual control mapping is validated.");
  }
  if (!blade_control_enabled_)
  {
    RCLCPP_WARN(
        get_logger(),
        "blade_control_enabled=false: mower_control remains provisional and will report failure.");
  }
  if (!charging_feedback_enabled_)
  {
    RCLCPP_WARN(get_logger(),
                "charging_feedback_enabled=false: charging-related status/power fields will remain "
                "conservative.");
  }

  RCLCPP_INFO(get_logger(), "MAVROS hardware bridge started.");
}

void MavrosHardwareBridgeNode::create_publishers()
{
  pub_status_ = create_publisher<mowgli_interfaces::msg::Status>(
      "~/status", rclcpp::SystemDefaultsQoS());
  pub_emergency_ = create_publisher<mowgli_interfaces::msg::Emergency>(
      "~/emergency", rclcpp::SystemDefaultsQoS());
  pub_power_ = create_publisher<mowgli_interfaces::msg::Power>(
      "~/power", rclcpp::SystemDefaultsQoS());
  pub_imu_ = create_publisher<sensor_msgs::msg::Imu>(
      "~/imu/data_raw", rclcpp::SensorDataQoS());
  pub_wheel_odom_ = create_publisher<nav_msgs::msg::Odometry>(
      "~/wheel_odom", rclcpp::SystemDefaultsQoS());
  pub_battery_state_ =
      create_publisher<sensor_msgs::msg::BatteryState>("/battery_state", rclcpp::SensorDataQoS());

  pub_manual_control_ =
      create_publisher<mavros_msgs::msg::ManualControl>("/mavros/manual_control/send", 10);
}

void MavrosHardwareBridgeNode::create_subscriptions()
{
  auto default_qos = rclcpp::SystemDefaultsQoS();
  auto sensor_qos = rclcpp::SensorDataQoS();

  sub_cmd_vel_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "~/cmd_vel",
      default_qos,
      std::bind(&MavrosHardwareBridgeNode::on_cmd_vel, this, std::placeholders::_1));

  sub_hl_status_ = create_subscription<mowgli_interfaces::msg::HighLevelStatus>(
      "/behavior_tree_node/high_level_status",
      default_qos,
      std::bind(&MavrosHardwareBridgeNode::on_high_level_status, this, std::placeholders::_1));

  sub_mavros_state_ = create_subscription<mavros_msgs::msg::State>(
      "/mavros/state",
      default_qos,
      std::bind(&MavrosHardwareBridgeNode::on_mavros_state, this, std::placeholders::_1));

  sub_mavros_imu_ = create_subscription<sensor_msgs::msg::Imu>(
      "/mavros/imu/data",
      sensor_qos,
      std::bind(&MavrosHardwareBridgeNode::on_mavros_imu, this, std::placeholders::_1));

  sub_mavros_battery_ = create_subscription<sensor_msgs::msg::BatteryState>(
      "/mavros/battery",
      sensor_qos,
      std::bind(&MavrosHardwareBridgeNode::on_mavros_battery, this, std::placeholders::_1));

  sub_mavros_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      "/mavros/local_position/odom",
      sensor_qos,
      std::bind(&MavrosHardwareBridgeNode::on_mavros_odom, this, std::placeholders::_1));
}

void MavrosHardwareBridgeNode::create_services()
{
  srv_mower_control_ = create_service<mowgli_interfaces::srv::MowerControl>(
      "~/mower_control",
      std::bind(&MavrosHardwareBridgeNode::on_mower_control,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

  srv_emergency_stop_ = create_service<mowgli_interfaces::srv::EmergencyStop>(
      "~/emergency_stop",
      std::bind(&MavrosHardwareBridgeNode::on_emergency_stop,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
}

void MavrosHardwareBridgeNode::create_clients()
{
  cli_arm_ = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
  cli_set_mode_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
}

void MavrosHardwareBridgeNode::create_timers()
{
  const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, status_publish_rate_hz_));

  timer_status_ = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(period),
                                    [this]()
                                    {
                                      publish_status();
                                      publish_emergency();
                                      publish_power();
                                    });
}

void MavrosHardwareBridgeNode::on_cmd_vel(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  if (!manual_control_enabled_)
  {
    RCLCPP_WARN_THROTTLE(get_logger(),
                         *get_clock(),
                         5000,
                         "Ignoring /cmd_vel because manual_control_enabled=false.");
    return;
  }

  mavros_msgs::msg::ManualControl cmd{};
  cmd.header.stamp = now();

  // This mapping remains provisional until Pixhawk manual control behavior
  // is validated on hardware.
  cmd.x = static_cast<int16_t>(msg->twist.linear.x * manual_control_linear_scale_);
  cmd.r = static_cast<int16_t>(msg->twist.angular.z * manual_control_yaw_scale_);

  pub_manual_control_->publish(cmd);
}

void MavrosHardwareBridgeNode::on_high_level_status(
    const mowgli_interfaces::msg::HighLevelStatus::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_high_level_status_ = *msg;
}

void MavrosHardwareBridgeNode::on_mavros_state(const mavros_msgs::msg::State::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  mavros_state_ = *msg;
}

void MavrosHardwareBridgeNode::on_mavros_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_imu_ = *msg;
  }
  pub_imu_->publish(*msg);
}

void MavrosHardwareBridgeNode::on_mavros_battery(
    const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_battery_ = *msg;
    battery_voltage_ = msg->voltage;

    if (charging_feedback_enabled_)
    {
      const auto status = msg->power_supply_status;
      is_charging_ = status == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING ||
                     status == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL;
      charger_enabled_ = is_charging_;
      charge_current_ =
          (is_charging_ && std::isfinite(msg->current)) ? std::abs(msg->current) : 0.0;
      charger_status_ = is_charging_ ? "charging" : "not_charging";
    }
    else
    {
      is_charging_ = false;
      charger_enabled_ = false;
      charge_current_ = 0.0;
      charger_status_ = "unknown";
    }
  }
  pub_battery_state_->publish(*msg);
}

void MavrosHardwareBridgeNode::on_mavros_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_odom_ = *msg;
  }
  pub_wheel_odom_->publish(*msg);
}

void MavrosHardwareBridgeNode::on_mower_control(
    const std::shared_ptr<mowgli_interfaces::srv::MowerControl::Request> request,
    std::shared_ptr<mowgli_interfaces::srv::MowerControl::Response> response)
{
  if (!blade_control_enabled_)
  {
    RCLCPP_WARN(get_logger(),
                "Mower control requested, but blade_control_enabled=false because the Pixhawk "
                "blade path is still provisional.");
    response->success = false;
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    mow_enabled_ = request->mow_enabled;
    mow_direction_ = request->mow_direction;
  }

  RCLCPP_WARN(get_logger(),
              "Mower control requested (enabled=%d, direction=%d) but cutting motor mapping on the "
              "Pixhawk/ArduPilot is not implemented yet.",
              request->mow_enabled,
              request->mow_direction);

  response->success = false;
}

void MavrosHardwareBridgeNode::on_emergency_stop(
    const std::shared_ptr<mowgli_interfaces::srv::EmergencyStop::Request> request,
    std::shared_ptr<mowgli_interfaces::srv::EmergencyStop::Response> response)
{
  const bool emergency_requested = (request->emergency != 0U);

  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (emergency_requested)
    {
      emergency_active_ = true;
      emergency_latched_ = true;
      emergency_reason_ = "SERVICE_EMERGENCY_STOP";
      mow_enabled_ = false;
    }
    else
    {
      emergency_active_ = false;
      emergency_reason_ = "NONE";
    }
  }

  if (!emergency_requested)
  {
    response->success = true;
    return;
  }

  // This service reports whether the emergency request was accepted locally
  // and forwarded to MAVROS. It does not imply the autopilot has already
  // confirmed or completed the requested state change.
  bool request_sent = true;

  if (!emergency_mode_.empty())
  {
    request_sent = send_mode_command(emergency_mode_) && request_sent;
  }
  if (emergency_disarm_)
  {
    request_sent = send_arm_command(false) && request_sent;
  }

  response->success = request_sent;

  if (request_sent)
  {
    RCLCPP_WARN(get_logger(),
                "Emergency stop request sent to MAVROS (mode='%s', disarm=%s). Autopilot "
                "confirmation will be logged asynchronously.",
                emergency_mode_.c_str(),
                emergency_disarm_ ? "true" : "false");
  }
  else
  {
    RCLCPP_ERROR(get_logger(),
                 "Emergency stop request could not be fully sent to MAVROS (mode='%s', disarm=%s).",
                 emergency_mode_.c_str(),
                 emergency_disarm_ ? "true" : "false");
  }
}

void MavrosHardwareBridgeNode::publish_status()
{
  mowgli_interfaces::msg::Status msg;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    msg.stamp = now();
    msg.raspberry_pi_power = raspberry_pi_power_;
    msg.is_charging = is_charging_;
    msg.esc_power = esc_power_;
    msg.rain_detected = rain_detected_;
    msg.sound_module_available = sound_module_available_;
    msg.sound_module_busy = sound_module_busy_;
    msg.ui_board_available = ui_board_available_;
    msg.mow_enabled = mow_enabled_;

    // TODO: define proper mower status enum mapping.
    msg.mower_status = mavros_state_.armed ? 1U : 0U;

    msg.mower_esc_status = mower_esc_status_;
    msg.mower_esc_temperature = mower_esc_temperature_;
    msg.mower_esc_current = mower_esc_current_;
    msg.mower_motor_temperature = mower_motor_temperature_;
    msg.mower_motor_rpm = mower_motor_rpm_;
  }

  pub_status_->publish(msg);
}

void MavrosHardwareBridgeNode::publish_emergency()
{
  mowgli_interfaces::msg::Emergency msg;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    msg.stamp = now();
    msg.active_emergency = emergency_active_;
    msg.latched_emergency = emergency_latched_;
    msg.reason = emergency_reason_;
  }

  pub_emergency_->publish(msg);
}

void MavrosHardwareBridgeNode::publish_power()
{
  mowgli_interfaces::msg::Power msg;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    msg.stamp = now();
    msg.v_charge = static_cast<float>(charge_voltage_);
    msg.v_battery = static_cast<float>(battery_voltage_);
    msg.charge_current = static_cast<float>(charge_current_);
    msg.charger_enabled = charger_enabled_;
    msg.charger_status = charger_status_;
  }

  pub_power_->publish(msg);
}

bool MavrosHardwareBridgeNode::send_arm_command(bool arm)
{
  if (!cli_arm_)
  {
    RCLCPP_ERROR(get_logger(), "CommandBool client is null");
    return false;
  }

  if (!cli_arm_->wait_for_service(std::chrono::seconds(1)))
  {
    RCLCPP_ERROR(get_logger(), "Service /mavros/cmd/arming not available");
    return false;
  }

  auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  request->value = arm;

  cli_arm_->async_send_request(
      request,
      [this, arm](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future)
      {
        try
        {
          const auto response = future.get();
          if (!response)
          {
            RCLCPP_ERROR(get_logger(), "Null response from /mavros/cmd/arming");
            return;
          }

          if (!response->success)
          {
            RCLCPP_ERROR(get_logger(),
                         "Autopilot rejected arming request: %s",
                         arm ? "ARM" : "DISARM");
            return;
          }

          RCLCPP_INFO(get_logger(),
                      "Autopilot confirmed arming request: %s",
                      arm ? "ARM" : "DISARM");
        }
        catch (const std::exception& e)
        {
          RCLCPP_ERROR(get_logger(), "Arm/disarm request failed asynchronously: %s", e.what());
        }
      });

  return true;
}

bool MavrosHardwareBridgeNode::send_mode_command(const std::string& mode)
{
  if (!cli_set_mode_)
  {
    RCLCPP_ERROR(get_logger(), "SetMode client is null");
    return false;
  }

  if (!cli_set_mode_->wait_for_service(std::chrono::seconds(1)))
  {
    RCLCPP_ERROR(get_logger(), "Service /mavros/set_mode not available");
    return false;
  }

  auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  request->custom_mode = mode;

  cli_set_mode_->async_send_request(
      request,
      [this, mode](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future)
      {
        try
        {
          const auto response = future.get();
          if (!response)
          {
            RCLCPP_ERROR(get_logger(), "Null response from /mavros/set_mode");
            return;
          }

          if (!response->mode_sent)
          {
            RCLCPP_ERROR(get_logger(), "Autopilot rejected mode request '%s'", mode.c_str());
            return;
          }

          RCLCPP_INFO(get_logger(), "Autopilot confirmed mode request '%s'", mode.c_str());
        }
        catch (const std::exception& e)
        {
          RCLCPP_ERROR(get_logger(),
                       "Set mode request failed asynchronously for '%s': %s",
                       mode.c_str(),
                       e.what());
        }
      });

  return true;
}

}  // namespace mowgli_mavros_bridge

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mowgli_mavros_bridge::MavrosHardwareBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
