#include <cmath>
#include <fstream>
#include <mutex>
#include <sstream>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>

#include "mowgli_brv_planner/brv_algorithm.hpp"
#include "mowgli_brv_planner/types.hpp"
#include <mowgli_interfaces/action/plan_coverage.hpp>
#include <mowgli_interfaces/msg/obstacle_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using PlanCoverage = mowgli_interfaces::action::PlanCoverage;
using GoalHandle = rclcpp_action::ServerGoalHandle<PlanCoverage>;

class BrvPlannerNode : public rclcpp::Node
{
public:
  BrvPlannerNode() : Node("coverage_planner_node")
  {
    // Parameters
    tool_width_ = declare_parameter("tool_width", 0.18);
    headland_passes_ = declare_parameter("headland_passes", 2);
    headland_width_ = declare_parameter("headland_width", 0.18);
    default_mow_angle_ = declare_parameter("default_mow_angle", -1.0);
    mowing_speed_ = declare_parameter("mowing_speed", 0.15);
    transit_speed_ = declare_parameter("transit_speed", 0.3);
    map_frame_ = declare_parameter("map_frame", std::string("map"));
    voronoi_sample_spacing_ = declare_parameter("voronoi_sample_spacing", 0.18);
    voronoi_knn_ = declare_parameter("voronoi_knn", 10);
    min_obstacle_area_ = declare_parameter("min_obstacle_area", 0.01);
    route_graph_filepath_ =
        declare_parameter("route_graph_filepath", std::string("/tmp/mowing_route.geojson"));

    // Publishers
    auto transient_qos = rclcpp::QoS(1).transient_local();
    path_pub_ = create_publisher<nav_msgs::msg::Path>("~/coverage_path", transient_qos);
    outline_pub_ = create_publisher<nav_msgs::msg::Path>("~/coverage_outline", transient_qos);
    route_graph_pub_ = create_publisher<std_msgs::msg::String>("~/route_graph", transient_qos);
    voronoi_pub_ =
        create_publisher<visualization_msgs::msg::MarkerArray>("~/voronoi_roadmap", transient_qos);

    // Obstacle subscription
    obstacle_sub_ = create_subscription<mowgli_interfaces::msg::ObstacleArray>(
        "/obstacle_tracker/obstacles",
        rclcpp::QoS(5),
        [this](const mowgli_interfaces::msg::ObstacleArray::SharedPtr msg)
        {
          std::lock_guard<std::mutex> lock(obs_mutex_);
          tracked_obstacles_ = msg->obstacles;
        });

    // Write a minimal valid GeoJSON graph so route_server doesn't fail at startup.
    // Format: Point nodes with "id", MultiLineString edges with "startid"/"endid".
    {
      std::ofstream f(route_graph_filepath_);
      if (f.is_open())
      {
        f << "{\n  \"type\": \"FeatureCollection\",\n  \"features\": [\n"
          << "    {\"type\": \"Feature\", \"geometry\": {\"type\": \"Point\", "
          << "\"coordinates\": [0, 0]}, \"properties\": {\"id\": 0}},\n"
          << "    {\"type\": \"Feature\", \"geometry\": {\"type\": \"Point\", "
          << "\"coordinates\": [0.1, 0]}, \"properties\": {\"id\": 1}},\n"
          << "    {\"type\": \"Feature\", \"geometry\": {\"type\": \"MultiLineString\", "
          << "\"coordinates\": [[[0, 0], [0.1, 0]]]}, \"properties\": {"
          << "\"id\": 0, \"startid\": 0, \"endid\": 1, \"cost\": 0.1}}\n"
          << "  ]\n}\n";
      }
    }

    // Action server
    action_server_ = rclcpp_action::create_server<PlanCoverage>(
        this,
        "~/plan_coverage",
        [this](auto, auto)
        {
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](auto)
        {
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](auto goal_handle)
        {
          execute(goal_handle);
        });

    RCLCPP_INFO(get_logger(),
                "BrvPlannerNode: tool_width=%.3f m, headland_passes=%d, "
                "headland_width=%.3f m, voronoi_knn=%d, frame='%s'",
                tool_width_,
                headland_passes_,
                headland_width_,
                voronoi_knn_,
                map_frame_.c_str());
    RCLCPP_INFO(get_logger(), "BrvPlannerNode ready — action server on ~/plan_coverage");
  }

private:
  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "B-RV planning requested");

    const auto& goal = goal_handle->get_goal();
    auto result = std::make_shared<PlanCoverage::Result>();
    auto feedback = std::make_shared<PlanCoverage::Feedback>();

    // Convert boundary
    const auto& boundary_msg = goal->outer_boundary;
    if (boundary_msg.points.size() < 3)
    {
      result->success = false;
      result->message = "Boundary must have at least 3 vertices";
      goal_handle->succeed(result);
      return;
    }

    brv::Polygon2D boundary;
    for (const auto& pt : boundary_msg.points)
    {
      boundary.push_back({static_cast<double>(pt.x), static_cast<double>(pt.y)});
    }

    // Collect obstacles from goal + tracked
    std::vector<brv::Polygon2D> obstacles;
    for (const auto& obs_msg : goal->obstacles)
    {
      if (obs_msg.points.size() < 3)
        continue;
      brv::Polygon2D obs;
      for (const auto& pt : obs_msg.points)
      {
        obs.push_back({static_cast<double>(pt.x), static_cast<double>(pt.y)});
      }
      obstacles.push_back(obs);
    }
    {
      std::lock_guard<std::mutex> lock(obs_mutex_);
      for (const auto& tracked : tracked_obstacles_)
      {
        if (tracked.polygon.points.size() < 3)
          continue;
        if (tracked.status != 1)
          continue;  // Only persistent obstacles
        brv::Polygon2D obs;
        for (const auto& pt : tracked.polygon.points)
        {
          obs.push_back({static_cast<double>(pt.x), static_cast<double>(pt.y)});
        }
        obstacles.push_back(obs);
      }
    }

    // Setup params
    brv::PlannerParams params;
    params.tool_width = tool_width_;
    params.headland_passes = goal->skip_outline ? 0 : headland_passes_;
    params.headland_width = headland_width_;
    params.mow_angle_deg = goal->mow_angle_deg;
    if (params.mow_angle_deg < 0)
    {
      params.mow_angle_deg = default_mow_angle_;
    }
    params.voronoi_sample_spacing = voronoi_sample_spacing_;
    params.voronoi_knn = voronoi_knn_;
    params.min_obstacle_area = min_obstacle_area_;

    // Progress callback
    auto progress_fn = [&](float pct, const std::string& phase)
    {
      feedback->progress_percent = pct;
      feedback->phase = phase;
      goal_handle->publish_feedback(feedback);
    };

    RCLCPP_INFO(get_logger(),
                "Planning with %zu boundary points, %zu obstacles",
                boundary.size(),
                obstacles.size());

    // Run B-RV algorithm
    brv::CoverageResult plan;
    try
    {
      plan = brv::plan_coverage(boundary, obstacles, params, progress_fn);
    }
    catch (const std::exception& e)
    {
      result->success = false;
      result->message = std::string("Planning failed: ") + e.what();
      goal_handle->succeed(result);
      return;
    }

    if (plan.full_path.empty())
    {
      result->success = false;
      result->message = "No coverage path generated (area too small?)";
      goal_handle->succeed(result);
      return;
    }

    RCLCPP_INFO(get_logger(),
                "B-RV plan: %zu waypoints, %zu swaths, %.1f m distance, %.1f m² area",
                plan.full_path.size(),
                plan.swaths.size(),
                plan.total_distance,
                plan.coverage_area);

    // Debug: log boundary, obstacles, and first/last waypoints
    for (size_t i = 0; i < boundary.size(); ++i)
    {
      RCLCPP_INFO(get_logger(), "  boundary[%zu]: (%.3f, %.3f)", i, boundary[i].x, boundary[i].y);
    }
    for (size_t i = 0; i < obstacles.size(); ++i)
    {
      std::string obs_str;
      for (const auto& pt : obstacles[i])
      {
        obs_str += "(" + std::to_string(pt.x) + "," + std::to_string(pt.y) + ") ";
      }
      RCLCPP_INFO(get_logger(), "  obstacle[%zu]: %s", i, obs_str.c_str());
    }
    size_t n = plan.full_path.size();
    for (size_t i = 0; i < std::min(n, size_t(5)); ++i)
    {
      RCLCPP_INFO(
          get_logger(), "  path[%zu]: (%.3f, %.3f)", i, plan.full_path[i].x, plan.full_path[i].y);
    }
    if (n > 10)
    {
      RCLCPP_INFO(get_logger(), "  ... (%zu waypoints) ...", n - 10);
    }
    for (size_t i = (n > 5 ? n - 5 : 0); i < n; ++i)
    {
      if (i >= 5)
      {
        RCLCPP_INFO(
            get_logger(), "  path[%zu]: (%.3f, %.3f)", i, plan.full_path[i].x, plan.full_path[i].y);
      }
    }

    // Build result
    result->success = true;
    result->message = "B-RV coverage path planned";
    result->total_distance = plan.total_distance;
    result->coverage_area = plan.coverage_area;

    // Full path with orientations
    result->path = to_path_msg(plan.full_path);

    // Outline path
    result->outline_path = to_path_msg(plan.outline_path);

    // Swath start/end points
    for (const auto& swath : plan.swaths)
    {
      geometry_msgs::msg::Point sp, ep;
      sp.x = swath.start.x;
      sp.y = swath.start.y;
      sp.z = 0.0;
      ep.x = swath.end.x;
      ep.y = swath.end.y;
      ep.z = 0.0;
      result->swath_starts.push_back(sp);
      result->swath_ends.push_back(ep);
    }

    // Publish visualizations
    path_pub_->publish(result->path);
    outline_pub_->publish(result->outline_path);
    publish_route_graph(plan);

    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "B-RV planning complete");
  }

  nav_msgs::msg::Path to_path_msg(const brv::Path2D& waypoints)
  {
    nav_msgs::msg::Path msg;
    msg.header.frame_id = map_frame_;
    msg.header.stamp = now();
    msg.poses.reserve(waypoints.size());

    for (size_t i = 0; i < waypoints.size(); ++i)
    {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = msg.header;
      ps.pose.position.x = waypoints[i].x;
      ps.pose.position.y = waypoints[i].y;
      ps.pose.position.z = 0.0;

      // Compute yaw toward next waypoint
      double yaw = 0.0;
      if (i + 1 < waypoints.size())
      {
        yaw = std::atan2(waypoints[i + 1].y - waypoints[i].y, waypoints[i + 1].x - waypoints[i].x);
      }
      else if (i > 0)
      {
        yaw = std::atan2(waypoints[i].y - waypoints[i - 1].y, waypoints[i].x - waypoints[i - 1].x);
      }

      ps.pose.orientation.z = std::sin(yaw / 2.0);
      ps.pose.orientation.w = std::cos(yaw / 2.0);
      msg.poses.push_back(ps);
    }
    return msg;
  }

  void publish_route_graph(const brv::CoverageResult& plan)
  {
    if (plan.swaths.empty())
      return;

    // Build GeoJSON compatible with Nav2 route_server.
    // Point nodes with "id", MultiLineString edges with "startid"/"endid".
    std::ostringstream json;
    json << std::fixed;
    json.precision(6);
    json << "{\n  \"type\": \"FeatureCollection\",\n  \"features\": [\n";

    size_t n = plan.swaths.size();
    bool first = true;

    // Point nodes: 2 per swath (start=2*i, end=2*i+1)
    for (size_t i = 0; i < n; ++i)
    {
      if (!first)
        json << ",\n";
      first = false;
      json << "    {\"type\": \"Feature\", \"geometry\": {\"type\": \"Point\", "
           << "\"coordinates\": [" << plan.swaths[i].start.x << ", " << plan.swaths[i].start.y
           << "]}, \"properties\": {\"id\": " << (2 * i) << "}}";
      json << ",\n";
      json << "    {\"type\": \"Feature\", \"geometry\": {\"type\": \"Point\", "
           << "\"coordinates\": [" << plan.swaths[i].end.x << ", " << plan.swaths[i].end.y
           << "]}, \"properties\": {\"id\": " << (2 * i + 1) << "}}";
    }

    // Swath edges (blade_on)
    size_t eid = 0;
    for (size_t i = 0; i < n; ++i)
    {
      double cost = plan.swaths[i].start.dist(plan.swaths[i].end);
      json << ",\n    {\"type\": \"Feature\", \"geometry\": {\"type\": \"MultiLineString\", "
           << "\"coordinates\": [[[" << plan.swaths[i].start.x << ", " << plan.swaths[i].start.y
           << "], [" << plan.swaths[i].end.x << ", " << plan.swaths[i].end.y
           << "]]]}, \"properties\": {\"id\": " << eid++ << ", \"startid\": " << (2 * i)
           << ", \"endid\": " << (2 * i + 1) << ", \"cost\": " << cost
           << ", \"speed_limit\": " << mowing_speed_ << ", \"operation\": \"blade_on\"}}";
    }

    // Turn edges (blade_off)
    for (size_t i = 0; i + 1 < n; ++i)
    {
      double cost = plan.swaths[i].end.dist(plan.swaths[i + 1].start);
      json << ",\n    {\"type\": \"Feature\", \"geometry\": {\"type\": \"MultiLineString\", "
           << "\"coordinates\": [[[" << plan.swaths[i].end.x << ", " << plan.swaths[i].end.y
           << "], [" << plan.swaths[i + 1].start.x << ", " << plan.swaths[i + 1].start.y
           << "]]]}, \"properties\": {\"id\": " << eid++ << ", \"startid\": " << (2 * i + 1)
           << ", \"endid\": " << (2 * (i + 1)) << ", \"cost\": " << cost
           << ", \"speed_limit\": " << transit_speed_ << ", \"operation\": \"blade_off\"}}";
    }

    json << "\n  ]\n}\n";

    auto msg = std_msgs::msg::String();
    msg.data = json.str();
    route_graph_pub_->publish(msg);

    std::ofstream f(route_graph_filepath_);
    if (f.is_open())
    {
      f << msg.data;
      RCLCPP_INFO(get_logger(),
                  "Route graph: %zu nodes, %zu edges -> %s",
                  2 * n,
                  eid,
                  route_graph_filepath_.c_str());
    }
  }

  // Parameters
  double tool_width_;
  int headland_passes_;
  double headland_width_;
  double default_mow_angle_;
  double mowing_speed_;
  double transit_speed_;
  std::string map_frame_;
  double voronoi_sample_spacing_;
  int voronoi_knn_;
  double min_obstacle_area_;
  std::string route_graph_filepath_;

  // ROS interfaces
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr outline_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr route_graph_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr voronoi_pub_;
  rclcpp::Subscription<mowgli_interfaces::msg::ObstacleArray>::SharedPtr obstacle_sub_;
  rclcpp_action::Server<PlanCoverage>::SharedPtr action_server_;

  // Tracked obstacles
  std::mutex obs_mutex_;
  std::vector<mowgli_interfaces::msg::TrackedObstacle> tracked_obstacles_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BrvPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
