#include "fourwis_hybrid_astar_cpp/planner_core.hpp"
#include "fourwis_hybrid_astar_cpp/safe_corridor_optimizer.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace fourwis_hybrid_astar_cpp
{

class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode() : Node("fourwis_hybrid_astar_cpp")
  {
    declare_parameter("xy_resolution", 0.2);
    declare_parameter("yaw_resolution", M_PI / 12.0);
    declare_parameter("heuristic_resolution", 0.2);
    declare_parameter("analytic_expansion_distance", 5.0);
    declare_parameter("goal_distance_tolerance", 0.25);
    declare_parameter("goal_yaw_tolerance", M_PI / 18.0);
    declare_parameter("max_iterations", 200000);
    declare_parameter("reference_velocity", 1.0);
    declare_parameter("sampling_time", 0.5);
    declare_parameter("integration_dt", 0.05);
    declare_parameter("wheelbase", 0.68);
    declare_parameter("robot_length", 1.0);
    declare_parameter("robot_width", 0.62);
    declare_parameter("collision_clearance", 0.0);
    declare_parameter("max_steer_deg", 30.0);
    declare_parameter("reverse_penalty", 2.0);
    declare_parameter("steer_penalty", 1.0);
    declare_parameter("steer_change_penalty", 1.0);
    declare_parameter("direction_change_penalty", 1.0);
    declare_parameter("obstacle_threshold", 50);
    declare_parameter("unknown_is_obstacle", true);
    declare_parameter("pose_arrow_length", 0.9);
    declare_parameter("pose_arrow_width", 0.16);
    declare_parameter("pose_arrow_height", 0.16);
    declare_parameter("pose_marker_z", 0.18);
    declare_parameter("body_trajectory_visualization_enabled", false);
    declare_parameter("body_trajectory_marker_z", 0.09);
    declare_parameter("backend_casadi_python", "/home/balmung/miniconda3/bin/python3");
    declare_parameter(
      "backend_casadi_script",
      "/home/balmung/Legacy/4WIS_Global/fourwis_hybrid_astar_cpp/scripts/casadi_backend.py");
    declare_parameter("backend_resample_distance", 0.1);
    declare_parameter("backend_corridor_max_distance", 1.2);
    declare_parameter("backend_corridor_fast_step", 0.25);
    declare_parameter("backend_corridor_fine_step", 0.05);
    declare_parameter("backend_corridor_axis_aligned", true);
    declare_parameter("backend_max_iterations", 1);
    declare_parameter("backend_ipopt_max_iterations", 1000);
    declare_parameter("backend_comfort_weight", 0.1);
    declare_parameter("backend_constraint_penalty", 1000.0);
    declare_parameter("backend_infeasibility_tolerance", 1.0e-6);
    declare_parameter("backend_max_velocity", 1.0);
    declare_parameter("backend_max_acceleration", 2.0);
    declare_parameter("backend_ipopt_tol", 1.0e-4);
    declare_parameter("backend_corridor_visualization_enabled", true);
    declare_parameter("backend_corridor_visualization_stride", 8);
    declare_parameter("backend_corridor_marker_z", 0.06);

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&PlannerNode::onMap, this, std::placeholders::_1));
    start_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", 10, std::bind(&PlannerNode::onStart, this, std::placeholders::_1));
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose", 10, std::bind(&PlannerNode::onGoal, this, std::placeholders::_1));

    path_pub_ = create_publisher<nav_msgs::msg::Path>("fourwis_path", 10);
    frontend_path_pub_ = create_publisher<nav_msgs::msg::Path>("fourwis_frontend_path", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("fourwis_path_markers", 10);
    body_marker_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("fourwis_body_markers", 10);
    corridor_marker_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("fourwis_corridor_markers", 10);
    pose_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("fourwis_pose_markers", 10);
    state_pub_ = create_publisher<std_msgs::msg::String>(
      "fourwis_planner_state", rclcpp::QoS(1).transient_local().reliable());
    metrics_text_pub_ = create_publisher<std_msgs::msg::String>(
      "fourwis_metrics_text", rclcpp::QoS(1).transient_local().reliable());
    status_pub_ = create_publisher<std_msgs::msg::String>("fourwis_planner_status", 10);
    status_text_ = "WAITING MAP";
    status_timer_ = create_wall_timer(
      std::chrono::seconds(1), std::bind(&PlannerNode::republishStatusMarker, this));
    body_timer_ = create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&PlannerNode::advanceBodyTrajectoryMarker, this));
    body_timer_->cancel();
  }

private:
  struct TrajectoryMetrics
  {
    double arrival_time{0.0};
    double comfort{0.0};
    double length{0.0};
    int states{0};
  };

  PlannerConfig config() const
  {
    PlannerConfig cfg;
    cfg.xy_resolution = get_parameter("xy_resolution").as_double();
    cfg.yaw_resolution = get_parameter("yaw_resolution").as_double();
    cfg.heuristic_resolution = get_parameter("heuristic_resolution").as_double();
    cfg.analytic_expansion_distance = get_parameter("analytic_expansion_distance").as_double();
    cfg.goal_distance_tolerance = get_parameter("goal_distance_tolerance").as_double();
    cfg.goal_yaw_tolerance = get_parameter("goal_yaw_tolerance").as_double();
    cfg.max_iterations = get_parameter("max_iterations").as_int();
    cfg.reference_velocity = get_parameter("reference_velocity").as_double();
    cfg.sampling_time = get_parameter("sampling_time").as_double();
    cfg.integration_dt = get_parameter("integration_dt").as_double();
    cfg.wheelbase = get_parameter("wheelbase").as_double();
    cfg.robot_length = get_parameter("robot_length").as_double();
    cfg.robot_width = get_parameter("robot_width").as_double();
    cfg.collision_clearance = get_parameter("collision_clearance").as_double();
    cfg.max_steer = get_parameter("max_steer_deg").as_double() * M_PI / 180.0;
    cfg.reverse_penalty = get_parameter("reverse_penalty").as_double();
    cfg.steer_penalty = get_parameter("steer_penalty").as_double();
    cfg.steer_change_penalty = get_parameter("steer_change_penalty").as_double();
    cfg.direction_change_penalty = get_parameter("direction_change_penalty").as_double();
    cfg.obstacle_threshold = get_parameter("obstacle_threshold").as_int();
    cfg.unknown_is_obstacle = get_parameter("unknown_is_obstacle").as_bool();
    return cfg;
  }

  static double normAngle(double angle)
  {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle <= -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

  static double signedVelocity(const State & state, const State & next, double dt)
  {
    const double dx = next.x - state.x;
    const double dy = next.y - state.y;
    const double sign = (dx * std::cos(state.yaw) + dy * std::sin(state.yaw)) < 0.0 ? -1.0 : 1.0;
    return sign * std::hypot(dx, dy) / std::max(dt, 1e-3);
  }

  static double estimateSteering(
    const State & state, const State & next, double velocity, double dt,
    const PlannerConfig & planner_cfg)
  {
    if (std::abs(velocity) < 1e-4) {
      return 0.0;
    }
    const double yaw_rate = normAngle(next.yaw - state.yaw) / std::max(dt, 1e-3);
    return std::clamp(
      std::atan(yaw_rate * planner_cfg.wheelbase / (2.0 * velocity)),
      -planner_cfg.max_steer, planner_cfg.max_steer);
  }

  static TrajectoryMetrics computeTrajectoryMetrics(
    const std::vector<State> & states, const PlannerConfig & planner_cfg,
    const SafeCorridorConfig & backend_cfg)
  {
    TrajectoryMetrics metrics;
    metrics.states = static_cast<int>(states.size());
    if (states.size() < 2) {
      return metrics;
    }

    const double dt = std::max(
      1e-3, backend_cfg.resample_distance / std::max(backend_cfg.max_velocity, 1e-3));
    std::vector<double> velocity(states.size() - 1, 0.0);
    std::vector<double> steering(states.size() - 1, 0.0);
    for (std::size_t i = 0; i + 1 < states.size(); ++i) {
      const double dx = states[i + 1].x - states[i].x;
      const double dy = states[i + 1].y - states[i].y;
      metrics.length += std::hypot(dx, dy);
      velocity[i] = signedVelocity(states[i], states[i + 1], dt);
      steering[i] = estimateSteering(states[i], states[i + 1], velocity[i], dt, planner_cfg);
    }

    metrics.arrival_time = static_cast<double>(states.size() - 1) * dt;
    for (std::size_t i = 0; i < velocity.size(); ++i) {
      const double omega = i + 1 < steering.size() ?
        normAngle(steering[i + 1] - steering[i]) / dt :
        0.0;
      metrics.comfort += velocity[i] * velocity[i] * omega * omega * dt;
    }
    return metrics;
  }

  SafeCorridorConfig optimizerConfig() const
  {
    SafeCorridorConfig cfg;
    cfg.casadi_python = get_parameter("backend_casadi_python").as_string();
    cfg.casadi_script = get_parameter("backend_casadi_script").as_string();
    cfg.resample_distance = get_parameter("backend_resample_distance").as_double();
    cfg.corridor_max_distance = get_parameter("backend_corridor_max_distance").as_double();
    cfg.corridor_fast_step = get_parameter("backend_corridor_fast_step").as_double();
    cfg.corridor_fine_step = get_parameter("backend_corridor_fine_step").as_double();
    cfg.corridor_axis_aligned = get_parameter("backend_corridor_axis_aligned").as_bool();
    cfg.max_iterations = get_parameter("backend_max_iterations").as_int();
    cfg.ipopt_max_iterations = get_parameter("backend_ipopt_max_iterations").as_int();
    cfg.comfort_weight = get_parameter("backend_comfort_weight").as_double();
    cfg.constraint_penalty = get_parameter("backend_constraint_penalty").as_double();
    cfg.infeasibility_tolerance = get_parameter("backend_infeasibility_tolerance").as_double();
    cfg.max_velocity = get_parameter("backend_max_velocity").as_double();
    cfg.max_acceleration = get_parameter("backend_max_acceleration").as_double();
    cfg.ipopt_tol = get_parameter("backend_ipopt_tol").as_double();
    return cfg;
  }

  void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    const auto cfg = config();
    grid_ = std::make_shared<OccupancyGrid>(
      msg->data, static_cast<int>(msg->info.width), static_cast<int>(msg->info.height),
      msg->info.resolution, msg->info.origin.position.x, msg->info.origin.position.y,
      cfg.obstacle_threshold, cfg.unknown_is_obstacle);
    map_frame_ = msg->header.frame_id.empty() ? "map" : msg->header.frame_id;
    RCLCPP_DEBUG(
      get_logger(), "received map %ux%u, resolution=%.3f",
      msg->info.width, msg->info.height, msg->info.resolution);
    publishPipelineStatus("WAITING START", {0.9, 0.75, 0.1, 1.0});
  }

  void onStart(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    start_ = poseToState(msg->pose.pose);
    has_start_ = true;
    RCLCPP_INFO(
      get_logger(), "start set to x=%.2f y=%.2f yaw=%.2f",
      start_.x, start_.y, start_.yaw);
    publishPoseMarkers();
    publishPipelineStatus("WAITING GOAL", {0.9, 0.75, 0.1, 1.0});
  }

  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!grid_) {
      publishStatus("no map received");
      publishPipelineStatus("WAITING MAP", {0.9, 0.75, 0.1, 1.0});
      return;
    }
    if (!has_start_) {
      publishStatus("no start pose received on /initialpose");
      publishPipelineStatus("WAITING START", {0.9, 0.75, 0.1, 1.0});
      return;
    }

    goal_ = poseToState(msg->pose);
    has_goal_ = true;
    publishPoseMarkers();

    AckermannHybridAStar planner(*grid_, config());
    PlanningResult result;
    const auto start_time = std::chrono::steady_clock::now();
    publishPipelineStatus("FRONT-END SEARCH", {0.1, 0.45, 1.0, 1.0});
    const bool ok = planner.plan(start_, goal_, result);
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - start_time).count();

    if (!ok) {
      publishStatus("planning failed");
      publishPipelineStatus("FAILED", {0.95, 0.1, 0.1, 1.0});
      return;
    }

    const std::vector<State> frontend_states = result.states;
    publishPath(frontend_states, frontend_path_pub_);

    const SafeCorridorConfig backend_cfg = optimizerConfig();
    const PlannerConfig planner_cfg = config();
    publishPipelineStatus("BACK-END OPT", {0.75, 0.35, 1.0, 1.0});
    SafeCorridorOptimizer optimizer(*grid_, planner_cfg, backend_cfg);
    OptimizerResult optimized = optimizer.optimize(frontend_states);
    publishCorridors(optimized.corridors);
    std::vector<State> output_states = std::move(optimized.states);

    TrajectoryMetrics metrics = computeTrajectoryMetrics(output_states, planner_cfg, backend_cfg);
    if (optimized.stats.optimized_time > 1e-9) {
      metrics.arrival_time = optimized.stats.optimized_time;
    }
    publishResult(output_states, metrics.arrival_time);
    publishMetrics(metrics);
    publishStatus(
      "planned " + std::to_string(output_states.size()) + " states, cost=" +
      std::to_string(result.cost) + ", iterations=" + std::to_string(result.iterations) +
      ", wall_time_ms=" + std::to_string(elapsed_ms) +
      ", core_total_ms=" + std::to_string(result.timing.total_ms) +
      ", heuristic_ms=" + std::to_string(result.timing.heuristic_map_ms) +
      ", search_ms=" + std::to_string(result.timing.search_loop_ms) +
      ", backend_ms=" + std::to_string(optimized.stats.total_ms) +
      ", backend_iterations=" + std::to_string(optimized.stats.iterations) +
      ", backend_ipopt_iterations=" + std::to_string(optimized.stats.ipopt_iterations) +
      ", backend_ipopt_status=" + std::to_string(optimized.stats.ipopt_status) +
      ", backend_solve_success=" + std::to_string(optimized.stats.solve_success) +
      ", backend_optimized_time=" + std::to_string(optimized.stats.optimized_time) +
      ", backend_max_delta=" + std::to_string(optimized.stats.max_position_delta) +
      ", backend_mean_delta=" + std::to_string(optimized.stats.mean_position_delta) +
      ", max_corridor_length=" + std::to_string(optimized.stats.max_corridor_length) +
      ", max_corridor_width=" + std::to_string(optimized.stats.max_corridor_width));
    publishPipelineStatus("COMPLETE", {0.1, 0.85, 0.25, 1.0});
  }

  void publishPath(
    const std::vector<State> & states,
    const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr & publisher) const
  {
    std_msgs::msg::Header header;
    header.stamp = now();
    header.frame_id = map_frame_;

    nav_msgs::msg::Path path_msg;
    path_msg.header = header;

    for (const auto & state : states) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = header;
      pose.pose.position.x = state.x;
      pose.pose.position.y = state.y;
      pose.pose.orientation = yawToQuaternion(state.yaw);
      path_msg.poses.push_back(pose);
    }
    publisher->publish(path_msg);
  }

  void publishResult(const std::vector<State> & states, double playback_time)
  {
    std_msgs::msg::Header header;
    header.stamp = now();
    header.frame_id = map_frame_;

    nav_msgs::msg::Path path_msg;
    path_msg.header = header;
    for (const auto & state : states) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = header;
      pose.pose.position.x = state.x;
      pose.pose.position.y = state.y;
      pose.pose.orientation = yawToQuaternion(state.yaw);
      path_msg.poses.push_back(pose);
    }

    path_pub_->publish(path_msg);
    marker_pub_->publish(makeMarkers(states, header));
    startBodyTrajectoryPlayback(states, playback_time);
  }

  void startBodyTrajectoryPlayback(const std::vector<State> & states, double playback_time)
  {
    body_trajectory_states_ = states;
    body_trajectory_index_ = 0;
    const double dt = states.size() > 1 ?
      playback_time / static_cast<double>(states.size() - 1) :
      0.1;
    body_playback_period_ = std::max(0.001, dt);
    body_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(body_playback_period_)),
      std::bind(&PlannerNode::advanceBodyTrajectoryMarker, this));
    publishBodyTrajectoryMarker();
  }

  void advanceBodyTrajectoryMarker()
  {
    if (!get_parameter("body_trajectory_visualization_enabled").as_bool()) {
      if (!body_marker_cleared_) {
        clearBodyTrajectoryMarker();
      }
      return;
    }
    if (body_trajectory_states_.empty()) {
      clearBodyTrajectoryMarker();
      return;
    }
    if (body_trajectory_index_ + 1 < body_trajectory_states_.size()) {
      ++body_trajectory_index_;
    }
    publishBodyTrajectoryMarker();
  }

  void clearBodyTrajectoryMarker()
  {
    std_msgs::msg::Header header;
    header.stamp = now();
    header.frame_id = map_frame_;
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker clear;
    clear.header = header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);
    body_marker_pub_->publish(array);
    body_marker_cleared_ = true;
  }

  void publishBodyTrajectoryMarker()
  {
    if (!get_parameter("body_trajectory_visualization_enabled").as_bool() ||
      body_trajectory_states_.empty())
    {
      clearBodyTrajectoryMarker();
      return;
    }

    std_msgs::msg::Header header;
    header.stamp = now();
    header.frame_id = map_frame_;
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker clear;
    clear.header = header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);

    const double marker_z = get_parameter("body_trajectory_marker_z").as_double();
    const double half_length = 0.5 * get_parameter("robot_length").as_double();
    const double half_width = 0.5 * get_parameter("robot_width").as_double();

    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = "fourwis_body_trajectory";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.04;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.85;
    marker.points = bodyCorners(
      body_trajectory_states_[body_trajectory_index_], half_length, half_width, marker_z);
    array.markers.push_back(marker);

    body_marker_pub_->publish(array);
    body_marker_cleared_ = false;
  }

  static std::vector<geometry_msgs::msg::Point> bodyCorners(
    const State & state, double half_length, double half_width, double z)
  {
    std::vector<geometry_msgs::msg::Point> points;
    points.reserve(5);
    const double c = std::cos(state.yaw);
    const double s = std::sin(state.yaw);
    const std::array<std::array<double, 2>, 5> local = {{
      {{half_length, half_width}},
      {{half_length, -half_width}},
      {{-half_length, -half_width}},
      {{-half_length, half_width}},
      {{half_length, half_width}},
    }};
    for (const auto & p : local) {
      geometry_msgs::msg::Point point;
      point.x = state.x + p[0] * c - p[1] * s;
      point.y = state.y + p[0] * s + p[1] * c;
      point.z = z;
      points.push_back(point);
    }
    return points;
  }

  void publishCorridors(const std::vector<SafeCorridor> & corridors)
  {
    std_msgs::msg::Header header;
    header.stamp = now();
    header.frame_id = map_frame_;

    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker clear;
    clear.header = header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);

    if (!get_parameter("backend_corridor_visualization_enabled").as_bool() || corridors.empty()) {
      corridor_marker_pub_->publish(array);
      return;
    }

    const int stride = std::max(1, static_cast<int>(
      get_parameter("backend_corridor_visualization_stride").as_int()));
    const double marker_z = get_parameter("backend_corridor_marker_z").as_double();
    int marker_id = 0;

    for (std::size_t i = 0; i < corridors.size(); i += static_cast<std::size_t>(stride)) {
      const auto & corridor = corridors[i];
      visualization_msgs::msg::Marker marker;
      marker.header = header;
      marker.ns = "fourwis_safe_corridors";
      marker.id = marker_id++;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.025;
      marker.color.r = 0.0;
      marker.color.g = 0.85;
      marker.color.b = 1.0;
      marker.color.a = 0.45;
      marker.points = corridorCorners(corridor, marker_z);
      array.markers.push_back(marker);
    }

    corridor_marker_pub_->publish(array);
  }

  static std::vector<geometry_msgs::msg::Point> corridorCorners(
    const SafeCorridor & corridor, double z)
  {
    std::vector<geometry_msgs::msg::Point> points;
    points.reserve(5);
    const std::array<std::array<double, 2>, 5> local = {{
      {{corridor.forward, corridor.left}},
      {{corridor.forward, -corridor.right}},
      {{-corridor.backward, -corridor.right}},
      {{-corridor.backward, corridor.left}},
      {{corridor.forward, corridor.left}},
    }};

    for (const auto & point_local : local) {
      geometry_msgs::msg::Point point;
      point.x =
        corridor.x + point_local[0] * corridor.tangent_x + point_local[1] * corridor.normal_x;
      point.y =
        corridor.y + point_local[0] * corridor.tangent_y + point_local[1] * corridor.normal_y;
      point.z = z;
      points.push_back(point);
    }
    return points;
  }

  visualization_msgs::msg::MarkerArray makeMarkers(
    const std::vector<State> & states, const std_msgs::msg::Header & header) const
  {
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker clear;
    clear.header = header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);
    if (states.size() < 2) {
      return array;
    }

    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = "fourwis_ackermann_path";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.035;
    marker.pose.orientation.w = 1.0;
    marker.color.r = 0.1;
    marker.color.g = 0.35;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    for (const auto & state : states) {
      geometry_msgs::msg::Point point;
      point.x = state.x;
      point.y = state.y;
      point.z = 0.13;
      marker.points.push_back(point);
    }
    array.markers.push_back(marker);
    return array;
  }

  State poseToState(const geometry_msgs::msg::Pose & pose)
  {
    State state;
    state.x = pose.position.x;
    state.y = pose.position.y;
    state.yaw = yawFromQuaternion(pose.orientation);
    return state;
  }

  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
  {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  static geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
  {
    geometry_msgs::msg::Quaternion q;
    q.z = std::sin(yaw / 2.0);
    q.w = std::cos(yaw / 2.0);
    return q;
  }

  void publishStatus(const std::string & text)
  {
    std_msgs::msg::String msg;
    msg.data = text;
    status_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "%s", text.c_str());
  }

  void publishPipelineStatus(const std::string & text, const std::array<double, 4> & color)
  {
    (void)color;
    status_text_ = text;
    std_msgs::msg::String msg;
    msg.data = status_text_;
    state_pub_->publish(msg);
  }

  void publishMetrics(const TrajectoryMetrics & metrics)
  {
    std::ostringstream text_stream;
    text_stream << std::fixed << std::setprecision(4);
    text_stream << "time=" << metrics.arrival_time
                << ";comfort=" << metrics.comfort
                << ";length=" << metrics.length
                << ";states=" << metrics.states;

    std_msgs::msg::String metrics_msg;
    metrics_msg.data = text_stream.str();
    metrics_text_pub_->publish(metrics_msg);
  }

  void republishStatusMarker()
  {
    std_msgs::msg::String msg;
    msg.data = status_text_;
    state_pub_->publish(msg);
  }

  void publishPoseMarkers()
  {
    visualization_msgs::msg::MarkerArray array;
    const auto stamp = now();

    visualization_msgs::msg::Marker clear;
    clear.header.stamp = stamp;
    clear.header.frame_id = map_frame_;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);

    if (has_start_) {
      addPoseArrow(array, start_, "start", 0, {0.0, 0.85, 0.15, 1.0}, "START");
    }
    if (has_goal_) {
      addPoseArrow(array, goal_, "goal", 10, {0.95, 0.05, 0.05, 1.0}, "GOAL");
    }
    pose_marker_pub_->publish(array);
  }

  void addPoseArrow(
    visualization_msgs::msg::MarkerArray & array, const State & state, const std::string & ns,
    int id_offset, const std::array<double, 4> & color, const std::string & label)
  {
    const double arrow_length = get_parameter("pose_arrow_length").as_double();
    const double arrow_width = get_parameter("pose_arrow_width").as_double();
    const double arrow_height = get_parameter("pose_arrow_height").as_double();
    const double marker_z = get_parameter("pose_marker_z").as_double();
    const auto stamp = now();

    visualization_msgs::msg::Marker arrow;
    arrow.header.stamp = stamp;
    arrow.header.frame_id = map_frame_;
    arrow.ns = "fourwis_" + ns + "_pose";
    arrow.id = id_offset;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.pose.position.x = state.x;
    arrow.pose.position.y = state.y;
    arrow.pose.position.z = marker_z;
    arrow.pose.orientation = yawToQuaternion(state.yaw);
    arrow.scale.x = arrow_length;
    arrow.scale.y = arrow_width;
    arrow.scale.z = arrow_height;
    arrow.color.r = color[0];
    arrow.color.g = color[1];
    arrow.color.b = color[2];
    arrow.color.a = color[3];
    array.markers.push_back(arrow);

    visualization_msgs::msg::Marker text;
    text.header.stamp = stamp;
    text.header.frame_id = map_frame_;
    text.ns = "fourwis_" + ns + "_pose";
    text.id = id_offset + 1;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose.position.x = state.x;
    text.pose.position.y = state.y;
    text.pose.position.z = marker_z + 0.55;
    text.pose.orientation.w = 1.0;
    text.scale.z = 0.34;
    text.color.r = color[0];
    text.color.g = color[1];
    text.color.b = color[2];
    text.color.a = 1.0;
    text.text = label;
    array.markers.push_back(text);
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr start_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr frontend_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr body_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr corridor_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pose_marker_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metrics_text_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr body_timer_;

  std::shared_ptr<OccupancyGrid> grid_;
  State start_;
  State goal_;
  std::vector<State> body_trajectory_states_;
  std::size_t body_trajectory_index_{0};
  double body_playback_period_{0.1};
  bool has_start_{false};
  bool has_goal_{false};
  bool body_marker_cleared_{true};
  std::string map_frame_{"map"};
  std::string status_text_;
};

}  // namespace fourwis_hybrid_astar_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<fourwis_hybrid_astar_cpp::PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
