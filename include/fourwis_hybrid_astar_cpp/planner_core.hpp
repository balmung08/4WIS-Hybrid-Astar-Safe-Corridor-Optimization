#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace fourwis_hybrid_astar_cpp
{

struct PlannerConfig
{
  double xy_resolution{0.2};
  double yaw_resolution{M_PI / 12.0};
  double heuristic_resolution{0.2};
  double analytic_expansion_distance{5.0};
  double goal_distance_tolerance{0.25};
  double goal_yaw_tolerance{M_PI / 18.0};
  int max_iterations{200000};
  double integration_dt{0.05};
  double sampling_time{0.5};
  double reference_velocity{1.0};
  double wheelbase{0.68};
  double robot_length{1.0};
  double robot_width{0.62};
  double collision_clearance{0.0};
  double max_steer{M_PI / 6.0};
  double reverse_penalty{2.0};
  double steer_penalty{1.0};
  double steer_change_penalty{1.0};
  double direction_change_penalty{1.0};
  int obstacle_threshold{50};
  bool unknown_is_obstacle{true};
};

struct State
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct TimingStats
{
  double total_ms{0.0};
  double start_goal_collision_ms{0.0};
  double heuristic_map_ms{0.0};
  double search_loop_ms{0.0};
  double analytic_expansion_ms{0.0};
  double successor_generation_ms{0.0};
  double reconstruction_ms{0.0};
  int expanded_nodes{0};
  int generated_successors{0};
  int accepted_successors{0};
  int analytic_attempts{0};
};

struct PlanningResult
{
  std::vector<State> states;
  double cost{0.0};
  int iterations{0};
  TimingStats timing;
};

class OccupancyGrid
{
public:
  OccupancyGrid() = default;
  OccupancyGrid(
    std::vector<int8_t> data, int width, int height, double resolution,
    double origin_x, double origin_y, int obstacle_threshold, bool unknown_is_obstacle);

  std::pair<int, int> worldToGrid(double x, double y) const;
  std::pair<double, double> gridToWorld(int gx, int gy) const;
  bool inBounds(int gx, int gy) const;
  bool occupied(int gx, int gy) const;
  double obstacleDistanceGrid(int gx, int gy) const;
  double obstacleDistanceWorld(double x, double y) const;
  int width() const { return width_; }
  int height() const { return height_; }
  double resolution() const { return resolution_; }
  double originX() const { return origin_x_; }
  double originY() const { return origin_y_; }

private:
  void computeObstacleDistanceField();

  std::vector<int8_t> data_;
  std::vector<double> obstacle_distance_;
  int width_{0};
  int height_{0};
  double resolution_{0.1};
  double origin_x_{0.0};
  double origin_y_{0.0};
  int obstacle_threshold_{50};
  bool unknown_is_obstacle_{true};
};

class AckermannHybridAStar
{
public:
  AckermannHybridAStar(OccupancyGrid grid, PlannerConfig config);
  bool plan(const State & start, const State & goal, PlanningResult & result);

private:
  struct Node
  {
    State state;
    double g{std::numeric_limits<double>::infinity()};
    double h{0.0};
    int parent{-1};
    int direction{1};
    double steer{0.0};
    std::vector<State> segment;
  };

  struct GridIndex
  {
    int x{0};
    int y{0};
    int yaw{0};
  };

  struct GridIndexHash
  {
    std::size_t operator()(const GridIndex & idx) const noexcept;
  };

  struct GridIndexEq
  {
    bool operator()(const GridIndex & a, const GridIndex & b) const noexcept;
  };

  class DistanceHeuristic
  {
  public:
    DistanceHeuristic(const OccupancyGrid & grid, double resolution);
    void compute(double goal_x, double goal_y);
    double get(double x, double y) const;

  private:
    std::pair<int, int> worldToGrid(double x, double y) const;
    bool isFree(int gx, int gy) const;

    const OccupancyGrid & grid_;
    double resolution_{0.2};
    int scale_{1};
    int width_{0};
    int height_{0};
    std::vector<double> dist_;
  };

  bool collidesPose(double x, double y, double yaw) const;
  bool circleCollides(double cx, double cy, double radius) const;
  bool collidesSegment(const std::vector<State> & segment) const;
  std::vector<State> rollout(const Node & current, int direction, double steer) const;
  void addSuccessors(const Node & current, int current_id, std::vector<Node> & successors) const;
  bool analyticExpansion(const Node & current, int current_id, const Node & goal, Node & out) const;
  bool omplReedsSheppShot(
    const Node & current, const Node & goal, std::vector<State> & segment,
    int & direction, double & terminal_steer) const;
  bool estimateSegmentControl(
    const std::vector<State> & segment, bool from_start, int & direction, double & steer) const;
  bool steeringContinuous(const Node & current, double initial_steer) const;
  double heuristic(const Node & current, const Node & goal, const DistanceHeuristic & heuristic_map) const;
  double rsHeuristicCost(const Node & current, const Node & goal) const;
  double segmentLength(const std::vector<State> & segment) const;
  double maxCurvature() const;
  bool goalReached(const Node & current, const Node & goal) const;
  GridIndex index(const Node & node) const;
  PlanningResult reconstruct(const std::vector<Node> & nodes, int goal_id, int iterations) const;
  static double normAngle(double angle);
  static double yawAbsDiff(double a, double b);

  OccupancyGrid grid_;
  PlannerConfig config_;
};

}  // namespace fourwis_hybrid_astar_cpp
