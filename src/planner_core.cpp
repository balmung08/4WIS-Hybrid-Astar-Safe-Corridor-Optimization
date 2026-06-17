#include "fourwis_hybrid_astar_cpp/planner_core.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <queue>
#include <stdexcept>
#include <tuple>

#include <ompl/base/spaces/ReedsSheppStateSpace.h>

namespace fourwis_hybrid_astar_cpp
{

namespace
{
constexpr double kInf = std::numeric_limits<double>::infinity();

double distance2d(double x0, double y0, double x1, double y1)
{
  return std::hypot(x1 - x0, y1 - y0);
}

double elapsedMs(const std::chrono::steady_clock::time_point & start)
{
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

double normalizeSteerAround(double angle, double center)
{
  while (angle - center > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle - center <= -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}
}  // namespace

OccupancyGrid::OccupancyGrid(
  std::vector<int8_t> data, int width, int height, double resolution,
  double origin_x, double origin_y, int obstacle_threshold, bool unknown_is_obstacle)
: data_(std::move(data)),
  width_(width),
  height_(height),
  resolution_(resolution),
  origin_x_(origin_x),
  origin_y_(origin_y),
  obstacle_threshold_(obstacle_threshold),
  unknown_is_obstacle_(unknown_is_obstacle)
{
  if (width_ <= 0 || height_ <= 0 || static_cast<int>(data_.size()) != width_ * height_) {
    throw std::runtime_error("invalid occupancy grid dimensions");
  }
  computeObstacleDistanceField();
}

std::pair<int, int> OccupancyGrid::worldToGrid(double x, double y) const
{
  return {
    static_cast<int>(std::floor((x - origin_x_) / resolution_)),
    static_cast<int>(std::floor((y - origin_y_) / resolution_))};
}

std::pair<double, double> OccupancyGrid::gridToWorld(int gx, int gy) const
{
  return {
    origin_x_ + (static_cast<double>(gx) + 0.5) * resolution_,
    origin_y_ + (static_cast<double>(gy) + 0.5) * resolution_};
}

bool OccupancyGrid::inBounds(int gx, int gy) const
{
  return gx >= 0 && gy >= 0 && gx < width_ && gy < height_;
}

bool OccupancyGrid::occupied(int gx, int gy) const
{
  if (!inBounds(gx, gy)) {
    return true;
  }
  const int value = static_cast<int>(data_[gy * width_ + gx]);
  if (value < 0) {
    return unknown_is_obstacle_;
  }
  return value >= obstacle_threshold_;
}

double OccupancyGrid::obstacleDistanceGrid(int gx, int gy) const
{
  if (!inBounds(gx, gy)) {
    return 0.0;
  }
  return obstacle_distance_[gy * width_ + gx];
}

double OccupancyGrid::obstacleDistanceWorld(double x, double y) const
{
  const auto [gx, gy] = worldToGrid(x, y);
  return obstacleDistanceGrid(gx, gy);
}

void OccupancyGrid::computeObstacleDistanceField()
{
  struct Item
  {
    double distance;
    int x;
    int y;
    bool operator>(const Item & other) const { return distance > other.distance; }
  };

  obstacle_distance_.assign(width_ * height_, kInf);
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> queue;

  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      const int flat = y * width_ + x;
      const int value = static_cast<int>(data_[flat]);
      const bool is_unknown_obstacle = value < 0 && unknown_is_obstacle_;
      const bool is_obstacle = is_unknown_obstacle || value >= obstacle_threshold_;
      if (is_obstacle) {
        obstacle_distance_[flat] = 0.0;
        queue.push({0.0, x, y});
      }
    }
  }

  const std::array<std::tuple<int, int, double>, 8> dirs = {
    std::make_tuple(-1, -1, std::sqrt(2.0)), std::make_tuple(-1, 0, 1.0),
    std::make_tuple(-1, 1, std::sqrt(2.0)), std::make_tuple(0, -1, 1.0),
    std::make_tuple(0, 1, 1.0), std::make_tuple(1, -1, std::sqrt(2.0)),
    std::make_tuple(1, 0, 1.0), std::make_tuple(1, 1, std::sqrt(2.0))};

  while (!queue.empty()) {
    const auto item = queue.top();
    queue.pop();
    const int flat = item.y * width_ + item.x;
    if (item.distance > obstacle_distance_[flat]) {
      continue;
    }
    for (const auto & [dx, dy, step] : dirs) {
      const int nx = item.x + dx;
      const int ny = item.y + dy;
      if (!inBounds(nx, ny)) {
        continue;
      }
      const int nflat = ny * width_ + nx;
      const double new_distance = item.distance + step * resolution_;
      if (new_distance < obstacle_distance_[nflat]) {
        obstacle_distance_[nflat] = new_distance;
        queue.push({new_distance, nx, ny});
      }
    }
  }
}

std::size_t AckermannHybridAStar::GridIndexHash::operator()(const GridIndex & idx) const noexcept
{
  std::size_t seed = 1469598103934665603ull;
  auto mix = [&seed](int value) {
    seed ^= static_cast<std::size_t>(value + 0x9e3779b9);
    seed *= 1099511628211ull;
  };
  mix(idx.x);
  mix(idx.y);
  mix(idx.yaw);
  return seed;
}

bool AckermannHybridAStar::GridIndexEq::operator()(const GridIndex & a, const GridIndex & b) const noexcept
{
  return a.x == b.x && a.y == b.y && a.yaw == b.yaw;
}

AckermannHybridAStar::DistanceHeuristic::DistanceHeuristic(
  const OccupancyGrid & grid, double resolution)
: grid_(grid), resolution_(resolution)
{
  scale_ = std::max(1, static_cast<int>(std::round(resolution_ / grid_.resolution())));
  width_ = static_cast<int>(std::ceil(static_cast<double>(grid_.width()) / scale_));
  height_ = static_cast<int>(std::ceil(static_cast<double>(grid_.height()) / scale_));
  dist_.assign(width_ * height_, kInf);
}

std::pair<int, int> AckermannHybridAStar::DistanceHeuristic::worldToGrid(double x, double y) const
{
  const auto [gx, gy] = grid_.worldToGrid(x, y);
  return {gx / scale_, gy / scale_};
}

bool AckermannHybridAStar::DistanceHeuristic::isFree(int gx, int gy) const
{
  if (gx < 0 || gy < 0 || gx >= width_ || gy >= height_) {
    return false;
  }
  const int min_x = gx * scale_;
  const int min_y = gy * scale_;
  const int max_x = std::min((gx + 1) * scale_, grid_.width());
  const int max_y = std::min((gy + 1) * scale_, grid_.height());
  for (int y = min_y; y < max_y; ++y) {
    for (int x = min_x; x < max_x; ++x) {
      if (grid_.occupied(x, y)) {
        return false;
      }
    }
  }
  return true;
}

void AckermannHybridAStar::DistanceHeuristic::compute(double goal_x, double goal_y)
{
  const auto [goal_gx, goal_gy] = worldToGrid(goal_x, goal_y);
  if (goal_gx < 0 || goal_gy < 0 || goal_gx >= width_ || goal_gy >= height_) {
    return;
  }

  struct Item
  {
    double cost;
    int x;
    int y;
    bool operator>(const Item & other) const { return cost > other.cost; }
  };
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> queue;
  dist_[goal_gy * width_ + goal_gx] = 0.0;
  queue.push({0.0, goal_gx, goal_gy});

  const std::array<std::tuple<int, int, double>, 8> dirs = {
    std::make_tuple(-1, -1, std::sqrt(2.0)), std::make_tuple(-1, 0, 1.0),
    std::make_tuple(-1, 1, std::sqrt(2.0)), std::make_tuple(0, -1, 1.0),
    std::make_tuple(0, 1, 1.0), std::make_tuple(1, -1, std::sqrt(2.0)),
    std::make_tuple(1, 0, 1.0), std::make_tuple(1, 1, std::sqrt(2.0))};

  std::vector<uint8_t> visited(width_ * height_, 0);
  while (!queue.empty()) {
    const auto item = queue.top();
    queue.pop();
    const int flat = item.y * width_ + item.x;
    if (visited[flat]) {
      continue;
    }
    visited[flat] = 1;

    for (const auto & [dx, dy, step] : dirs) {
      const int nx = item.x + dx;
      const int ny = item.y + dy;
      if (!isFree(nx, ny)) {
        continue;
      }
      const int nflat = ny * width_ + nx;
      const double new_cost = item.cost + step * resolution_;
      if (new_cost < dist_[nflat]) {
        dist_[nflat] = new_cost;
        queue.push({new_cost, nx, ny});
      }
    }
  }
}

double AckermannHybridAStar::DistanceHeuristic::get(double x, double y) const
{
  const auto [gx, gy] = worldToGrid(x, y);
  if (gx < 0 || gy < 0 || gx >= width_ || gy >= height_) {
    return kInf;
  }
  return dist_[gy * width_ + gx];
}

AckermannHybridAStar::AckermannHybridAStar(OccupancyGrid grid, PlannerConfig config)
: grid_(std::move(grid)), config_(config)
{
}

bool AckermannHybridAStar::plan(const State & start, const State & goal, PlanningResult & result)
{
  TimingStats timing;
  const auto total_start = std::chrono::steady_clock::now();
  auto stage_start = std::chrono::steady_clock::now();
  if (collidesPose(start.x, start.y, start.yaw) || collidesPose(goal.x, goal.y, goal.yaw)) {
    timing.start_goal_collision_ms = elapsedMs(stage_start);
    timing.total_ms = elapsedMs(total_start);
    result.timing = timing;
    return false;
  }
  timing.start_goal_collision_ms = elapsedMs(stage_start);

  stage_start = std::chrono::steady_clock::now();
  DistanceHeuristic heuristic_map(grid_, config_.heuristic_resolution);
  heuristic_map.compute(goal.x, goal.y);
  timing.heuristic_map_ms = elapsedMs(stage_start);

  Node start_node;
  start_node.state = start;
  start_node.state.yaw = normAngle(start_node.state.yaw);
  start_node.g = 0.0;

  Node goal_node;
  goal_node.state = goal;
  goal_node.state.yaw = normAngle(goal_node.state.yaw);

  struct QueueItem
  {
    double f;
    int id;
    bool operator>(const QueueItem & other) const { return f > other.f; }
  };
  std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> open;
  std::vector<Node> nodes;
  nodes.reserve(static_cast<std::size_t>(std::min(config_.max_iterations, 500000)));
  nodes.push_back(start_node);

  std::unordered_map<GridIndex, double, GridIndexHash, GridIndexEq> best;
  std::unordered_set<GridIndex, GridIndexHash, GridIndexEq> closed;
  best[index(start_node)] = 0.0;
  nodes[0].h = heuristic(nodes[0], goal_node, heuristic_map);
  open.push({nodes[0].g + nodes[0].h, 0});

  const auto search_start = std::chrono::steady_clock::now();
  for (int iteration = 1; iteration <= config_.max_iterations; ++iteration) {
    if (open.empty()) {
      timing.search_loop_ms = elapsedMs(search_start);
      timing.total_ms = elapsedMs(total_start);
      result.timing = timing;
      return false;
    }

    const auto item = open.top();
    open.pop();
    const Node current = nodes[item.id];
    const GridIndex current_index = index(current);
    if (closed.find(current_index) != closed.end()) {
      continue;
    }
    closed.insert(current_index);
    timing.expanded_nodes++;

    if (goalReached(current, goal_node)) {
      const auto reconstruct_start = std::chrono::steady_clock::now();
      result = reconstruct(nodes, item.id, iteration);
      timing.reconstruction_ms = elapsedMs(reconstruct_start);
      timing.search_loop_ms = elapsedMs(search_start);
      timing.total_ms = elapsedMs(total_start);
      result.timing = timing;
      return true;
    }

    if (heuristic_map.get(current.state.x, current.state.y) <= config_.analytic_expansion_distance) {
      Node shot;
      timing.analytic_attempts++;
      const auto analytic_start = std::chrono::steady_clock::now();
      if (analyticExpansion(current, item.id, goal_node, shot)) {
        timing.analytic_expansion_ms += elapsedMs(analytic_start);
        shot.parent = item.id;
        shot.h = 0.0;
        const GridIndex shot_index = index(shot);
        const auto found = best.find(shot_index);
        if (
          closed.find(shot_index) == closed.end() &&
          (found == best.end() || shot.g < found->second))
        {
          best[shot_index] = shot.g;
          nodes.push_back(std::move(shot));
          const int shot_id = static_cast<int>(nodes.size() - 1);
          open.push({nodes[shot_id].g, shot_id});
          timing.accepted_successors++;
        }
      } else {
        timing.analytic_expansion_ms += elapsedMs(analytic_start);
      }
    }

    std::vector<Node> successors;
    const auto successor_start = std::chrono::steady_clock::now();
    addSuccessors(current, item.id, successors);
    timing.successor_generation_ms += elapsedMs(successor_start);
    timing.generated_successors += static_cast<int>(successors.size());
    for (auto & successor : successors) {
      const GridIndex succ_idx = index(successor);
      if (closed.find(succ_idx) != closed.end()) {
        continue;
      }
      const auto found = best.find(succ_idx);
      if (found != best.end() && successor.g >= found->second) {
        continue;
      }
      successor.h = heuristic(successor, goal_node, heuristic_map);
      best[succ_idx] = successor.g;
      nodes.push_back(std::move(successor));
      const int id = static_cast<int>(nodes.size() - 1);
      open.push({nodes[id].g + nodes[id].h, id});
      timing.accepted_successors++;
    }
  }
  timing.search_loop_ms = elapsedMs(search_start);
  timing.total_ms = elapsedMs(total_start);
  result.timing = timing;
  return false;
}

void AckermannHybridAStar::addSuccessors(
  const Node & current, int current_id, std::vector<Node> & successors) const
{
  const std::array<int, 2> directions = {1, -1};
  const std::array<double, 3> steers = {-config_.max_steer, 0.0, config_.max_steer};

  for (const int direction : directions) {
    for (const double steer : steers) {
      std::vector<State> segment = rollout(current, direction, steer);
      if (segment.empty()) {
        continue;
      }
      Node successor;
      successor.state = segment.back();
      successor.parent = current_id;
      successor.segment = std::move(segment);
      successor.direction = direction;
      successor.steer = steer;
      double cost = segmentLength(successor.segment);
      if (direction < 0) {
        cost *= config_.reverse_penalty;
      }
      cost += std::abs(steer) * config_.steer_penalty;
      cost += std::abs(normalizeSteerAround(steer - current.steer, 0.0)) *
        config_.steer_change_penalty;
      if (direction != current.direction) {
        cost += config_.direction_change_penalty;
      }
      successor.g = current.g + cost;
      successors.push_back(std::move(successor));
    }
  }
}

std::vector<State> AckermannHybridAStar::rollout(
  const Node & current, int direction, double steer) const
{
  std::vector<State> segment;
  State state = current.state;
  const int steps = std::max(1, static_cast<int>(std::round(config_.sampling_time / config_.integration_dt)));
  const double v = static_cast<double>(direction) * config_.reference_velocity;
  segment.reserve(static_cast<std::size_t>(steps + 1));

  for (int i = 0; i < steps; ++i) {
    segment.push_back(state);
    if (collidesPose(state.x, state.y, state.yaw)) {
      return {};
    }
    state.x += v * std::cos(state.yaw) * config_.integration_dt;
    state.y += v * std::sin(state.yaw) * config_.integration_dt;
    state.yaw += 2.0 * v * std::tan(steer) / config_.wheelbase * config_.integration_dt;
    state.yaw = normAngle(state.yaw);
  }
  segment.push_back(state);
  if (collidesPose(state.x, state.y, state.yaw)) {
    return {};
  }
  return segment;
}

bool AckermannHybridAStar::analyticExpansion(
  const Node & current, int current_id, const Node & goal, Node & out) const
{
  (void)current_id;
  std::vector<State> segment;
  int shot_direction = 1;
  double terminal_steer = 0.0;
  const bool connected =
    omplReedsSheppShot(current, goal, segment, shot_direction, terminal_steer);
  if (!connected) {
    return false;
  }

  double cost = segmentLength(segment);
  if (shot_direction < 0) {
    cost *= config_.reverse_penalty;
  }
  cost += yawAbsDiff(current.state.yaw, goal.state.yaw) / std::max(maxCurvature(), 1e-6);

  out.state = goal.state;
  out.parent = current_id;
  out.segment = std::move(segment);
  out.direction = shot_direction;
  out.steer = terminal_steer;
  out.g = current.g + cost;
  return true;
}

bool AckermannHybridAStar::omplReedsSheppShot(
  const Node & current, const Node & goal, std::vector<State> & segment,
  int & direction, double & terminal_steer) const
{
  const double curvature = maxCurvature();
  if (curvature <= 1e-9) {
    return false;
  }

  namespace ob = ompl::base;
  ob::ReedsSheppStateSpace space(1.0 / curvature);
  ob::State * from = space.allocState();
  ob::State * to = space.allocState();
  ob::State * sample = space.allocState();

  auto * from_se2 = from->as<ob::SE2StateSpace::StateType>();
  auto * to_se2 = to->as<ob::SE2StateSpace::StateType>();
  from_se2->setXY(current.state.x, current.state.y);
  from_se2->setYaw(normAngle(current.state.yaw + current.steer));
  to_se2->setXY(goal.state.x, goal.state.y);
  to_se2->setYaw(goal.state.yaw);

  const double length = space.distance(from, to);
  const int samples = std::max(2, static_cast<int>(std::ceil(length / config_.xy_resolution)) + 1);
  segment.clear();
  segment.reserve(static_cast<std::size_t>(samples));

  bool first_time = true;
  auto path = space.reedsShepp(from, to);
  for (int i = 0; i < samples; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(samples - 1);
    space.interpolate(from, to, t, first_time, path, sample);
    const auto * se2 = sample->as<ob::SE2StateSpace::StateType>();
    State state;
    state.x = se2->getX();
    state.y = se2->getY();
    state.yaw = normAngle(se2->getYaw());
    segment.push_back(state);
  }

  space.freeState(from);
  space.freeState(to);
  space.freeState(sample);
  direction = 1;
  if (segment.size() > 1) {
    const State & a = segment.front();
    const State & b = segment[1];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    direction = dx * std::cos(current.state.yaw + current.steer) +
        dy * std::sin(current.state.yaw + current.steer) >= 0.0 ? 1 : -1;
  }
  int final_direction = direction;
  if (!estimateSegmentControl(segment, false, final_direction, terminal_steer)) {
    terminal_steer = 0.0;
  }
  return !collidesSegment(segment);
}

bool AckermannHybridAStar::estimateSegmentControl(
  const std::vector<State> & segment, bool from_start, int & direction, double & steer) const
{
  if (segment.size() < 2) {
    return false;
  }
  const std::size_t anchor = from_start ? 0 : segment.size() - 1;
  const int search_sign = from_start ? 1 : -1;
  for (std::size_t step = 1; step < segment.size(); ++step) {
    const std::size_t other_index = static_cast<std::size_t>(
      static_cast<int>(anchor) + search_sign * static_cast<int>(step));
    if (other_index >= segment.size()) {
      break;
    }
    const State & a = segment[anchor];
    const State & b = segment[other_index];
    const double dx = from_start ? b.x - a.x : a.x - b.x;
    const double dy = from_start ? b.y - a.y : a.y - b.y;
    const double ds = std::hypot(dx, dy);
    if (ds < 1e-6) {
      continue;
    }
    direction = (dx * std::cos(a.yaw) + dy * std::sin(a.yaw)) >= 0.0 ? 1 : -1;
    const double dyaw = from_start ? normAngle(b.yaw - a.yaw) : normAngle(a.yaw - b.yaw);
    const double signed_curvature = dyaw / ds;
    steer = std::atan(0.5 * signed_curvature * config_.wheelbase * static_cast<double>(direction));
    steer = std::clamp(steer, -config_.max_steer, config_.max_steer);
    return true;
  }
  return false;
}

bool AckermannHybridAStar::steeringContinuous(const Node & current, double initial_steer) const
{
  return std::abs(normalizeSteerAround(initial_steer - current.steer, 0.0)) <= M_PI / 180.0;
}

double AckermannHybridAStar::heuristic(
  const Node & current, const Node & goal, const DistanceHeuristic & heuristic_map) const
{
  const double euclidean = distance2d(current.state.x, current.state.y, goal.state.x, goal.state.y);
  double grid_dist = heuristic_map.get(current.state.x, current.state.y);
  if (!std::isfinite(grid_dist)) {
    grid_dist = 0.0;
  }
  const double rs = rsHeuristicCost(current, goal);
  return std::max({euclidean, grid_dist, rs});
}

double AckermannHybridAStar::rsHeuristicCost(const Node & current, const Node & goal) const
{
  const double curvature = maxCurvature();
  if (curvature <= 1e-9) {
    return kInf;
  }

  namespace ob = ompl::base;
  ob::ReedsSheppStateSpace space(1.0 / curvature);
  ob::State * from = space.allocState();
  ob::State * to = space.allocState();
  auto * from_se2 = from->as<ob::SE2StateSpace::StateType>();
  auto * to_se2 = to->as<ob::SE2StateSpace::StateType>();
  from_se2->setXY(current.state.x, current.state.y);
  from_se2->setYaw(normAngle(current.state.yaw + current.steer));
  to_se2->setXY(goal.state.x, goal.state.y);
  to_se2->setYaw(goal.state.yaw);
  const double cost = space.distance(from, to);
  space.freeState(from);
  space.freeState(to);
  return cost;
}

bool AckermannHybridAStar::collidesPose(double x, double y, double yaw) const
{
  const double radius =
    std::hypot(config_.robot_length / 4.0, config_.robot_width / 2.0) +
    config_.collision_clearance;
  for (const double sign : {1.0, -1.0}) {
    const double cx = x + sign * config_.robot_length * std::cos(yaw) / 4.0;
    const double cy = y + sign * config_.robot_length * std::sin(yaw) / 4.0;
    if (circleCollides(cx, cy, radius)) {
      return true;
    }
  }
  return false;
}

bool AckermannHybridAStar::circleCollides(double cx, double cy, double radius) const
{
  return grid_.obstacleDistanceWorld(cx, cy) <= radius;
}

bool AckermannHybridAStar::collidesSegment(const std::vector<State> & segment) const
{
  for (const auto & state : segment) {
    if (collidesPose(state.x, state.y, state.yaw)) {
      return true;
    }
  }
  return false;
}

double AckermannHybridAStar::segmentLength(const std::vector<State> & segment) const
{
  double length = 0.0;
  for (std::size_t i = 1; i < segment.size(); ++i) {
    length += distance2d(segment[i - 1].x, segment[i - 1].y, segment[i].x, segment[i].y);
  }
  return length;
}

double AckermannHybridAStar::maxCurvature() const
{
  return 2.0 * std::tan(config_.max_steer) / config_.wheelbase;
}

bool AckermannHybridAStar::goalReached(const Node & current, const Node & goal) const
{
  return distance2d(current.state.x, current.state.y, goal.state.x, goal.state.y) <=
           config_.goal_distance_tolerance &&
         yawAbsDiff(current.state.yaw, goal.state.yaw) <= config_.goal_yaw_tolerance;
}

AckermannHybridAStar::GridIndex AckermannHybridAStar::index(const Node & node) const
{
  const int yaw_bins = static_cast<int>(std::round(2.0 * M_PI / config_.yaw_resolution));
  int yaw_idx = static_cast<int>(std::round(normAngle(node.state.yaw) / config_.yaw_resolution));
  yaw_idx = ((yaw_idx % yaw_bins) + yaw_bins) % yaw_bins;
  return {
    static_cast<int>(std::round((node.state.x - grid_.originX()) / config_.xy_resolution)),
    static_cast<int>(std::round((node.state.y - grid_.originY()) / config_.xy_resolution)),
    yaw_idx};
}

PlanningResult AckermannHybridAStar::reconstruct(
  const std::vector<Node> & nodes, int goal_id, int iterations) const
{
  std::vector<int> chain;
  for (int id = goal_id; id >= 0; id = nodes[id].parent) {
    chain.push_back(id);
    if (nodes[id].parent == id) {
      break;
    }
  }
  std::reverse(chain.begin(), chain.end());

  PlanningResult result;
  result.cost = nodes[goal_id].g;
  result.iterations = iterations;
  for (std::size_t i = 0; i < chain.size(); ++i) {
    const Node & node = nodes[chain[i]];
    if (i == 0 || node.segment.empty()) {
      result.states.push_back(node.state);
      continue;
    }
    for (std::size_t k = 0; k < node.segment.size(); ++k) {
      if (!result.states.empty() && k == 0) {
        const auto & last = result.states.back();
        const auto & first = node.segment.front();
        if (
          std::abs(last.x - first.x) < 1e-6 && std::abs(last.y - first.y) < 1e-6 &&
          std::abs(last.yaw - first.yaw) < 1e-6)
        {
          continue;
        }
      }
      result.states.push_back(node.segment[k]);
    }
  }
  return result;
}

double AckermannHybridAStar::normAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle <= -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double AckermannHybridAStar::yawAbsDiff(double a, double b)
{
  return std::abs(normAngle(a - b));
}

}  // namespace fourwis_hybrid_astar_cpp
