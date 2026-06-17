#include "fourwis_hybrid_astar_cpp/safe_corridor_optimizer.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <string>

namespace fourwis_hybrid_astar_cpp
{

namespace
{
double elapsedMs(const std::chrono::steady_clock::time_point & start)
{
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

double distance2d(const State & a, const State & b)
{
  return std::hypot(b.x - a.x, b.y - a.y);
}

}  // namespace

SafeCorridorOptimizer::SafeCorridorOptimizer(
  const OccupancyGrid & grid, const PlannerConfig & planner_config,
  SafeCorridorConfig optimizer_config)
: grid_(grid), planner_config_(planner_config), optimizer_config_(optimizer_config)
{
  corridor_circle_radius_ =
    std::hypot(planner_config.robot_length / 4.0, planner_config.robot_width / 2.0) +
    planner_config.collision_clearance;
}

OptimizerResult SafeCorridorOptimizer::optimize(const std::vector<State> & input) const
{
  OptimizerResult result;
  const auto total_start = std::chrono::steady_clock::now();
  if (input.size() < 3) {
    result.states = input;
    result.stats.total_ms = elapsedMs(total_start);
    return result;
  }

  std::vector<State> states = resample(input);
  const std::vector<State> initial_reference = states;

  const int backend_iterations = std::max(1, optimizer_config_.max_iterations);
  const double convergence_tol = std::max(optimizer_config_.infeasibility_tolerance, 1e-12);
  for (int iter = 0; iter < backend_iterations; ++iter) {
    const std::vector<State> reference = states;
    const std::vector<SafeCorridor> corridors = buildCorridors(reference);
    const std::vector<SafeCorridor> front_corridors =
      buildCorridors(circleCenterStates(reference, 1.0));
    const std::vector<SafeCorridor> rear_corridors =
      buildCorridors(circleCenterStates(reference, -1.0));
    result.corridors = corridors;
    for (const auto & corridor : corridors) {
      result.stats.max_corridor_length = std::max(
        result.stats.max_corridor_length, corridor.forward + corridor.backward);
      result.stats.max_corridor_width = std::max(
        result.stats.max_corridor_width, corridor.left + corridor.right);
    }

    double optimized_time = 0.0;
    bool solve_success = false;
    int status_code = 0;
    std::vector<State> casadi_states = solveCasadiBackend(
      reference, corridors, front_corridors, rear_corridors, optimized_time, solve_success,
      status_code);
    result.stats.optimized_time = optimized_time;
    result.stats.solve_success = solve_success;
    result.stats.ipopt_status = status_code;
    result.stats.iterations++;
    if (!solve_success || casadi_states.size() != reference.size()) {
      break;
    }

    double iteration_max_delta = 0.0;
    for (std::size_t i = 0; i < casadi_states.size(); ++i) {
      iteration_max_delta = std::max(iteration_max_delta, distance2d(casadi_states[i], reference[i]));
    }
    states = std::move(casadi_states);
    if (iteration_max_delta <= convergence_tol) {
      break;
    }
  }

  result.states = std::move(states);

  if (result.states.size() == initial_reference.size() && !result.states.empty()) {
    double sum_delta = 0.0;
    for (std::size_t i = 0; i < result.states.size(); ++i) {
      const double delta = distance2d(result.states[i], initial_reference[i]);
      result.stats.max_position_delta = std::max(result.stats.max_position_delta, delta);
      sum_delta += delta;
    }
    result.stats.mean_position_delta = sum_delta / static_cast<double>(result.states.size());
  }
  result.stats.total_ms = elapsedMs(total_start);
  return result;
}

std::vector<State> SafeCorridorOptimizer::resample(const std::vector<State> & input) const
{
  if (input.size() < 2) {
    return input;
  }

  const double target_distance = std::max(optimizer_config_.resample_distance, 1e-3);
  double input_length = 0.0;
  for (std::size_t i = 1; i < input.size(); ++i) {
    input_length += distance2d(input[i - 1], input[i]);
  }

  std::vector<State> output;
  output.reserve(std::max<std::size_t>(
    2, static_cast<std::size_t>(std::ceil(input_length / target_distance)) + input.size() / 8));

  auto appendState = [&output](const State & state) {
    if (
      !output.empty() && distance2d(output.back(), state) < 1e-9 &&
      std::abs(SafeCorridorOptimizer::normAngle(output.back().yaw - state.yaw)) < 1e-9)
    {
      return;
    }
    output.push_back(state);
  };

  auto intervalDirection = [](const State & a, const State & b) {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    if (std::hypot(dx, dy) < 1e-9) {
      return 0;
    }
    return dx * std::cos(a.yaw) + dy * std::sin(a.yaw) >= 0.0 ? 1 : -1;
  };

  auto resampleChunk = [&](std::size_t begin, std::size_t end) {
    if (begin >= end) {
      appendState(input[begin]);
      return;
    }

    std::vector<double> cumulative(end - begin + 1, 0.0);
    for (std::size_t i = begin + 1; i <= end; ++i) {
      cumulative[i - begin] = cumulative[i - begin - 1] + distance2d(input[i - 1], input[i]);
    }
    const double total_length = cumulative.back();
    if (total_length < 1e-9) {
      appendState(input[begin]);
      appendState(input[end]);
      return;
    }

    const int intervals = std::max(1, static_cast<int>(std::round(total_length / target_distance)));
    appendState(input[begin]);
    std::size_t source = begin;
    for (int sample = 1; sample < intervals; ++sample) {
      const double arc = total_length * static_cast<double>(sample) / static_cast<double>(intervals);
      while (source + 1 < end && cumulative[source + 1 - begin] < arc) {
        ++source;
      }
      const double segment_start = cumulative[source - begin];
      const double segment_length = cumulative[source + 1 - begin] - segment_start;
      const double t = segment_length > 1e-9 ? (arc - segment_start) / segment_length : 0.0;
      State state;
      state.x = input[source].x + t * (input[source + 1].x - input[source].x);
      state.y = input[source].y + t * (input[source + 1].y - input[source].y);
      state.yaw = interpolateAngle(input[source].yaw, input[source + 1].yaw, t);
      appendState(state);
    }
    appendState(input[end]);
  };

  std::size_t chunk_begin = 0;
  int previous_direction = 0;
  for (std::size_t i = 0; i + 1 < input.size(); ++i) {
    const int direction = intervalDirection(input[i], input[i + 1]);
    if (previous_direction != 0 && direction != 0 && direction != previous_direction) {
      resampleChunk(chunk_begin, i);
      chunk_begin = i;
    }
    if (direction != 0) {
      previous_direction = direction;
    }
  }
  resampleChunk(chunk_begin, input.size() - 1);
  return output;
}

std::vector<State> SafeCorridorOptimizer::solveCasadiBackend(
  const std::vector<State> & reference, const std::vector<SafeCorridor> & corridors,
  const std::vector<SafeCorridor> & front_corridors,
  const std::vector<SafeCorridor> & rear_corridors,
  double & optimized_time, bool & solve_success, int & status_code) const
{
  optimized_time = 0.0;
  solve_success = false;
  status_code = -100;
  const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
  const std::string base = "/tmp/fourwis_casadi_" + std::to_string(stamp);
  const std::string input_path = base + ".in";
  const std::string output_path = base + ".out";
  const std::string log_path = base + ".log";

  if (!writeCasadiInput(input_path, reference, corridors, front_corridors, rear_corridors)) {
    status_code = -101;
    return reference;
  }

  const std::string command =
    optimizer_config_.casadi_python + " " + optimizer_config_.casadi_script + " " +
    input_path + " " + output_path + " > " + log_path + " 2>&1";
  const int ret = std::system(command.c_str());
  if (ret != 0) {
    status_code = ret;
    std::remove(input_path.c_str());
    std::remove(output_path.c_str());
    std::remove(log_path.c_str());
    return reference;
  }

  std::vector<State> states;
  if (!readCasadiOutput(output_path, states, optimized_time, solve_success, status_code)) {
    status_code = -102;
    states = reference;
  }
  std::remove(input_path.c_str());
  std::remove(output_path.c_str());
  std::remove(log_path.c_str());
  return states;
}

bool SafeCorridorOptimizer::writeCasadiInput(
  const std::string & path, const std::vector<State> & reference,
  const std::vector<SafeCorridor> & corridors,
  const std::vector<SafeCorridor> & front_corridors,
  const std::vector<SafeCorridor> & rear_corridors) const
{
  std::ofstream out(path);
  if (!out) {
    return false;
  }
  out << std::setprecision(17);
  out << "CONFIG "
      << planner_config_.wheelbase << " " << planner_config_.max_steer << " "
      << optimizer_config_.max_velocity << " " << optimizer_config_.max_acceleration << " "
      << optimizer_config_.comfort_weight << " " << optimizer_config_.constraint_penalty << " "
      << optimizer_config_.ipopt_max_iterations << " " << optimizer_config_.ipopt_tol << " "
      << optimizer_config_.infeasibility_tolerance << " " << planner_config_.robot_length << "\n";
  out << "POINTS " << reference.size() << "\n";
  for (const auto & state : reference) {
    out << state.x << " " << state.y << " " << state.yaw << "\n";
  }
  auto writeCorridors = [&](const char * name, const std::vector<SafeCorridor> & values) {
    out << name << " " << values.size() << "\n";
    for (const auto & corridor : values) {
      out << corridor.x << " " << corridor.y << " "
          << corridor.tangent_x << " " << corridor.tangent_y << " "
          << corridor.normal_x << " " << corridor.normal_y << " "
          << corridor.backward << " " << corridor.forward << " "
          << corridor.right << " " << corridor.left << "\n";
    }
  };
  writeCorridors("CENTER_CORRIDORS", corridors);
  writeCorridors("FRONT_CORRIDORS", front_corridors);
  writeCorridors("REAR_CORRIDORS", rear_corridors);
  return true;
}

bool SafeCorridorOptimizer::readCasadiOutput(
  const std::string & path, std::vector<State> & states, double & optimized_time,
  bool & solve_success, int & status_code) const
{
  std::ifstream in(path);
  if (!in) {
    return false;
  }
  std::string tag;
  in >> tag;
  if (tag != "STATUS") {
    return false;
  }
  int success = 0;
  in >> success >> status_code >> optimized_time;
  solve_success = success != 0;
  in >> tag;
  if (tag != "STATES") {
    return false;
  }
  std::size_t count = 0;
  in >> count;
  states.clear();
  states.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    State state;
    in >> state.x >> state.y >> state.yaw;
    states.push_back(state);
  }
  return static_cast<bool>(in) || states.size() == count;
}

std::vector<SafeCorridor> SafeCorridorOptimizer::buildCorridors(
  const std::vector<State> & states) const
{
  std::vector<SafeCorridor> corridors;
  corridors.reserve(states.size());
  for (std::size_t i = 0; i < states.size(); ++i) {
    corridors.push_back(buildCorridor(states, i));
  }
  return corridors;
}

SafeCorridor SafeCorridorOptimizer::buildCorridor(
  const std::vector<State> & states, std::size_t index) const
{
  const State & state = states[index];
  double vx = std::cos(state.yaw);
  double vy = std::sin(state.yaw);
  double motion_vx = vx;
  double motion_vy = vy;

  if (states.size() >= 2) {
    const State & prev = states[index == 0 ? index : index - 1];
    const State & next = states[index + 1 < states.size() ? index + 1 : index];
    const double path_vx = next.x - prev.x;
    const double path_vy = next.y - prev.y;
    const double path_norm = std::hypot(path_vx, path_vy);
    if (path_norm > 1e-6) {
      motion_vx = path_vx / path_norm;
      motion_vy = path_vy / path_norm;
      if (!optimizer_config_.corridor_axis_aligned) {
        vx = motion_vx;
        vy = motion_vy;
      }
    }
  }
  if (optimizer_config_.corridor_axis_aligned) {
    vx = 1.0;
    vy = 0.0;
  }

  const double norm = std::max(1e-9, std::hypot(vx, vy));
  SafeCorridor corridor;
  corridor.x = state.x;
  corridor.y = state.y;
  corridor.tangent_x = vx / norm;
  corridor.tangent_y = vy / norm;
  corridor.normal_x = -corridor.tangent_y;
  corridor.normal_y = corridor.tangent_x;

  if (!centerIsSafe(corridor.x, corridor.y)) {
    return corridor;
  }

  SafeCorridor max_target = corridor;
  const double max_distance = std::max(0.0, optimizer_config_.corridor_max_distance);
  max_target.forward = max_distance;
  max_target.backward = max_distance;
  max_target.left = max_distance;
  max_target.right = max_distance;

  const double fast_step = std::max(optimizer_config_.corridor_fast_step, 1e-3);
  const double fine_step = std::max(optimizer_config_.corridor_fine_step, 1e-3);
  std::array<int, 4> side_order{0, 1, 2, 3};
  if (optimizer_config_.corridor_axis_aligned) {
    const int x_side = motion_vx >= 0.0 ? 0 : 1;
    const int y_side = motion_vy >= 0.0 ? 2 : 3;
    const int opposite_x_side = motion_vx >= 0.0 ? 1 : 0;
    const int opposite_y_side = motion_vy >= 0.0 ? 3 : 2;
    if (std::abs(motion_vx) >= std::abs(motion_vy)) {
      side_order = {x_side, y_side, opposite_x_side, opposite_y_side};
    } else {
      side_order = {y_side, x_side, opposite_y_side, opposite_x_side};
    }
  }
  growCorridor(corridor, fast_step, max_target, side_order);

  SafeCorridor fine_target = corridor;
  fine_target.forward = std::min(max_target.forward, corridor.forward + fast_step);
  fine_target.backward = std::min(max_target.backward, corridor.backward + fast_step);
  fine_target.left = std::min(max_target.left, corridor.left + fast_step);
  fine_target.right = std::min(max_target.right, corridor.right + fast_step);
  growCorridor(corridor, fine_step, fine_target, side_order);
  return corridor;
}

void SafeCorridorOptimizer::growCorridor(
  SafeCorridor & corridor, double step, const SafeCorridor & target,
  const std::array<int, 4> & side_order) const
{
  std::array<bool, 4> active{true, true, true, true};
  bool changed = true;
  while (changed) {
    changed = false;
    for (const int side : side_order) {
      if (!active[side]) {
        continue;
      }
      if (tryExpand(corridor, side, step, target)) {
        changed = true;
      } else {
        active[side] = false;
      }
    }
  }
}

bool SafeCorridorOptimizer::tryExpand(
  SafeCorridor & corridor, int side, double step, const SafeCorridor & target) const
{
  SafeCorridor candidate = corridor;
  auto grow = [&](double current, double limit) {
    if (current >= limit - 1e-9) {
      return current;
    }
    return std::min(limit, current + step);
  };

  if (side == 0) {
    candidate.forward = grow(corridor.forward, target.forward);
  } else if (side == 1) {
    candidate.backward = grow(corridor.backward, target.backward);
  } else if (side == 2) {
    candidate.left = grow(corridor.left, target.left);
  } else {
    candidate.right = grow(corridor.right, target.right);
  }

  if (
    candidate.forward == corridor.forward && candidate.backward == corridor.backward &&
    candidate.left == corridor.left && candidate.right == corridor.right)
  {
    return false;
  }
  if (!expansionStripIsSafe(corridor, candidate, side)) {
    return false;
  }
  corridor = candidate;
  return true;
}

bool SafeCorridorOptimizer::expansionStripIsSafe(
  const SafeCorridor & previous, const SafeCorridor & candidate, int side) const
{
  double s_min = -candidate.backward;
  double s_max = candidate.forward;
  double l_min = -candidate.right;
  double l_max = candidate.left;

  if (side == 0) {
    s_min = previous.forward;
    s_max = candidate.forward;
  } else if (side == 1) {
    s_min = -candidate.backward;
    s_max = -previous.backward;
  } else if (side == 2) {
    l_min = previous.left;
    l_max = candidate.left;
  } else {
    l_min = -candidate.right;
    l_max = -previous.right;
  }

  const double sample_step = std::max(optimizer_config_.corridor_fine_step, grid_.resolution());
  const int s_steps = std::max(1, static_cast<int>(std::ceil((s_max - s_min) / sample_step)));
  const int l_steps = std::max(1, static_cast<int>(std::ceil((l_max - l_min) / sample_step)));

  for (int si = 0; si <= s_steps; ++si) {
    const double s = s_min + (s_max - s_min) * static_cast<double>(si) / static_cast<double>(s_steps);
    for (int li = 0; li <= l_steps; ++li) {
      const double l = l_min + (l_max - l_min) * static_cast<double>(li) / static_cast<double>(l_steps);
      const double x = candidate.x + s * candidate.tangent_x + l * candidate.normal_x;
      const double y = candidate.y + s * candidate.tangent_y + l * candidate.normal_y;
      if (!centerIsSafe(x, y)) {
        return false;
      }
    }
  }
  return true;
}

bool SafeCorridorOptimizer::corridorIsSafe(const SafeCorridor & corridor) const
{
  if (!centerIsSafe(corridor.x, corridor.y)) {
    return false;
  }

  const double sample_step = std::max(optimizer_config_.corridor_fine_step, grid_.resolution());
  const int s_steps = std::max(
    1, static_cast<int>(std::ceil((corridor.forward + corridor.backward) / sample_step)));
  const int l_steps = std::max(
    1, static_cast<int>(std::ceil((corridor.left + corridor.right) / sample_step)));

  for (int si = 0; si <= s_steps; ++si) {
    const double s = -corridor.backward +
      (corridor.forward + corridor.backward) * static_cast<double>(si) / static_cast<double>(s_steps);
    for (int li = 0; li <= l_steps; ++li) {
      const double l = -corridor.right +
        (corridor.left + corridor.right) * static_cast<double>(li) / static_cast<double>(l_steps);
      const double x = corridor.x + s * corridor.tangent_x + l * corridor.normal_x;
      const double y = corridor.y + s * corridor.tangent_y + l * corridor.normal_y;
      if (!centerIsSafe(x, y)) {
        return false;
      }
    }
  }

  const double corners[4][2] = {
    {corridor.forward, corridor.left},
    {corridor.forward, -corridor.right},
    {-corridor.backward, corridor.left},
    {-corridor.backward, -corridor.right},
  };
  for (const auto & corner : corners) {
    const double x = corridor.x + corner[0] * corridor.tangent_x + corner[1] * corridor.normal_x;
    const double y = corridor.y + corner[0] * corridor.tangent_y + corner[1] * corridor.normal_y;
    if (!centerIsSafe(x, y)) {
      return false;
    }
  }
  return true;
}

bool SafeCorridorOptimizer::centerIsSafe(double x, double y) const
{
  return grid_.obstacleDistanceWorld(x, y) >= corridor_circle_radius_;
}

std::vector<State> SafeCorridorOptimizer::circleCenterStates(
  const std::vector<State> & states, double sign) const
{
  std::vector<State> centers;
  centers.reserve(states.size());
  const double offset = sign * planner_config_.robot_length * 0.25;
  for (const auto & state : states) {
    State center = state;
    center.x = state.x + offset * std::cos(state.yaw);
    center.y = state.y + offset * std::sin(state.yaw);
    centers.push_back(center);
  }
  return centers;
}

double SafeCorridorOptimizer::normAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle <= -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double SafeCorridorOptimizer::interpolateAngle(double from, double to, double t)
{
  return normAngle(from + t * normAngle(to - from));
}

}  // namespace fourwis_hybrid_astar_cpp
