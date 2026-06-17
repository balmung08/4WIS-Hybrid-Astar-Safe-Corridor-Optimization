#pragma once

#include <array>
#include <string>
#include <vector>

#include "fourwis_hybrid_astar_cpp/planner_core.hpp"

namespace fourwis_hybrid_astar_cpp
{

struct SafeCorridorConfig
{
  std::string casadi_python{"/home/balmung/miniconda3/bin/python3"};
  std::string casadi_script{"/home/balmung/Legacy/4WIS_Global/fourwis_hybrid_astar_cpp/scripts/casadi_backend.py"};
  double resample_distance{0.2};
  double corridor_max_distance{1.2};
  double corridor_fast_step{0.25};
  double corridor_fine_step{0.05};
  bool corridor_axis_aligned{true};
  int max_iterations{1};
  int ipopt_max_iterations{80};
  double comfort_weight{0.1};
  double constraint_penalty{1.0e6};
  double infeasibility_tolerance{1.0e-6};
  double max_velocity{1.0};
  double max_acceleration{2.0};
  double ipopt_tol{1.0e-4};
};

struct OptimizerStats
{
  double total_ms{0.0};
  int iterations{0};
  int ipopt_iterations{0};
  int ipopt_status{0};
  bool solve_success{false};
  double optimized_time{0.0};
  double max_position_delta{0.0};
  double mean_position_delta{0.0};
  double max_corridor_length{0.0};
  double max_corridor_width{0.0};
};

struct SafeCorridor
{
  double x{0.0};
  double y{0.0};
  double tangent_x{1.0};
  double tangent_y{0.0};
  double normal_x{0.0};
  double normal_y{1.0};
  double backward{0.0};
  double forward{0.0};
  double right{0.0};
  double left{0.0};
};

struct OptimizerResult
{
  std::vector<State> states;
  std::vector<SafeCorridor> corridors;
  OptimizerStats stats;
};

class SafeCorridorOptimizer
{
public:
  SafeCorridorOptimizer(
    const OccupancyGrid & grid, const PlannerConfig & planner_config,
    SafeCorridorConfig optimizer_config);

  OptimizerResult optimize(const std::vector<State> & input) const;

private:
  std::vector<State> resample(const std::vector<State> & input) const;
  std::vector<SafeCorridor> buildCorridors(const std::vector<State> & states) const;
  SafeCorridor buildCorridor(const std::vector<State> & states, std::size_t index) const;
  void growCorridor(
    SafeCorridor & corridor, double step, const SafeCorridor & target,
    const std::array<int, 4> & side_order) const;
  bool tryExpand(SafeCorridor & corridor, int side, double step, const SafeCorridor & target) const;
  bool expansionStripIsSafe(
    const SafeCorridor & previous, const SafeCorridor & candidate, int side) const;
  bool corridorIsSafe(const SafeCorridor & corridor) const;
  bool centerIsSafe(double x, double y) const;
  std::vector<State> circleCenterStates(const std::vector<State> & states, double sign) const;
  std::vector<State> solveCasadiBackend(
    const std::vector<State> & reference, const std::vector<SafeCorridor> & corridors,
    const std::vector<SafeCorridor> & front_corridors,
    const std::vector<SafeCorridor> & rear_corridors,
    double & optimized_time, bool & solve_success, int & status_code) const;
  bool writeCasadiInput(
    const std::string & path, const std::vector<State> & reference,
    const std::vector<SafeCorridor> & corridors,
    const std::vector<SafeCorridor> & front_corridors,
    const std::vector<SafeCorridor> & rear_corridors) const;
  bool readCasadiOutput(
    const std::string & path, std::vector<State> & states, double & optimized_time,
    bool & solve_success, int & status_code) const;
  static double normAngle(double angle);
  static double interpolateAngle(double from, double to, double t);

  const OccupancyGrid & grid_;
  PlannerConfig planner_config_;
  SafeCorridorConfig optimizer_config_;
  double corridor_circle_radius_{0.0};
};

}  // namespace fourwis_hybrid_astar_cpp
