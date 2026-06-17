#include <algorithm>
#include <cmath>
#include <string>
#include <tuple>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
struct Scenario
{
  double width;
  double height;
  std::vector<std::tuple<double, double, double, double>> obstacles;
  double origin_x{0.0};
  double origin_y{0.0};
};

Scenario legacyMazeScenario()
{
  return {
    20.0,
    20.0,
    {
      {0, 1, 0, 20}, {0, 20, 0, 1}, {0, 20, 19, 20}, {19, 20, 0, 20},
      {3, 4, 3, 17}, {6, 17, 3, 4}, {16, 17, 4, 17}, {4, 16, 16, 17},
      {6, 8, 6, 14}, {10, 14, 6, 8}, {10, 12, 10, 12}, {12, 14, 12, 14},
    }};
}

Scenario inflateScenario(Scenario scenario, double inflation)
{
  for (auto & obstacle : scenario.obstacles) {
    std::get<0>(obstacle) = std::max(0.0, std::get<0>(obstacle) - inflation);
    std::get<1>(obstacle) = std::min(scenario.width, std::get<1>(obstacle) + inflation);
    std::get<2>(obstacle) = std::max(0.0, std::get<2>(obstacle) - inflation);
    std::get<3>(obstacle) = std::min(scenario.height, std::get<3>(obstacle) + inflation);
  }
  return scenario;
}

Scenario scenarioByName(const std::string & name)
{
  if (name == "reference_parking") {
    return {
      13.0,
      5.4,
      {
        {0.0, 13.0, -0.2, 0.0}, {0.0, 13.0, 5.0, 5.2},
        {4.0, 4.55, 0.0, 1.8}, {6.0, 6.4, 2.2, 3.8},
        {2.0, 2.5, 3.0, 3.4}, {4.0, 4.5, 3.6, 4.0},
        {7.8, 8.2, 1.0, 1.4}, {8.8, 9.2, 3.8, 4.2},
        {8.8, 9.2, 2.8, 3.2}, {9.8, 10.2, 0.8, 1.2},
      },
      0.0,
      -0.2};
  }
  if (name == "tight_complex") {
    return {
      18.0,
      14.0,
      {
        {0.0, 0.4, 0.0, 14.0}, {0.0, 18.0, 0.0, 0.4}, {17.6, 18.0, 0.0, 14.0},
        {0.0, 18.0, 13.6, 14.0},

        {2.8, 3.3, 0.4, 5.5}, {2.8, 3.3, 8.0, 13.6},
        {5.8, 6.3, 3.0, 13.6}, {8.8, 9.3, 0.4, 7.7},
        {11.8, 12.3, 5.8, 13.6}, {14.8, 15.3, 0.4, 9.0},

        {3.3, 5.0, 8.0, 8.5}, {6.3, 8.1, 2.5, 3.0}, {9.3, 11.1, 7.7, 8.2},
        {12.3, 14.0, 5.8, 6.3}, {15.3, 17.6, 8.5, 9.0},

        {4.6, 7.7, 5.2, 5.7}, {4.6, 7.7, 7.2, 7.7},
        {7.2, 7.7, 5.7, 6.4}, {4.6, 5.1, 6.5, 7.2},

        {9.8, 13.0, 9.6, 10.1}, {9.8, 13.0, 11.6, 12.1},
        {9.8, 10.3, 10.1, 10.8}, {12.5, 13.0, 10.9, 11.6},

        {1.3, 2.1, 2.2, 3.4}, {1.3, 2.1, 10.4, 11.6},
        {16.0, 16.8, 2.2, 3.4}, {16.0, 16.8, 10.4, 11.6},

        {6.9, 7.8, 10.4, 11.2}, {10.4, 11.3, 2.0, 2.8},
        {13.1, 13.9, 3.2, 4.0},
      }};
  }
  if (name == "legacy_parking") {
    return {
      15.0,
      10.0,
      {
        {0.0, 0.5, 0.0, 10.0}, {0.5, 15.0, 0.0, 0.5}, {14.5, 15.0, 0.5, 10.0},
        {0.5, 14.5, 9.5, 10.0}, {2.0, 2.5, 0.5, 2.5}, {4.0, 4.5, 0.5, 2.5},
        {6.0, 6.5, 0.5, 2.5}, {8.0, 8.5, 0.5, 2.5}, {10.0, 10.5, 0.5, 2.5},
        {12.0, 12.5, 0.5, 2.5}, {14.0, 14.5, 0.5, 2.5}, {0.5, 11.5, 5.0, 5.5},
        {2.5, 3.0, 5.5, 7.0}, {5.0, 5.5, 5.5, 7.0}, {7.5, 8.0, 5.5, 7.0},
        {10.0, 11.5, 5.5, 7.0},
      }};
  }
  if (name == "legacy_maze_inflated_0_1") {
    return inflateScenario(legacyMazeScenario(), 0.1);
  }
  return legacyMazeScenario();
}
}  // namespace

class DemoMapNode : public rclcpp::Node
{
public:
  DemoMapNode() : Node("fourwis_demo_map_cpp")
  {
    declare_parameter("map_scenario", "legacy_maze");
    declare_parameter("resolution", 0.1);
    declare_parameter("frame_id", "map");

    pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "map", rclcpp::QoS(1).transient_local().reliable());
    parameter_callback_handle_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        return onParameters(parameters);
      });
    map_ = buildMap(
      get_parameter("map_scenario").as_string(), get_parameter("resolution").as_double(),
      get_parameter("frame_id").as_string());
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { publishMap(); });
    publishMap();
  }

private:
  nav_msgs::msg::OccupancyGrid buildMap()
  {
    return buildMap(
      get_parameter("map_scenario").as_string(), get_parameter("resolution").as_double(),
      get_parameter("frame_id").as_string());
  }

  nav_msgs::msg::OccupancyGrid buildMap(
    const std::string & name, double resolution, const std::string & frame_id)
  {
    const Scenario scenario = scenarioByName(name);
    const int width = static_cast<int>(std::round(scenario.width / resolution));
    const int height = static_cast<int>(std::round(scenario.height / resolution));

    nav_msgs::msg::OccupancyGrid msg;
    msg.header.frame_id = frame_id;
    msg.info.resolution = resolution;
    msg.info.width = width;
    msg.info.height = height;
    msg.info.origin.position.x = scenario.origin_x;
    msg.info.origin.position.y = scenario.origin_y;
    msg.info.origin.orientation.w = 1.0;
    msg.data.assign(static_cast<std::size_t>(width * height), 0);

    auto fill = [&](double x_min, double x_max, double y_min, double y_max) {
      const int gx_min = std::max(
        0, static_cast<int>(std::floor((x_min - scenario.origin_x) / resolution)));
      const int gx_max = std::min(
        width - 1, static_cast<int>(std::floor((x_max - scenario.origin_x) / resolution)));
      const int gy_min = std::max(
        0, static_cast<int>(std::floor((y_min - scenario.origin_y) / resolution)));
      const int gy_max = std::min(
        height - 1, static_cast<int>(std::floor((y_max - scenario.origin_y) / resolution)));
      for (int gy = gy_min; gy <= gy_max; ++gy) {
        for (int gx = gx_min; gx <= gx_max; ++gx) {
          msg.data[gy * width + gx] = 100;
        }
      }
    };

    for (const auto & obstacle : scenario.obstacles) {
      fill(std::get<0>(obstacle), std::get<1>(obstacle), std::get<2>(obstacle), std::get<3>(obstacle));
    }
    RCLCPP_DEBUG(
      get_logger(), "loaded C++ demo map '%s', %.1fx%.1f m, obstacles=%zu",
      name.c_str(), scenario.width, scenario.height, scenario.obstacles.size());
    return msg;
  }

  void publishMap()
  {
    map_.header.stamp = now();
    pub_->publish(map_);
  }

  rcl_interfaces::msg::SetParametersResult onParameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    bool rebuild = false;
    std::string scenario = get_parameter("map_scenario").as_string();
    double resolution = get_parameter("resolution").as_double();
    std::string frame_id = get_parameter("frame_id").as_string();
    for (const auto & parameter : parameters) {
      if (parameter.get_name() == "map_scenario") {
        scenario = parameter.as_string();
        rebuild = true;
      } else if (parameter.get_name() == "resolution") {
        resolution = parameter.as_double();
        rebuild = true;
      } else if (parameter.get_name() == "frame_id") {
        frame_id = parameter.as_string();
        rebuild = true;
      }
    }
    if (rebuild) {
      map_ = buildMap(scenario, resolution, frame_id);
      publishMap();
    }
    return result;
  }

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  nav_msgs::msg::OccupancyGrid map_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DemoMapNode>());
  rclcpp::shutdown();
  return 0;
}
