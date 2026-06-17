cmake_minimum_required(VERSION 3.8)
project(fourwis_hybrid_astar_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ompl REQUIRED)

set(CMAKE_AUTOMOC ON)

include_directories(include)

add_library(fourwis_planner_core
  src/planner_core.cpp
  src/safe_corridor_optimizer.cpp
)
target_compile_features(fourwis_planner_core PUBLIC cxx_std_17)
target_link_libraries(fourwis_planner_core PUBLIC ${OMPL_LIBRARIES})
target_include_directories(fourwis_planner_core PUBLIC ${OMPL_INCLUDE_DIRS})

add_executable(fourwis_planner_cpp src/planner_node.cpp)
target_link_libraries(fourwis_planner_cpp fourwis_planner_core)
ament_target_dependencies(fourwis_planner_cpp
  geometry_msgs nav_msgs rclcpp std_msgs visualization_msgs
)

add_library(status_panel SHARED
  include/fourwis_hybrid_astar_cpp/status_panel.hpp
  src/status_panel.cpp
)
target_compile_features(status_panel PUBLIC cxx_std_17)
target_link_libraries(status_panel Qt5::Widgets)
ament_target_dependencies(status_panel
  pluginlib rcl_interfaces rclcpp rviz_common std_msgs
)
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

add_executable(fourwis_demo_map_cpp src/demo_map_node.cpp)
ament_target_dependencies(fourwis_demo_map_cpp nav_msgs rclcpp)

install(TARGETS
  fourwis_planner_core
  fourwis_planner_cpp
  status_panel
  fourwis_demo_map_cpp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES config/planner.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch rviz scripts docs
  DESTINATION share/${PROJECT_NAME}
)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
