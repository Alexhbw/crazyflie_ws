cmake_minimum_required(VERSION 3.8)
project(crazyflie_ode_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
add_executable(ode_engine_node
  src/ode_engine_node.cpp
  src/safety_monitor.cpp
  src/interval_reachability.cpp
)
target_include_directories(ode_engine_node PRIVATE
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(ode_engine_node yaml-cpp)
ament_target_dependencies(ode_engine_node rclcpp std_msgs sensor_msgs geometry_msgs rosgraph_msgs nav_msgs tf2_ros visualization_msgs ament_index_cpp)

install(TARGETS
  ode_engine_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

install(DIRECTORY urdf launch rviz meshes config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/counterexample_mining.py
  scripts/run_experiments.py
  scripts/safety_boundary_analysis.py
  scripts/plot_results.py
  DESTINATION lib/${PROJECT_NAME}
)
