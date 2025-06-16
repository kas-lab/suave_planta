cmake_minimum_required(VERSION 3.8)
project(suave_planta)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(system_modes_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

set(dependencies
  ament_cmake
  diagnostic_msgs
  mavros_msgs
  std_msgs
  std_srvs
  rclcpp
  plansys2_executor
  plansys2_domain_expert
  plansys2_problem_expert
  plansys2_planner
  system_modes_msgs
  lifecycle_msgs
)

include_directories(
  include
)

add_executable(action_search_pipeline src/action_search_pipeline.cpp)
ament_target_dependencies(action_search_pipeline ${dependencies})

add_executable(action_inspect_pipeline src/action_inspect_pipeline.cpp)
ament_target_dependencies(action_inspect_pipeline ${dependencies})

add_executable(action_start_robot src/action_start_robot.cpp)
ament_target_dependencies(action_start_robot ${dependencies})

add_executable(action_recharge src/action_recharge.cpp)
ament_target_dependencies(action_recharge ${dependencies})

add_executable(action_reconfigure src/action_reconfigure.cpp)
ament_target_dependencies(action_reconfigure ${dependencies})

add_executable(suave_planta_controller src/suave_planta_controller.cpp)
ament_target_dependencies(suave_planta_controller ${dependencies})


install(TARGETS
  action_search_pipeline
  action_inspect_pipeline
  action_start_robot
  action_recharge
  action_reconfigure
  suave_planta_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  pddl
  config
  owl
  DESTINATION share/${PROJECT_NAME}
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

ament_export_include_directories(include)
ament_package()
