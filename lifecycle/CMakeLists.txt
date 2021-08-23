cmake_minimum_required(VERSION 3.5)
project(lifecycle_examples)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

set(executable_name lifecycle_return_code_test)

set(dependencies
  rclcpp
  rclcpp_lifecycle
)

# Executable
add_executable(${executable_name}
  main.cpp
)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
