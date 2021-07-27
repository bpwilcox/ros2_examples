cmake_minimum_required(VERSION 3.5)
project(plugins)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(pluginlib REQUIRED)

# include_directories(
#   include
# )

set(executable_name plugin-lifecycle-example)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  std_msgs
  example_interfaces
  pluginlib
)

# plugins
add_library(plugin-examples SHARED
  plugin-example-a.cpp
  plugin-example-b.cpp
)

ament_target_dependencies(plugin-examples
  ${dependencies}
)

pluginlib_export_plugin_description_file(plugins plugins.xml)

# Executable
add_executable(${executable_name}
  main.cpp
)

ament_target_dependencies(${executable_name}
  ${dependencies}
)


install(TARGETS plugin-examples
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# install(DIRECTORY include/
#   DESTINATION include/
# )

install(FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
#   find_package(ament_cmake_gtest REQUIRED)
#   add_subdirectory(test)
# endif()

# ament_export_include_directories(include)
# ament_export_libraries(${library_name}
#   nav2_backup_recovery
#   nav2_spin_recovery
#   nav2_wait_recovery
# )

ament_package()
