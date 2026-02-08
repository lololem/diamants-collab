cmake_minimum_required(VERSION 3.8)
project(multi_agent_framework)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Install Python executable scripts
# These are referenced by launch files and must be available as ROS2 executables
install(PROGRAMS
  scripts/drone_position_coordinator.py
  multi_agent_framework/web_interface/websocket_bridge.py
  multi_agent_framework/web_interface/web_server.py
  multi_agent_framework/swarm_intelligence/intelligence_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
install(DIRECTORY
  multi_agent_framework/
  DESTINATION lib/python3/dist-packages/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.py"
)

# Install web interface static files and templates
install(DIRECTORY
  multi_agent_framework/web_interface/static/
  DESTINATION share/${PROJECT_NAME}/web_interface/static/
)

install(DIRECTORY
  multi_agent_framework/web_interface/templates/
  DESTINATION share/${PROJECT_NAME}/web_interface/templates/
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
