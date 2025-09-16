cmake_minimum_required(VERSION 3.5)

project(crazyflie_ros2_multiranger_bringup)

find_package(ament_cmake REQUIRED)
find_package(ros_gz_crazyflie_bringup REQUIRED)

# Install project launch and config files
install(DIRECTORY
  launch
  config

  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()