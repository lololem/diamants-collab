cmake_minimum_required(VERSION 3.8)
project(diamants_missions)

find_package(ament_cmake REQUIRED)

# Install launch files and mission YAMLs
install(DIRECTORY launch missions
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
