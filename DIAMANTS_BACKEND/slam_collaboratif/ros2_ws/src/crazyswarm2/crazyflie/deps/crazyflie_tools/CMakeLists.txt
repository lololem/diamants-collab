cmake_minimum_required(VERSION 3.5)
project(crazyflie_tools)

# Enable C++14 and warnings
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

if (WIN32)
  # Boost library handling
  set(Boost_USE_STATIC_LIBS OFF)
  set(Boost_USE_MULTITHREADED ON)
  set(Boost_USE_STATIC_RUNTIME OFF)
  unset(Boost_INCLUDE_DIR CACHE)
  unset(Boost_LIBRARY_DIRS CACHE)
  add_definitions(-DBOOST_ALL_NO_LIB)
  add_definitions(-DBOOST_ALL_DYN_LINK)
endif()

find_package(Boost REQUIRED COMPONENTS program_options)

# Apple macOS libraries
if (APPLE)
  set(CMAKE_EXE_LINKER_FLAGS "-lobjc -framework IOKit -framework CoreFoundation -framework Security")
  set(CMAKE_MODULE_LINKER_FLAGS "-lobjc -framework IOKit -framework CoreFoundation -framework Security")
endif()

add_subdirectory(crazyflie_cpp EXCLUDE_FROM_ALL)

include_directories(
  crazyflie_cpp/include
  ${Boost_INCLUDE_DIRS}
)

link_directories(
  ${Boost_LIBRARY_DIRS}
)

# scan
add_executable(scan
  src/scan.cpp
)
target_link_libraries(scan
  crazyflie_cpp
  ${Boost_LIBRARIES}
)

# listParams
add_executable(listParams
  src/listParams.cpp
)
target_link_libraries(listParams
  crazyflie_cpp
  ${Boost_LIBRARIES}
)

# listLogVariables
add_executable(listLogVariables
  src/listLogVariables.cpp
)
target_link_libraries(listLogVariables
  crazyflie_cpp
  ${Boost_LIBRARIES}
)

# listMemories
add_executable(listMemories
  src/listMemories.cpp
)
target_link_libraries(listMemories
  crazyflie_cpp
  ${Boost_LIBRARIES}
)

# reboot
add_executable(reboot
  src/reboot.cpp
)
target_link_libraries(reboot
  crazyflie_cpp
  ${Boost_LIBRARIES}
)

# battery
add_executable(battery
  src/battery.cpp
)
target_link_libraries(battery
  crazyflie_cpp
  ${Boost_LIBRARIES}
)

# version
add_executable(version
  src/version.cpp
)
target_link_libraries(version
  crazyflie_cpp
  ${Boost_LIBRARIES}
)

# console
add_executable(console
  src/console.cpp
)
target_link_libraries(console
  crazyflie_cpp
  ${Boost_LIBRARIES}
)

# # flash
# add_executable(flash
#   src/flash.cpp
# )
# target_link_libraries(flash
#   crazyflie_cpp
#   ${Boost_LIBRARIES}
# )

# comCheck
add_executable(comCheck
  src/comCheck.cpp
)
target_link_libraries(comCheck
  crazyflie_cpp
  ${Boost_LIBRARIES}
)

# log
add_executable(log
  src/log.cpp
)
target_link_libraries(log
  crazyflie_cpp
  ${Boost_LIBRARIES}
)

# downloadUSDLogfile
add_executable(downloadUSDLogfile
  src/downloadUSDLogfile.cpp
)
target_link_libraries(downloadUSDLogfile
  crazyflie_cpp
  ${Boost_LIBRARIES}
)

# setParam
add_executable(setParam
  src/setParam.cpp
)
target_link_libraries(setParam
  crazyflie_cpp
  ${Boost_LIBRARIES}
)