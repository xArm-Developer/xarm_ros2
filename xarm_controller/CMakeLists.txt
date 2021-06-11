cmake_minimum_required(VERSION 3.5)
project(xarm_controller)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic
    -Wno-sign-compare
    -Wno-unused-parameter 
  )
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(xarm_msgs REQUIRED)
find_package(xarm_api REQUIRED)

add_library(
  xarm_hw
  SHARED
  src/xarm_hw.cpp
)
target_include_directories(
  xarm_hw
  PRIVATE
  include
  ${xarm_api_INCLUDE_DIRS}
)
ament_target_dependencies(
  xarm_hw
  hardware_interface
  pluginlib
  rclcpp
  std_msgs
  sensor_msgs
  xarm_msgs
)

target_link_libraries(xarm_hw 
  ${xarm_api_LIBRARIES}
  ${ament_LIBRARIES}
)

add_library(
  fake_xarm_hw
  SHARED
  src/fake_xarm_hw.cpp
)
target_include_directories(
  fake_xarm_hw
  PRIVATE
  include
)
ament_target_dependencies(
  fake_xarm_hw
  hardware_interface
  pluginlib
  rclcpp
  std_msgs
  sensor_msgs
)
target_link_libraries(fake_xarm_hw 
  ${ament_LIBRARIES}
)

pluginlib_export_plugin_description_file(hardware_interface xarm_hw_plugin.xml)
pluginlib_export_plugin_description_file(hardware_interface fake_xarm_hw_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS xarm_hw fake_xarm_hw
  DESTINATION lib
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_libraries(xarm_hw)
ament_export_libraries(fake_xarm_hw)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
)

ament_package()
