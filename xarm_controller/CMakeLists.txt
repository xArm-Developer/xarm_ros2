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
# find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(xarm_msgs REQUIRED)
find_package(xarm_api REQUIRED)
find_package(xarm_sdk REQUIRED)

add_library(
  uf_robot_hardware
  SHARED
  src/hardware/uf_robot_system_hardware.cpp
  src/hardware/uf_robot_fake_system_hardware.cpp
)
target_include_directories(
  uf_robot_hardware
  PRIVATE
  include
  ${xarm_sdk_INCLUDE_DIRS}
  ${xarm_api_INCLUDE_DIRS}
  ${ament_INCLUDE_DIRS}
)
ament_target_dependencies(
  uf_robot_hardware
  hardware_interface
  # controller_interface
  pluginlib
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  sensor_msgs
  controller_manager_msgs
  xarm_msgs
)

# message(STATUS "xarm_sdk_INCLUDE_DIRS: ${xarm_sdk_INCLUDE_DIRS}")
# message(WARNING "xarm_api_INCLUDE_DIRS: ${xarm_api_INCLUDE_DIRS}")
# message(WARNING "xarm_sdk_LIBRARIES: ${xarm_sdk_LIBRARIES}")
# message(WARNING "xarm_api_LIBRARIES: ${xarm_api_LIBRARIES}")

target_link_libraries(uf_robot_hardware 
  ${xarm_api_LIBRARIES}
  ${ament_LIBRARIES}
)

pluginlib_export_plugin_description_file(hardware_interface uf_hardware_interface_plugins.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS uf_robot_hardware
  DESTINATION lib
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_libraries(uf_robot_hardware)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
)

ament_package()
