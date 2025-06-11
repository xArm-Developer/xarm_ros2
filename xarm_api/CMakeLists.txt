cmake_minimum_required(VERSION 3.5)
project(xarm_api)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic
    -Wno-sign-compare
    -Wno-unused-parameter 
    -Wno-unused-variable
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

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(xarm_msgs REQUIRED)
find_package(xarm_sdk REQUIRED)

include_directories(
  include
  ${xarm_sdk_INCLUDE_DIRS}
  ${ament_INCLUDE_DIRS}
)

set(dependencies "rclcpp" "rclcpp_action" "std_msgs" "sensor_msgs" "control_msgs" "geometry_msgs" "xarm_msgs")

add_library(xarm_ros_driver SHARED
  src/xarm_driver.cpp
  src/xarm_driver_service.cpp
)
ament_target_dependencies(xarm_ros_driver ${dependencies})
target_link_libraries(xarm_ros_driver 
  ${xarm_sdk_LIBRARIES}
  ${ament_LIBRARIES}
)

add_executable(xarm_driver_node
  src/xarm_driver_node.cpp
)

ament_target_dependencies(xarm_driver_node ${dependencies})
target_link_libraries(xarm_driver_node 
  xarm_ros_driver
  ${ament_LIBRARIES}
)

add_library(xarm_ros_client SHARED
  src/xarm_ros_client.cpp
)
ament_target_dependencies(xarm_ros_client ${dependencies})

add_executable(test_xarm_ros_client
   test/test_xarm_ros_client.cpp
)
ament_target_dependencies(test_xarm_ros_client ${dependencies})
target_link_libraries(test_xarm_ros_client 
  xarm_ros_client
  ${ament_LIBRARIES}
)

add_executable(test_xarm_velo_move
   test/test_xarm_velo_move.cpp
)
ament_target_dependencies(test_xarm_velo_move ${dependencies})
target_link_libraries(test_xarm_velo_move 
  xarm_ros_client
  ${ament_LIBRARIES}
)

add_executable(test_robot_states
   test/test_robot_states.cpp
)
ament_target_dependencies(test_robot_states ${dependencies})

ament_export_libraries(xarm_ros_driver)
ament_export_libraries(xarm_ros_client)
ament_export_include_directories(include)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS xarm_ros_driver xarm_ros_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(
  TARGETS
  xarm_driver_node 
  test_xarm_ros_client
  test_xarm_velo_move
  test_robot_states
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
