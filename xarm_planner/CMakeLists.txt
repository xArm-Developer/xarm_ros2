cmake_minimum_required(VERSION 3.5)
project(xarm_planner)

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
    -Wno-unused-variable
  )
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(xarm_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(dependencies "rclcpp" "std_msgs" "geometry_msgs" "xarm_msgs")

include_directories(
  include
  ${ament_INCLUDE_DIRS}
)

add_library(xarm_planner SHARED
  src/xarm_planner.cpp
)
ament_target_dependencies(xarm_planner 
  moveit_ros_planning_interface
  ${dependencies}
)

add_executable(xarm_planner_node src/xarm_planner_node.cpp)
ament_target_dependencies(xarm_planner_node
  ${dependencies}
)
target_link_libraries(xarm_planner_node 
  xarm_planner
  ${ament_LIBRARIES}
)

add_executable(test_xarm_planner_joint test/test_xarm_planner_joint.cpp)
ament_target_dependencies(test_xarm_planner_joint
  ${dependencies}
)
target_link_libraries(test_xarm_planner_joint 
  xarm_planner
  ${ament_LIBRARIES}
)

add_executable(test_xarm_planner_pose test/test_xarm_planner_pose.cpp)
ament_target_dependencies(test_xarm_planner_pose
  ${dependencies}
)
target_link_libraries(test_xarm_planner_pose 
  xarm_planner
  ${ament_LIBRARIES}
)

add_executable(test_xarm_planner_node_joint test/test_xarm_planner_node_joint.cpp)
ament_target_dependencies(test_xarm_planner_node_joint
  ${dependencies}
)

add_executable(test_xarm_planner_node_pose test/test_xarm_planner_node_pose.cpp)
ament_target_dependencies(test_xarm_planner_node_pose
  ${dependencies}
)

# add_executable(xarm_simple_planner src/xarm_simple_planner.cpp)
# ament_target_dependencies(xarm_simple_planner
#   moveit_ros_planning_interface
#   ${dependencies}
# )

# add_executable(xarm_simple_planner_joint_test test/xarm_simple_planner_joint_test.cpp)
# ament_target_dependencies(xarm_simple_planner_joint_test
#   ${dependencies}
# )
# add_executable(xarm_simple_planner_pose_test test/xarm_simple_planner_pose_test.cpp)
# ament_target_dependencies(xarm_simple_planner_pose_test
#   ${dependencies}
# )

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

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS 
  xarm_planner
  xarm_planner_node
  test_xarm_planner_joint
  test_xarm_planner_pose
  test_xarm_planner_node_joint
  test_xarm_planner_node_pose
  # xarm_simple_planner 
  # xarm_simple_planner_joint_test 
  # xarm_simple_planner_pose_test
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
