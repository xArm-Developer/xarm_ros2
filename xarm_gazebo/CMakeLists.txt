cmake_minimum_required(VERSION 3.5)
project(xarm_gazebo)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(controller_manager REQUIRED)
find_package(gz_ros2_control REQUIRED)
find_package(ros_gz_sim REQUIRED)

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

# include_directories(
#   include
#   ${ament_INCLUDE_DIRS}
#   ${GAZEBO_INCLUDE_DIRS}
# )

# link_directories(${GAZEBO_LIBRARY_DIRS})
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# add_library(gazebo_mimic_joint_plugin SHARED src/mimic_joint_plugin.cpp)
# ament_target_dependencies(gazebo_mimic_joint_plugin
#   control_toolbox
#   controller_manager
#   gazebo_ros
# )
# target_link_libraries(gazebo_mimic_joint_plugin ${ament_LIBRARIES} ${GAZEBO_LIBRARIES})

# install(
#   DIRECTORY include/
#   DESTINATION include
# )

# install(TARGETS
# gazebo_mimic_joint_plugin
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )


install(DIRECTORY
  worlds
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
