cmake_minimum_required(VERSION 3.5)
project(realsense_gazebo_plugin)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wno-inconsistent-missing-override")
endif()

set(FastRTPS_INCLUDE_DIR /opt/ros/foxy/include) 
set(FastRTPS_LIBRARY_RELEASE /opt/ros/foxy/lib/libfastrtps.so)
# Load catkin and all dependencies required for this package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(gazebo_ros REQUIRED)
# find_package(image_transport REQUIRED)
# find_package(camera_info_manager REQUIRED)
# find_package(sensor_msgs REQUIRED)

# find_package(gazebo REQUIRED)

# #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# set( dependencies
#     rclcpp
#     gazebo_ros
#     image_transport
#     camera_info_manager
#     sensor_msgs
# )

# include_directories(
#     SYSTEM
#     include
#     ${GAZEBO_INCLUDE_DIRS}
# )

# link_directories(${GAZEBO_LIBRARY_DIRS})

# add_library(
#     ${PROJECT_NAME} SHARED
#     src/RealSensePlugin.cpp
#     src/gazebo_ros_realsense.cpp
#     )

# ament_target_dependencies(${PROJECT_NAME} ${dependencies})
# target_link_libraries(${PROJECT_NAME} ${GAZEBO_LIBRARIES})

# install(TARGETS ${PROJECT_NAME} 
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# install(DIRECTORY include/
#   DESTINATION include/
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
