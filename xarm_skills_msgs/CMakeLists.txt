cmake_minimum_required(VERSION 3.8)
project(xarm_skills_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Stir.action"
  "action/PickAndPlace.action"
  "action/Dispense.action"
  "action/Glue.action"
  "action/Transfer.action"
  "action/WaitForUser.action"
  DEPENDENCIES geometry_msgs
)

ament_package()
