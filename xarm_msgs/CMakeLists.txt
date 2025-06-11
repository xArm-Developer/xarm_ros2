cmake_minimum_required(VERSION 3.5)
project(xarm_msgs)

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

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/RobotMsg.msg"
  "msg/IOState.msg"
  "msg/CIOState.msg"
  "msg/MoveVelocity.msg"
)

set(srv_dir "srv")
set(srv_files
  ${srv_dir}/BioGripperCtrl.srv
  ${srv_dir}/BioGripperEnable.srv
  ${srv_dir}/Call.srv
  ${srv_dir}/GetAnalogIO.srv
  ${srv_dir}/GetDigitalIO.srv
  ${srv_dir}/GetFloat32.srv
  ${srv_dir}/GetFloat32List.srv
  ${srv_dir}/GetInt16.srv
  ${srv_dir}/GetInt16List.srv
  ${srv_dir}/GetInt32.srv
  ${srv_dir}/GetInt32ByType.srv
  ${srv_dir}/GetSetModbusData.srv
  ${srv_dir}/GripperMove.srv
  ${srv_dir}/MoveCartesian.srv
  ${srv_dir}/MoveCircle.srv
  ${srv_dir}/MoveHome.srv
  ${srv_dir}/MoveJoint.srv
  ${srv_dir}/MoveVelocity.srv
  ${srv_dir}/RobotiqActivate.srv
  ${srv_dir}/RobotiqGetStatus.srv
  ${srv_dir}/RobotiqMove.srv
  ${srv_dir}/RobotiqReset.srv
  ${srv_dir}/SetAnalogIO.srv
  ${srv_dir}/SetDigitalIO.srv
  ${srv_dir}/SetFloat32.srv
  ${srv_dir}/SetFloat32List.srv
  ${srv_dir}/SetInt16.srv
  ${srv_dir}/SetInt16ById.srv
  ${srv_dir}/SetInt16List.srv
  ${srv_dir}/SetInt32.srv
  ${srv_dir}/SetInt32ByType.srv
  ${srv_dir}/TrajCtrl.srv
  ${srv_dir}/TrajPlay.srv
  ${srv_dir}/VacuumGripperCtrl.srv
  ${srv_dir}/SetTcpLoad.srv
  ${srv_dir}/SetModbusTimeout.srv

  ${srv_dir}/IdenLoad.srv
  ${srv_dir}/FtCaliLoad.srv
  ${srv_dir}/FtForceConfig.srv
  ${srv_dir}/FtForcePid.srv
  ${srv_dir}/FtImpedance.srv
  ${srv_dir}/LinearTrackBackOrigin.srv
  ${srv_dir}/LinearTrackSetPos.srv

  ${srv_dir}/PlanExec.srv
  ${srv_dir}/PlanJoint.srv
  ${srv_dir}/PlanPose.srv
  ${srv_dir}/PlanSingleStraight.srv
)

# set(active_files
#   "action/GripperMove.action"
# )

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  # ${active_files}
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()
