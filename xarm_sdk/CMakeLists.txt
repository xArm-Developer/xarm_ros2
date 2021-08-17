cmake_minimum_required(VERSION 3.5)
project(xarm_sdk)

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
    -Wno-unused-but-set-variable
    -Wno-type-limits
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

set(CXX_SDK_DIR cxx)
set(CXX_SDK_INC ${CXX_SDK_DIR}/include)
set(CXX_SDK_SRC ${CXX_SDK_DIR}/src)

set(CXX_SDK_SRC_XARM ${CXX_SDK_SRC}/xarm)
set(CXX_SDK_SRC_XARM_WRAPPER ${CXX_SDK_SRC_XARM}/wrapper)
set(CXX_SDK_SRC_XARM_CORE ${CXX_SDK_SRC_XARM}/core)
set(CXX_SDK_SRC_SERIAL ${CXX_SDK_SRC}/serial)

if(NOT EXISTS ${PROJECT_SOURCE_DIR}/${CXX_SDK_INC}
  OR NOT EXISTS ${PROJECT_SOURCE_DIR}/${CXX_SDK_INC}/xarm/wrapper/xarm_api.h
  OR NOT EXISTS ${PROJECT_SOURCE_DIR}/${CXX_SDK_SRC_XARM_WRAPPER}
  OR NOT EXISTS ${PROJECT_SOURCE_DIR}/${CXX_SDK_SRC_XARM_CORE}
  OR NOT EXISTS ${PROJECT_SOURCE_DIR}/${CXX_SDK_SRC_SERIAL}
  OR NOT EXISTS ${PROJECT_SOURCE_DIR}/${CXX_SDK_SRC_XARM_WRAPPER}/xarm_api.cc
)
  message(FATAL_ERROR "xarm_sdk contains sub-modules, Please use the following command to get or update the sub-module code first
  $ git pull
  $ git submodule sync
  $ git submodule update --init --remote")
endif()

include_directories(
  ${CXX_SDK_INC}
  ${ament_INCLUDE_DIRS}
)

file(GLOB_RECURSE CXX_SDK_SRC_FILES ${CXX_SDK_SRC}/*.cc)
# list(APPEND CXX_SDK_SRC_FILE_LIST ${CXX_SDK_SRC_FILES})
set(CXX_SDK_SRC_FILE_LIST "")
foreach(f ${CXX_SDK_SRC_FILES})
  if (NOT f MATCHES "bak")
    list(APPEND CXX_SDK_SRC_FILE_LIST ${f}) 
  endif()
endforeach()

add_library(xarm_cxx_sdk SHARED
  ${CXX_SDK_SRC_FILE_LIST}
  # ${CXX_SDK_SRC_XARM_WRAPPER}/xarm_api.cc
  # ${CXX_SDK_SRC_XARM_WRAPPER}/xarm_bio.cc
  # ${CXX_SDK_SRC_XARM_WRAPPER}/xarm_events.cc
  # ${CXX_SDK_SRC_XARM_WRAPPER}/xarm_gpio.cc
  # ${CXX_SDK_SRC_XARM_WRAPPER}/xarm_gripper.cc
  # ${CXX_SDK_SRC_XARM_WRAPPER}/xarm_motion.cc
  # ${CXX_SDK_SRC_XARM_WRAPPER}/xarm_params.cc
  # ${CXX_SDK_SRC_XARM_WRAPPER}/xarm_record.cc
  # ${CXX_SDK_SRC_XARM_WRAPPER}/xarm_report.cc
  # ${CXX_SDK_SRC_XARM_WRAPPER}/xarm_robotiq.cc
  # ${CXX_SDK_SRC_XARM_WRAPPER}/xarm_servo.cc

  # ${CXX_SDK_SRC_XARM_CORE}/connect.cc
  # ${CXX_SDK_SRC_XARM_CORE}/report_data.cc

  # ${CXX_SDK_SRC_XARM_CORE}/common/crc16.cc
  # ${CXX_SDK_SRC_XARM_CORE}/common/queue_memcpy.cc

  # ${CXX_SDK_SRC_XARM_CORE}/debug/debug_print.cc

  # ${CXX_SDK_SRC_XARM_CORE}/instruction/uxbus_cmd.cc
  # ${CXX_SDK_SRC_XARM_CORE}/instruction/uxbus_cmd_ser.cc
  # ${CXX_SDK_SRC_XARM_CORE}/instruction/uxbus_cmd_tcp.cc

  #  ${CXX_SDK_SRC_XARM_CORE}/os/network.cc

  # ${CXX_SDK_SRC_XARM_CORE}/port/ser.cc
  # ${CXX_SDK_SRC_XARM_CORE}/port/socket.cc

  # ${CXX_SDK_SRC_SERIAL}/serial.cc
  # ${CXX_SDK_SRC_SERIAL}/impl/unix.cc
  # ${CXX_SDK_SRC_SERIAL}/impl/win.cc
  # ${CXX_SDK_SRC_SERIAL}/impl/list_ports/list_ports_linux.cc
  # ${CXX_SDK_SRC_SERIAL}/impl/list_ports/list_ports_osx.cc
  # ${CXX_SDK_SRC_SERIAL}/impl/list_ports/list_ports_win.cc
)

if(MSVC)
  target_link_libraries(xarm_cxx_sdk 
    Ws2_32.lib
    ${catkin_LIBRARIES}
  )
endif()

ament_export_libraries(xarm_cxx_sdk)
ament_export_include_directories(${CXX_SDK_INC})

install(DIRECTORY ${CXX_SDK_INC}
        DESTINATION ${CXX_SDK_DIR})

install(
  TARGETS xarm_cxx_sdk
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()