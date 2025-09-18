cmake_minimum_required(VERSION 3.5)
project(ur5e_moveit)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Disable Wredundant-decls warnings since rosidl generates redundant function
#  declarations
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS 
    "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wno-redundant-decls")
endif()

# find_package(rospy REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# find_package(ros_tcp_endpoint REQUIRED)
# find_package(message_generation REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(controller_manager REQUIRED)
# find_package(roslaunch REQUIRED)
find_package(trajectory_msgs REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

set(msg_files
  "msg/UR5eMoveitJoints.msg"
  "msg/UR5eTrajectory.msg"
)

set(srv_files
  "srv/MoverService.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
    std_msgs
    moveit_msgs
    sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()