cmake_minimum_required(VERSION 3.8)
project(ballrobot_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LeftRightFloat32.msg"
  "msg/LeftRightInt32.msg"
  "msg/ServoPulseWidth.msg"
  DEPENDENCIES geometry_msgs sensor_msgs
)

# Python ROS nodes
install(PROGRAMS
  src/nodes/controllers/teensy_interface_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()