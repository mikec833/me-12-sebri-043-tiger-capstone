cmake_minimum_required(VERSION 3.8)
project(ballrobot_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/YourMessage.msg"
  DEPENDENCIES geometry_msgs sensor_msgs
)

# Python ROS nodes
install(PROGRAMS
  src/nodes/your_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()