cmake_minimum_required(VERSION 3.8)
project(ballrobot_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ImuRvc.msg"
  "msg/LeftRightFloat32.msg"
  "msg/LeftRightInt32.msg"
  "msg/ServoPulseWidth.msg"
  DEPENDENCIES geometry_msgs sensor_msgs std_msgs
)

# Python ROS nodes
install(PROGRAMS
  src/nodes/uwb_node.py
  src/nodes/imu_node.py
  src/nodes/imu_quaternion_node.py
  src/nodes/controllers/teensy_interface_node.py
  src/nodes/controllers/cmd_vel_bridge.py
  src/nodes/controllers/wasd_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

# Helper modules imported by the nodes above (not standalone
# executables, so no execute bit; installed to the same directory so
# they resolve on sys.path when their node is run)
install(FILES
  src/nodes/uwb_position_reader.py
  src/nodes/uwb_trilateration.py
  src/nodes/run_logging.py
  DESTINATION lib/${PROJECT_NAME}
)

# Launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()