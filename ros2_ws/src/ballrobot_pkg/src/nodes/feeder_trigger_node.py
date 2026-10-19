#!/usr/bin/env python3
"""
feeder_trigger_node.py

Decides when the robot has reached the feeder box, from the UWB
position. Compares the (x, y) of each uwb/position fix with the
configured feeder position (z is ignored) and publishes True on
/feeder_box_threshold_reached once the robot has been within
trigger_radius_m for confirm_samples fixes in a row. Requiring several
fixes stops one noisy UWB reading near the boundary from releasing
food. It then stays True until the robot is more than
trigger_radius_m + exit_margin_m away for confirm_samples fixes in a
row, so UWB noise at the boundary can't flip it on and off.
solenoid_release_node.py turns that True into the BLE OPEN
command to the feeder-box XIAO and latches, so this node can keep
publishing True while the robot stays in the zone.

Set feeder_position and trigger_radius_m per anchor layout in the
config YAMLs (config/<name>.yaml, block `feeder_trigger_node`); the
feeder position is in the same frame as the anchor_positions there.

Subscribed topics
    uwb/position                  geometry_msgs/PointStamped

Published topics
    feeder_box_threshold_reached  std_msgs/Bool     one per UWB fix
    feeder/distance               std_msgs/Float32  (x, y) distance to
                                                    the feeder, m (debug)

Test without the robot moving (feeder at the origin, radius 0.5 m):
    ros2 topic pub -r 10 /uwb/position geometry_msgs/msg/PointStamped \\
        "{header: {frame_id: map}, point: {x: 0.2, y: 0.1, z: 0.0}}"
    ros2 topic echo /feeder_box_threshold_reached

Parameters (read only, set at startup)
    feeder_position    float[2]  [0.0, 0.0]  feeder (x, y), m
    trigger_radius_m   float     0.5         trigger when this close
    exit_margin_m      float     0.2         extra distance beyond the
                                             radius before leaving the zone
    confirm_samples    int       3           consecutive fixes needed to
                                             enter or leave the zone
"""

import math

import rclpy
from geometry_msgs.msg import PointStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Float32


class FeederTriggerNode(Node):

    def __init__(self):
        super().__init__('feeder_trigger_node')

        read_only = ParameterDescriptor(read_only=True)
        # dynamic_typing: accept whole numbers in the YAML too, e.g.
        # [4, 4] or 1, which ROS would otherwise reject as the wrong type
        read_only_number = ParameterDescriptor(read_only=True, dynamic_typing=True)
        self.declare_parameter('feeder_position', [0.0, 0.0], read_only_number)
        self.declare_parameter('trigger_radius_m', 0.5, read_only_number)
        self.declare_parameter('exit_margin_m', 0.2, read_only_number)
        self.declare_parameter('confirm_samples', 3, read_only)

        position = [float(v) for v in self.get_parameter('feeder_position').value]
        if len(position) != 2:
            raise ValueError(
                f'feeder_position must hold 2 values (x, y), got {len(position)}')
        self.feeder_x, self.feeder_y = position
        self.radius = float(self.get_parameter('trigger_radius_m').value)
        self.exit_radius = self.radius + float(self.get_parameter('exit_margin_m').value)
        self.confirm_samples = max(1, int(self.get_parameter('confirm_samples').value))

        # Consecutive fixes on the other side of the boundary from the
        # current state (inside trigger_radius_m while not triggered,
        # beyond exit_radius while triggered)
        self.change_count = 0
        self.triggered = False

        self.trigger_pub = self.create_publisher(Bool, 'feeder_box_threshold_reached', 10)
        self.distance_pub = self.create_publisher(Float32, 'feeder/distance', 10)
        self.create_subscription(
            PointStamped, 'uwb/position', self.position_callback, qos_profile_sensor_data)

        self.get_logger().info(
            f'Feeder at ({self.feeder_x:.2f}, {self.feeder_y:.2f}) m, '
            f'trigger within {self.radius:.2f} m, leave beyond {self.exit_radius:.2f} m, '
            f'after {self.confirm_samples} fixes')

    def position_callback(self, msg: PointStamped):
        distance = math.hypot(msg.point.x - self.feeder_x, msg.point.y - self.feeder_y)
        self.distance_pub.publish(Float32(data=distance))

        if self.triggered:
            crossing = distance > self.exit_radius
        else:
            crossing = distance <= self.radius
        self.change_count = self.change_count + 1 if crossing else 0

        if self.change_count >= self.confirm_samples:
            self.change_count = 0
            self.triggered = not self.triggered
            if self.triggered:
                self.get_logger().info(
                    f'Reached feeder: ({msg.point.x:.2f}, {msg.point.y:.2f}) m is '
                    f'{distance:.2f} m away -- requesting OPEN')
            else:
                self.get_logger().info(f'Left feeder zone ({distance:.2f} m away)')
        self.trigger_pub.publish(Bool(data=self.triggered))


def main(args=None):
    rclpy.init(args=args)
    node = FeederTriggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
