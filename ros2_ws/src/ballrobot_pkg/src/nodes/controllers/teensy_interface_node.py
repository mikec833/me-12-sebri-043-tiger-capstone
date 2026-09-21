#!/usr/bin/env python3
# Pi <-> Teensy serial bridge. 
# To add:
# reconnect handling, parameters, error recovery.
#
# Wire protocol (newline-terminated ASCII lines):
#   Pi -> Teensy:  "REF,<left_rad_s>,<right_rad_s>\n"
#   Teensy -> Pi:  "MEAS,<left_rad_s>,<right_rad_s>\n"


import rclpy
from rclpy.node import Node
import serial

from ballrobot_pkg.msg import LeftRightFloat32

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200


class TeensyInterfaceNode(Node):

    def __init__(self):
        super().__init__('teensy_interface_node')

        self.serial_conn = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0)
        self._seq = 0

        # self.subscription = self.create_subscription(
        #     MessageType,
        #     'topic_name',
        #     self.callback_function,
        #     10 - QoS setting - mostly 10
        # )
        
        # SUBSCRIBERS
        # Pi -> Teensy: wheel speed theta_l/r_dot_ref (rad/s)
        self.cmd_sub = self.create_subscription(
            LeftRightFloat32, 'wheel_speed_cmd', self.on_cmd, 10)

        # PUBLISHERS
        # Teensy -> Pi: measured wheel speed theta_l/r_dot
        self.speed_pub = self.create_publisher(LeftRightFloat32, 'wheel_speed_meas', 10)

        # Poll instead of blocking-read so callbacks still get serviced
        self.create_timer(0.02, self.poll_serial)  # 50 Hz

    def on_cmd(self, msg): # encodes two floats and writes to serial port
        line = f"REF,{msg.left:.3f},{msg.right:.3f}\n"
        self.serial_conn.write(line.encode('ascii'))

    def poll_serial(self): # reads wheel speed measurement from serial port and publishes it
        line = self.serial_conn.readline().decode('ascii', errors='ignore').strip()
        if not line.startswith('MEAS'):
            return

        _, left_rads, right_rads = line.split(',')

        speed_msg = LeftRightFloat32()
        speed_msg.left = float(left_rads)
        speed_msg.right = float(right_rads)
        speed_msg.seq_num = self._seq
        self.speed_pub.publish(speed_msg)

        self._seq += 1


def main(args=None):
    rclpy.init(args=args)
    node = TeensyInterfaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
