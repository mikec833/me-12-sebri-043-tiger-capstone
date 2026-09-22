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
import csv
import os

from ballrobot_pkg.msg import LeftRightFloat32

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

# Relative to wherever `ros2 run` is launched from (i.e. ros2_ws/)
OUTPUT_DIR = 'src/outputs'


class TeensyInterfaceNode(Node):

    def __init__(self):
        super().__init__('teensy_interface_node')

        self.serial_conn = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0)
        self._seq = 0

        # For the reference-vs-measured CSV export
        self.start_time = self.get_clock().now()
        self.cmd_time = []
        self.cmd_left = []
        self.cmd_right = []
        self.meas_time = []
        self.meas_left = []
        self.meas_right = []

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

        self.cmd_time.append(self.elapsed_seconds())
        self.cmd_left.append(msg.left)
        self.cmd_right.append(msg.right)

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

        self.meas_time.append(self.elapsed_seconds())
        self.meas_left.append(speed_msg.left)
        self.meas_right.append(speed_msg.right)

        self._seq += 1

    def elapsed_seconds(self):
        return (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

    def save_csv(self):
        os.makedirs(OUTPUT_DIR, exist_ok=True)

        with open(os.path.join(OUTPUT_DIR, 'wheel_speed_cmd.csv'), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time_s', 'left', 'right'])
            writer.writerows(zip(self.cmd_time, self.cmd_left, self.cmd_right))

        with open(os.path.join(OUTPUT_DIR, 'wheel_speed_meas.csv'), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time_s', 'left', 'right'])
            writer.writerows(zip(self.meas_time, self.meas_left, self.meas_right))


def main(args=None):
    rclpy.init(args=args)
    node = TeensyInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_csv()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
