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
import glob

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

        # For the reference-vs-measured CSV export: one row per measurement,
        # tagged with whatever reference was in effect at that moment
        self.start_time = self.get_clock().now()
        self.ref_left = 0.0
        self.ref_right = 0.0
        self.log_time = []
        self.log_ref_left = []
        self.log_ref_right = []
        self.log_meas_left = []
        self.log_meas_right = []

        # self.subscription = self.create_subscription(
        #     MessageType,
        #     'topic_name',
        #     self.callback_function,
        #     10 - QoS setting - mostly 10
        # )
        
        # self.publisher = self.create_publisher(
        #     MessageType,
        #     'topic_name',
        #     10
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

        self.get_logger().info(f'Connected to {SERIAL_PORT}, node up and spinning.')

    def on_cmd(self, msg): # encodes two floats and writes to serial port
        line = f"REF,{msg.left:.3f},{msg.right:.3f}\n"
        self.serial_conn.write(line.encode('ascii'))
        self.get_logger().info(f'Sent {line.strip()}')

        self.ref_left = msg.left
        self.ref_right = msg.right

    def poll_serial(self): # drains every line currently buffered, not just one
        while True:
            line = self.serial_conn.readline().decode('ascii', errors='ignore').strip()
            if not line:
                break
            if not line.startswith('MEAS'):
                self.get_logger().info(f'Teensy: {line}')
                continue

            parts = line.split(',')
            if len(parts) != 3:
                self.get_logger().warn(f'Malformed line, skipping: {line}')
                continue

            try:
                left = float(parts[1])
                right = float(parts[2])
            except ValueError:
                self.get_logger().warn(f'Malformed line, skipping: {line}')
                continue

            speed_msg = LeftRightFloat32()
            speed_msg.left = left
            speed_msg.right = right
            speed_msg.seq_num = self._seq
            self.speed_pub.publish(speed_msg)

            self.log_time.append(self.elapsed_seconds())
            self.log_ref_left.append(self.ref_left)
            self.log_ref_right.append(self.ref_right)
            self.log_meas_left.append(left)
            self.log_meas_right.append(right)

            self._seq += 1

    def elapsed_seconds(self):
        return (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

    def next_run_number(self):
        existing = glob.glob(os.path.join(OUTPUT_DIR, 'wheel_speed_log_run*.csv'))
        run_numbers = [0]
        for path in existing:
            digits = os.path.basename(path)[len('wheel_speed_log_run'):-len('.csv')]
            if digits.isdigit():
                run_numbers.append(int(digits))
        return max(run_numbers) + 1

    def save_csv(self):
        os.makedirs(OUTPUT_DIR, exist_ok=True)
        filename = f'wheel_speed_log_run{self.next_run_number()}.csv'

        with open(os.path.join(OUTPUT_DIR, filename), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time_s', 'ref_left', 'ref_right', 'meas_left', 'meas_right'])
            writer.writerows(zip(
                self.log_time, self.log_ref_left, self.log_ref_right,
                self.log_meas_left, self.log_meas_right))

        self.get_logger().info(f'Saved {filename}')


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
