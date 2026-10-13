#!/usr/bin/env python3
# Pi <-> drive XIAO serial bridge (historically the Teensy, hence the name).
#
# Wire protocol (newline-terminated ASCII lines):
#   XIAO -> Pi:  "MEAS,<left_rad_s>,<right_rad_s>\n"
#            or  "MEAS,<left_rad_s>,<right_rad_s>,<left_rad>,<right_rad>\n"
#   (the optional trailing pair is total wheel angle from the encoders; the
#   Teensy firmware sends the 3-field form, a board that also reports
#   position can send the 5-field form)
#   XIAO -> Pi (firmware v4.7): STATE,..., FAULT,..., OWNER,<NONE|BLE|SERIAL>,
#            BAT,<volts>,<low>, VERSION,<x>
#   Pi -> XIAO (firmware v4.7): the website's command grammar, e.g.
#            "C,<accel_ms>,<brake_ms>,<max_percent>", "A", "K", "R", "S",
#            "M,<left_pct>,<right_pct>", "P,<left_pct>,<right_pct>"
#   The old "REF,<left_rad_s>,<right_rad_s>" output was a Teensy-era wheel
#   speed reference; the XIAO firmware never used it, so it is no longer sent.
#
# Measurements are published on wheel_states as sensor_msgs/JointState,
# names ["left_wheel", "right_wheel"], velocity in rad/s, position in rad
# (empty when the board only sends the 3-field form), header.stamp is the
# time the line was read.
#
# Driving (firmware v4.7): /cmd_vel (geometry_msgs/Twist) is converted to an
# M or P line by drive_command.py and sent at cmd_rate_hz, but only while this
# node has armed the robot and owns control (OWNER,SERIAL). A cmd_vel older
# than cmd_timeout_s sends S instead. Arm with the ~/arm service (sends C,
# then A, and waits for STATE,ARMED). The keeper website can always take over
# (OWNER,BLE) or stop the robot; this node never re-arms by itself.
#   Services   ~/arm, ~/disarm, ~/reset_fault   (std_srvs/Trigger)
#   Publishes  /drive/status (std_msgs/String, every non-MEAS line)
#              /drive/armed  (std_msgs/Bool, armed under SERIAL, latched)
#              /drive/owner  (std_msgs/String, NONE|BLE|SERIAL|UNKNOWN, latched)
# Drive parameters come from the shared setup file (config/<name>.yaml,
# teensy_interface_node block); the defaults below are conservative.


import sys
import threading
import time

import rclpy
from rclpy.node import Node
import serial
import csv
import os

# run_logging.py is installed as a flat sibling of this script (see
# CMakeLists.txt), but this source file itself lives one directory
# down (src/nodes/controllers/, not src/nodes/). With
# `colcon build --symlink-install`, the installed copy is a symlink
# back to this source file, and Python's default sys.path[0] follows
# that symlink to its real (source-tree) directory rather than the
# flat install directory -- so the bare `import run_logging` below
# would only resolve by accident. Adding the script's own invoked
# location (sys.argv[0], not its symlink target) makes it resolve
# deterministically regardless of interpreter/OS symlink behavior.
sys.path.insert(0, os.path.dirname(os.path.abspath(sys.argv[0])))

from ballrobot_pkg.msg import LeftRightFloat32
from geometry_msgs.msg import Twist
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from drive_command import DriveParams, command_line
from run_logging import resolve_run_output_dir

DEFAULT_SERIAL_PORT = '/dev/ttyACM0'
WHEEL_NAMES = ['left_wheel', 'right_wheel']
BAUD_RATE = 115200
RECONNECT_PERIOD_S = 2.0
MAX_PARTIAL_LINE_BYTES = 1024
WRITE_TIMEOUT_S = 0.05  # a stuck USB write gives up instead of stalling MEAS

# Relative to wherever `ros2 run` is launched from (i.e. ros2_ws/)
OUTPUT_DIR = 'src/outputs/wheel_outputs'


class TeensyInterfaceNode(Node):

    def __init__(self):
        super().__init__('teensy_interface_node')

        self.declare_parameter('serial_port', DEFAULT_SERIAL_PORT)
        self._serial_port = self.get_parameter('serial_port').value
        # Shared across nodes in the same bringup; see run_logging.py.
        self.declare_parameter('run_id', '')

        # Drive parameters. TODO calibrate on the ground (brief section 2.2);
        # these defaults are deliberately slow. Override them per setup in the
        # shared config file, teensy_interface_node block.
        self.declare_parameter('accel_ms', 800)      # firmware C, 100..4000
        self.declare_parameter('brake_ms', 700)      # firmware C, 100..4000
        self.declare_parameter('max_percent', 20)    # firmware C, 10..100, integer
        self.declare_parameter('r_eff', 0.072)       # m, cmd_vel_bridge default
        self.declare_parameter('b_eff', 0.22)        # m, cmd_vel_bridge default
        self.declare_parameter('p0', 3.0)            # %, wheels-raised deadband
        self.declare_parameter('kp', 1.8)            # % per rad/s, wheels-raised fit
        self.declare_parameter('eps_v', 0.02)        # m/s
        self.declare_parameter('cmd_rate_hz', 20.0)
        self.declare_parameter('cmd_timeout_s', 0.3)
        self.declare_parameter('arm_timeout_s', 0.5)
        gp = self.get_parameter
        self._accel_ms = int(gp('accel_ms').value)
        self._brake_ms = int(gp('brake_ms').value)
        self._max_percent = int(round(float(gp('max_percent').value)))
        if not (100 <= self._accel_ms <= 4000 and 100 <= self._brake_ms <= 4000
                and 10 <= self._max_percent <= 100):
            raise ValueError('accel_ms/brake_ms must be 100..4000 and max_percent 10..100 '
                             '(firmware C limits)')
        self._drive = DriveParams(
            r_eff=float(gp('r_eff').value), b_eff=float(gp('b_eff').value),
            p0=float(gp('p0').value), kp=float(gp('kp').value),
            max_percent=float(self._max_percent), eps_v=float(gp('eps_v').value))
        self._cmd_timeout_s = float(gp('cmd_timeout_s').value)
        self._arm_timeout_s = float(gp('arm_timeout_s').value)

        # Drive state, from the firmware's status lines
        self._armed = False          # armed with SERIAL as owner
        self._owner = 'UNKNOWN'
        self._firmware_version = None
        self._last_twist = None      # (v, omega, monotonic receipt time)
        self._arm_event = threading.Event()
        self._arm_reply = None
        # Guards writes and (re)connects. Reads stay outside it, so a slow
        # write never delays MEAS. Re-entrant: write_line connects under it.
        self._write_lock = threading.RLock()

        self.serial_conn = None
        self._last_connect_attempt = 0.0
        # Bytes received but not yet terminated by a newline; only complete
        # lines are parsed, so a line split across two reads is never
        # mistaken for a shorter valid one.
        self._rx_buffer = bytearray()
        self._connect()

        # For the reference-vs-measured CSV export: one row per measurement,
        # tagged with whatever reference was in effect at that moment
        self.start_time = self.get_clock().now()
        self.ref_left = 0.0
        self.ref_right = 0.0
        self.log_time = []
        self.log_stamp_s = []
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
                
        # Reading, driving and the blocking ~/arm service run in separate
        # callback groups on a MultiThreadedExecutor, so a service waiting
        # for STATE,ARMED never holds up MEAS.
        rx_group = MutuallyExclusiveCallbackGroup()
        tx_group = MutuallyExclusiveCallbackGroup()
        srv_group = MutuallyExclusiveCallbackGroup()

        # SUBSCRIBERS
        # cmd_vel_bridge.py wheel speed reference: logged to the CSV only
        self.cmd_sub = self.create_subscription(
            LeftRightFloat32, 'wheel_speed_cmd', self.on_cmd, 10)
        self.create_subscription(
            Twist, '/cmd_vel', self.on_cmd_vel, 10, callback_group=tx_group)

        # PUBLISHERS
        # Teensy -> Pi: measured wheel speed theta_l/r_dot (and angle, if sent)
        self.wheel_pub = self.create_publisher(JointState, 'wheel_states', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '/drive/status', 50)
        self.armed_pub = self.create_publisher(Bool, '/drive/armed', latched)
        self.owner_pub = self.create_publisher(String, '/drive/owner', latched)
        self._publish_drive_state()

        # SERVICES
        self.create_service(Trigger, '~/arm', self.srv_arm, callback_group=srv_group)
        self.create_service(Trigger, '~/disarm', self.srv_disarm, callback_group=srv_group)
        self.create_service(
            Trigger, '~/reset_fault', self.srv_reset_fault, callback_group=srv_group)

        # Poll instead of blocking-read so callbacks still get serviced
        self.create_timer(0.02, self.poll_serial, callback_group=rx_group)  # 50 Hz
        self.create_timer(
            1.0 / float(gp('cmd_rate_hz').value), self.send_drive, callback_group=tx_group)
        self.write_line('V')  # logs the firmware version; v4.6 never answers

    def _connect(self):
        if self.serial_conn is not None:
            return
        self._last_connect_attempt = time.monotonic()
        try:
            self.serial_conn = serial.Serial(
                self._serial_port, BAUD_RATE, timeout=0, write_timeout=WRITE_TIMEOUT_S)
            self.get_logger().info(f'Connected to {self._serial_port}, node up and spinning.')
        except serial.SerialException as exc:
            self.get_logger().warn(
                f'Teensy link fault ({exc}), reconnecting in {RECONNECT_PERIOD_S:.1f} s',
                throttle_duration_sec=5.0)
            self.serial_conn = None

    def _disconnect(self):
        try:
            if self.serial_conn is not None:
                self.serial_conn.close()
        except Exception:
            pass
        self.serial_conn = None
        self._rx_buffer.clear()

    def _ensure_connected(self):
        if self.serial_conn is not None:
            return True
        if time.monotonic() - self._last_connect_attempt < RECONNECT_PERIOD_S:
            return False
        self._connect()
        return self.serial_conn is not None

    def on_cmd(self, msg):  # wheel speed reference, CSV only (REF is no longer sent)
        self.ref_left = msg.left
        self.ref_right = msg.right

    # ---- driving (firmware v4.7) ----------------------------------------

    def write_line(self, line):
        """Send one command line. Every write goes through this lock."""
        with self._write_lock:
            if not self._ensure_connected():
                return False
            try:
                self.serial_conn.write((line + '\n').encode('ascii'))
                return True
            except (serial.SerialException, serial.SerialTimeoutException, OSError):
                self._disconnect()
                return False

    def on_cmd_vel(self, msg):
        self._last_twist = (msg.linear.x, msg.angular.z, time.monotonic())

    def send_drive(self):
        if not (self._armed and self._owner == 'SERIAL'):
            return  # never drive, or feed the watchdog, unless we own an armed robot
        twist = self._last_twist
        if twist is None or time.monotonic() - twist[2] > self._cmd_timeout_s:
            self.write_line('S')
            return
        self.write_line(command_line(twist[0], twist[1], self._drive))

    def _publish_drive_state(self):
        self.armed_pub.publish(Bool(data=self._armed))
        self.owner_pub.publish(String(data=self._owner))

    def handle_status(self, line):
        """Track arm and owner state from a non-MEAS firmware line."""
        self.status_pub.publish(String(data=line))
        kind = line.split(',', 1)[0]
        before = (self._armed, self._owner)
        if line == 'STATE,ARMED':
            self._armed, self._owner = True, 'SERIAL'
        elif line in ('STATE,DISARMED', 'STATE,TIMEOUT_DISARMED', 'STATE,FAULT_RESET'):
            self._armed = False
        elif kind == 'FAULT':
            self._armed = False
        elif kind == 'OWNER':
            self._owner = line.split(',', 1)[1] if ',' in line else 'UNKNOWN'
            self._armed = self._armed and self._owner == 'SERIAL'
        elif kind == 'VERSION':
            self._firmware_version = line.split(',', 1)[1] if ',' in line else line
            self.get_logger().info(f'Drive firmware {line}')

        if kind == 'BAT':
            self.get_logger().debug(f'XIAO: {line}')
        elif kind not in ('VERSION',):
            self.get_logger().info(f'XIAO: {line}')
        if line == 'STATE,ARMED' or line.startswith('STATE,BUSY') or kind == 'FAULT':
            self._arm_reply = line
            self._arm_event.set()
        if (self._armed, self._owner) != before:
            self._publish_drive_state()

    def srv_arm(self, request, response):
        for attempt in (1, 2):
            self._arm_event.clear()
            self._arm_reply = None
            if not (self.write_line(f'C,{self._accel_ms},{self._brake_ms},{self._max_percent}')
                    and self.write_line('A')):
                response.success, response.message = False, 'serial port not available'
                return response
            if self._arm_event.wait(self._arm_timeout_s):
                reply = self._arm_reply
                response.success = reply == 'STATE,ARMED'
                response.message = 'armed' if response.success else reply.split(',', 1)[1]
                return response
            self.get_logger().warn(f'No reply to arm request (attempt {attempt})')
        response.success = False
        response.message = 'timeout'
        if self._firmware_version is None:
            response.message += ' (no VERSION reply: is firmware v4.7 flashed?)'
        return response

    def srv_disarm(self, request, response):
        response.success = self.write_line('K')
        response.message = 'K sent' if response.success else 'serial port not available'
        if response.success and self._armed:
            self._armed = False
            self._publish_drive_state()
        return response

    def srv_reset_fault(self, request, response):
        response.success = self.write_line('R')
        response.message = 'R sent' if response.success else 'serial port not available'
        return response

    def shutdown_drive(self):
        """Disarm on the way out; the firmware also disarms on port close."""
        if self.serial_conn is not None:
            self.write_line('K')

    def poll_serial(self): # drains every complete line currently buffered, not just one
        with self._write_lock:
            if not self._ensure_connected():
                return
            conn = self.serial_conn

        try:
            waiting = conn.in_waiting
            if waiting:
                self._rx_buffer += conn.read(waiting)
        except (serial.SerialException, OSError, AttributeError, TypeError):
            with self._write_lock:
                if self.serial_conn is conn:
                    self._disconnect()
            return

        *lines, remainder = self._rx_buffer.split(b'\n')
        self._rx_buffer = bytearray(remainder)
        if len(self._rx_buffer) > MAX_PARTIAL_LINE_BYTES:
            self.get_logger().warn('No newline in serial data, discarding buffer')
            self._rx_buffer.clear()

        for raw in lines:
            line = raw.decode('ascii', errors='ignore').strip()
            if not line:
                continue
            if not line.startswith('MEAS'):
                self.handle_status(line)
                continue

            parts = line.split(',')
            if len(parts) not in (3, 5):
                self.get_logger().warn(f'Malformed line, skipping: {line}')
                continue

            try:
                values = [float(p) for p in parts[1:]]
            except ValueError:
                self.get_logger().warn(f'Malformed line, skipping: {line}')
                continue
            left, right = values[0], values[1]

            now = self.get_clock().now()
            wheel_msg = JointState()
            wheel_msg.header.stamp = now.to_msg()
            wheel_msg.name = WHEEL_NAMES
            wheel_msg.velocity = [left, right]
            if len(values) == 4:
                wheel_msg.position = [values[2], values[3]]
            self.wheel_pub.publish(wheel_msg)

            self.log_time.append(self.elapsed_seconds())
            self.log_stamp_s.append(now.nanoseconds * 1e-9)
            self.log_ref_left.append(self.ref_left)
            self.log_ref_right.append(self.ref_right)
            self.log_meas_left.append(left)
            self.log_meas_right.append(right)

    def elapsed_seconds(self):
        return (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

    def save_csv(self):
        run_id = (self.get_parameter('run_id').value or '').strip()
        if not run_id:
            run_id = time.strftime('%Y%m%d_%H%M%S')
        run_dir = resolve_run_output_dir(run_id)
        filename = f'wheel_speed_log_run{run_id}.csv'

        with open(os.path.join(run_dir, filename), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(
                ['time_s', 'stamp_s', 'ref_left', 'ref_right', 'meas_left', 'meas_right']
            )
            writer.writerows(zip(
                self.log_time, self.log_stamp_s, self.log_ref_left, self.log_ref_right,
                self.log_meas_left, self.log_meas_right))

        self.get_logger().info(f'Saved {filename} in {run_dir}')


def main(args=None):
    rclpy.init(args=args)
    node = TeensyInterfaceNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_drive()
        node.save_csv()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
