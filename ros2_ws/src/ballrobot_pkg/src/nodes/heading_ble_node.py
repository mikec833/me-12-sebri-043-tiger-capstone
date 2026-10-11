#!/usr/bin/env python3
"""
Stream IMU heading from the Pi to the operator GUI over BLE.

The Pi acts as a BLE peripheral (GATT server), the same role the drive
XIAO plays for the GUI, so the browser connects to it with Web Bluetooth
and no Wi-Fi is needed. Subscribes to imu/rpy (geometry_msgs/Vector3Stamped
from imu_node.py: x = roll, y = pitch, z = yaw, degrees), keeps the latest
sample, and notifies it at notify_rate_hz.

Characteristic value (ASCII, one complete sample per notification, no
newline framing):
    "<yaw>,<pitch>,<roll>"   degrees, 1 decimal, e.g. "-123.4,1.2,-0.8"
Worst case "-179.9,-179.9,-179.9" is exactly 20 bytes, so it fits the
default 23-byte ATT MTU without the GUI having to reassemble anything.
Nothing is notified until the first IMU frame arrives, and nothing new is
notified if imu/rpy stops, so the GUI's stale indicator trips.

BNO085 RVC yaw is relative to the heading at IMU power-up, not magnetic
north; the GUI's "Zero heading" button sets the reference.

BLE runs on its own asyncio thread so it never blocks the rclpy executor
(same pattern as solenoid_release_node.py). The Pi can be the BLE central
for the solenoid XIAO and this peripheral at the same time on one adapter.

Requires bless on the Pi (BlueZ GATT server; not packaged in apt):
    pip install bless        (add --break-system-packages on Ubuntu 24.04)
and Bluetooth enabled -- `bluetoothctl show` must list a controller and
`Powered: yes`. With the IMU on /dev/serial0, use dtoverlay=miniuart-bt
rather than disable-bt in /boot/firmware/config.txt. If start() fails with
a D-Bus permission error, add the user to the bluetooth group:
    sudo usermod -aG bluetooth $USER    (then log out and back in)

Test without the GUI: nRF Connect (phone) -> scan for "TigerBall-Pi" ->
connect -> enable notifications on the heading characteristic.

Parameters
    device_name      str    "TigerBall-Pi"  must match PI_NAME in the GUI
    notify_rate_hz   float  20.0            GUI update rate; the IMU runs ~100 Hz
"""

import asyncio
import threading

import rclpy
from bless import (
    BlessServer,
    GATTAttributePermissions,
    GATTCharacteristicProperties,
)
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

# Must match PI_SERVICE / PI_HEADING in controller_v4_6.html
SERVICE_UUID = '7b6a2001-6c3a-4c9f-ae60-62b79b0e5135'
HEADING_UUID = '7b6a2002-6c3a-4c9f-ae60-62b79b0e5135'


def format_sample(msg: Vector3Stamped) -> bytes:
    roll, pitch, yaw = msg.vector.x, msg.vector.y, msg.vector.z
    return f'{yaw:.1f},{pitch:.1f},{roll:.1f}'.encode('ascii')[:20]


class HeadingBleNode(Node):

    def __init__(self):
        super().__init__('heading_ble_node')

        self.declare_parameter('device_name', 'TigerBall-Pi')
        self.declare_parameter('notify_rate_hz', 20.0)

        self.device_name = str(self.get_parameter('device_name').value)
        rate = float(self.get_parameter('notify_rate_hz').value)

        self.lock = threading.Lock()
        self.latest = None   # newest formatted sample not yet notified
        self.server = None
        self.sent_count = 0

        self.sub = self.create_subscription(
            Vector3Stamped, 'imu/rpy', self.imu_callback, qos_profile_sensor_data)
        self.timer = self.create_timer(1.0 / max(rate, 1.0), self.notify_tick)
        self.status_timer = self.create_timer(10.0, self.report_status)

        self.loop = asyncio.new_event_loop()
        self.ble_thread = threading.Thread(target=self.run_loop, daemon=True)
        self.ble_thread.start()
        asyncio.run_coroutine_threadsafe(self.start_server(), self.loop)

    # ---- rclpy side ----------------------------------------------------

    def imu_callback(self, msg: Vector3Stamped):
        sample = format_sample(msg)
        with self.lock:
            self.latest = sample

    def notify_tick(self):
        with self.lock:
            sample, self.latest = self.latest, None
        if sample is None or self.server is None:
            return
        self.loop.call_soon_threadsafe(self.push, sample)

    def report_status(self):
        self.get_logger().info(
            f'{self.sent_count} heading notifications sent in the last 10 s')
        self.sent_count = 0

    # ---- BLE side (runs on self.loop) ------------------------------------

    def run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    async def start_server(self):
        try:
            server = BlessServer(name=self.device_name, loop=self.loop)
            server.read_request_func = lambda characteristic, **_: characteristic.value
            await server.add_new_service(SERVICE_UUID)
            await server.add_new_characteristic(
                SERVICE_UUID, HEADING_UUID,
                GATTCharacteristicProperties.read | GATTCharacteristicProperties.notify,
                bytearray(b'0.0,0.0,0.0'),
                GATTAttributePermissions.readable)
            await server.start()
        except Exception as e:  # BlueZ/D-Bus errors surface as a mix of types
            self.get_logger().error(
                f'BLE GATT server failed to start: {e} -- is Bluetooth powered '
                '(`bluetoothctl show`) and is bless installed?')
            return
        self.server = server
        self.get_logger().info(f'Advertising as "{self.device_name}"')

    def push(self, sample: bytes):
        try:
            self.server.get_characteristic(HEADING_UUID).value = bytearray(sample)
            self.server.update_value(SERVICE_UUID, HEADING_UUID)
            self.sent_count += 1
        except Exception as e:
            self.get_logger().warn(f'Heading notify failed: {e}', throttle_duration_sec=5.0)

    async def stop_server(self):
        if self.server is not None:
            await self.server.stop()

    def destroy_node(self):
        try:
            asyncio.run_coroutine_threadsafe(self.stop_server(), self.loop).result(timeout=3.0)
        except Exception:
            pass
        self.loop.call_soon_threadsafe(self.loop.stop)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HeadingBleNode()
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
