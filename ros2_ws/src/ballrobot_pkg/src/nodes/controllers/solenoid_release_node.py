#!/usr/bin/env python3
"""
Forward the outer loop's feeder-box trigger to the solenoid XIAO over BLE.

Subscribes to std_msgs/Bool on /feeder_box_threshold_reached. On the first
True, writes "RELEASE:<token>" to the command characteristic of the XIAO
running xiao/solenoid_release/solenoid_release.ino, then reads its status
characteristic to confirm it fired. Retries every retry_period_s until
confirmed or release_timeout_s runs out. The XIAO treats repeat releases as
no-ops, so retrying is safe. Once confirmed, this node latches: later True
messages are ignored until restart, so the outer loop can publish True at
its own rate without re-firing.

A background task keeps a BLE connection to the XIAO open whenever it's in
range (scan -> connect -> reconnect on drop), so the trigger only pays for
one write + read (tens of ms) instead of a fresh scan/connect (1-2 s). BLE
runs on its own asyncio thread so it never blocks the rclpy executor.

Publishes std_msgs/Bool on /solenoid_released (transient-local, so late
subscribers still get it) once the XIAO has confirmed the release.

Requires bleak on the Pi:
    sudo apt install python3-bleak      (or: pip install bleak)
and the Pi's Bluetooth enabled -- `bluetoothctl show` must list a
controller. On a Pi 3/4 with the IMU on /dev/serial0, use
dtoverlay=miniuart-bt rather than disable-bt in /boot/firmware/config.txt.

Test without the outer loop:
    ros2 topic pub --once /feeder_box_threshold_reached std_msgs/msg/Bool "{data: true}"

CSV export
    On shutdown, writes src/outputs/run_<id>/solenoid_log_run<id>.csv
    (one row per event: time_s, stamp_s, event, detail). See run_logging.py.

Parameters
    xiao_name          str    "feeder-solenoid"  must match DEVICE_NAME in the sketch
    xiao_address       str    ""     BLE address printed by the XIAO on boot;
                                     if set, used instead of xiao_name
    token              str    "tiger-feeder"     must match TOKEN in the sketch
    scan_timeout_s     float  5.0
    retry_period_s     float  0.2
    release_timeout_s  float  15.0   give up if not confirmed within this
    run_id             str    ""
"""

import asyncio
import csv
import os
import sys
import threading
import time

import rclpy
from bleak import BleakClient, BleakScanner
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool

# See cmd_vel_bridge.py for why this is needed for the flat run_logging import.
sys.path.insert(0, os.path.dirname(os.path.abspath(sys.argv[0])))

from run_logging import resolve_run_output_dir

# Must match solenoid_release.ino
CMD_UUID = '6e0f0002-3b5a-4c2e-9d1a-2f5c8e7b4a10'
STATUS_UUID = '6e0f0003-3b5a-4c2e-9d1a-2f5c8e7b4a10'


class SolenoidReleaseNode(Node):

    def __init__(self):
        super().__init__('solenoid_release_node')

        self.declare_parameter('xiao_name', 'feeder-solenoid')
        self.declare_parameter('xiao_address', '')
        self.declare_parameter('token', 'tiger-feeder')
        self.declare_parameter('scan_timeout_s', 5.0)
        self.declare_parameter('retry_period_s', 0.2)
        self.declare_parameter('release_timeout_s', 15.0)
        # Shared across nodes in the same bringup; see run_logging.py.
        self.declare_parameter('run_id', '')

        self.xiao_name = str(self.get_parameter('xiao_name').value)
        self.xiao_address = str(self.get_parameter('xiao_address').value).strip()
        self.token = str(self.get_parameter('token').value)
        self.scan_timeout = float(self.get_parameter('scan_timeout_s').value)
        self.retry_period = float(self.get_parameter('retry_period_s').value)
        self.release_timeout = float(self.get_parameter('release_timeout_s').value)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.released_pub = self.create_publisher(Bool, 'solenoid_released', latched)
        self.released_pub.publish(Bool(data=False))
        self.sub = self.create_subscription(
            Bool, 'feeder_box_threshold_reached', self.trigger_callback, 10)

        self.lock = threading.Lock()
        self.in_flight = False
        self.released = False
        self.client = None
        self.stopping = False

        self.start_time = self.get_clock().now()
        self.log_rows = []

        self.loop = asyncio.new_event_loop()
        self.ble_thread = threading.Thread(target=self.run_loop, daemon=True)
        self.ble_thread.start()
        asyncio.run_coroutine_threadsafe(self.connection_manager(), self.loop)

    # ---- rclpy side ----------------------------------------------------

    def trigger_callback(self, msg: Bool):
        if not msg.data:
            return
        with self.lock:
            if self.released or self.in_flight:
                return
            self.in_flight = True
        self.log_event('trigger', '')
        self.get_logger().info('Feeder box threshold reached -- releasing solenoid')
        asyncio.run_coroutine_threadsafe(self.release(), self.loop)

    # ---- BLE side (runs on self.loop) ------------------------------------

    def run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def target_desc(self):
        return self.xiao_address or f'"{self.xiao_name}"'

    async def connection_manager(self):
        while not self.stopping:
            if self.client is not None and self.client.is_connected:
                await asyncio.sleep(0.5)
                continue
            try:
                if self.xiao_address:
                    device = await BleakScanner.find_device_by_address(
                        self.xiao_address, timeout=self.scan_timeout)
                else:
                    device = await BleakScanner.find_device_by_name(
                        self.xiao_name, timeout=self.scan_timeout)
                if device is None:
                    self.get_logger().warn(
                        f'XIAO {self.target_desc()} not found (out of range or '
                        'not powered?) -- still scanning',
                        throttle_duration_sec=10.0)
                    continue
                client = BleakClient(device, disconnected_callback=self.on_disconnect)
                await client.connect(timeout=10.0)
                status = (await client.read_gatt_char(STATUS_UUID)).decode().strip()
                self.client = client
                self.log_event('connected', f'{device.address} status={status}')
                self.get_logger().info(
                    f'Connected to XIAO {device.address} (status: {status})')
            except Exception as e:  # bleak raises a mix of BleakError/OSError/TimeoutError
                self.log_event('connect_fail', str(e))
                self.get_logger().warn(
                    f'BLE connect to XIAO failed: {e}', throttle_duration_sec=5.0)
                await asyncio.sleep(1.0)

    def on_disconnect(self, _client):
        self.client = None
        if not self.stopping:
            self.log_event('disconnected', '')
            self.get_logger().warn('XIAO BLE link dropped -- reconnecting')

    async def release(self):
        attempt = 0
        t0 = time.monotonic()
        while time.monotonic() - t0 < self.release_timeout and not self.stopping:
            client = self.client
            if client is None or not client.is_connected:
                await asyncio.sleep(0.1)
                continue
            attempt += 1
            try:
                await client.write_gatt_char(
                    CMD_UUID, f'RELEASE:{self.token}'.encode(), response=True)
                status = (await client.read_gatt_char(STATUS_UUID)).decode().strip()
            except Exception as e:
                self.log_event('write_fail', f'{e} attempt={attempt}')
                self.get_logger().warn(
                    f'Release write failed (attempt {attempt}): {e}',
                    throttle_duration_sec=1.0)
                await asyncio.sleep(self.retry_period)
                continue

            if status in ('released', 'already_released'):
                latency_ms = (time.monotonic() - t0) * 1e3
                with self.lock:
                    self.released = True
                    self.in_flight = False
                self.released_pub.publish(Bool(data=True))
                self.log_event('ack', f'{status} attempt={attempt} latency_ms={latency_ms:.0f}')
                self.get_logger().info(
                    f'XIAO confirmed: {status} (attempt {attempt}, {latency_ms:.0f} ms)')
                return
            if status == 'bad_token':
                # Retrying won't fix a token mismatch.
                self.log_event('bad_token', f'attempt={attempt}')
                self.get_logger().error('XIAO rejected release: token mismatch')
                break
            self.log_event('unexpected_status', f'{status} attempt={attempt}')
            await asyncio.sleep(self.retry_period)

        # Gave up: clear in_flight so the next True from the outer loop retries.
        with self.lock:
            self.in_flight = False
        self.log_event('gave_up', f'attempts={attempt}')
        self.get_logger().error(
            f'Solenoid release NOT confirmed after {attempt} attempts '
            f'({self.release_timeout:.0f} s)')

    async def disconnect(self):
        if self.client is not None and self.client.is_connected:
            await self.client.disconnect()

    # ---- logging / shutdown --------------------------------------------

    def log_event(self, event, detail):
        now = self.get_clock().now()
        with self.lock:
            self.log_rows.append((
                (now - self.start_time).nanoseconds * 1e-9,
                now.nanoseconds * 1e-9,
                event,
                detail,
            ))

    def save_csv(self):
        run_id = (self.get_parameter('run_id').value or '').strip()
        if not run_id:
            run_id = time.strftime('%Y%m%d_%H%M%S')
        run_dir = resolve_run_output_dir(run_id)
        filename = f'solenoid_log_run{run_id}.csv'

        with open(os.path.join(run_dir, filename), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time_s', 'stamp_s', 'event', 'detail'])
            with self.lock:
                writer.writerows(self.log_rows)

        self.get_logger().info(f'Saved {filename} in {run_dir}')

    def destroy_node(self):
        self.stopping = True
        try:
            asyncio.run_coroutine_threadsafe(self.disconnect(), self.loop).result(timeout=3.0)
        except Exception:
            pass
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.save_csv()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SolenoidReleaseNode()
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
