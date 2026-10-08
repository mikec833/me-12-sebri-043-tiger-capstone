#!/usr/bin/env python3
"""
Play a sound file (animal noises) out of the Pi's 3.5 mm audio jack.

Playback is done by an mpv subprocess, which decodes mp4/m4a/mp3/wav etc.
and plays only the audio track. Volume can be changed while a sound is
playing: the node talks to mpv over its JSON IPC socket.

Subscribes
    play_sound    std_msgs/Bool     True = play from the start (restarts if
                                    already playing), False = stop
    sound_volume  std_msgs/Float32  0-100 (percent); applies immediately to
                                    the current sound and to later ones

Requires mpv on the Pi:
    sudo apt install mpv

The default sound_file is the repo's audio/ folder, as a path relative to
ros2_ws/ (where the nodes are run from, like src/outputs in run_logging.py).
Run from elsewhere, or point at another file, with e.g.
    --ros-args -p sound_file:=/home/<user>/sounds/chicken.mp4

Test:
    ros2 run ballrobot_pkg sound_player_node.py
    ros2 topic pub --once /play_sound std_msgs/msg/Bool "{data: true}"
    ros2 topic pub --once /sound_volume std_msgs/msg/Float32 "{data: 40.0}"
    ros2 topic pub --once /play_sound std_msgs/msg/Bool "{data: false}"

No sound? List mpv's outputs with `mpv --audio-device=help` and pass the
jack's entry as audio_device. On Pi OS the jack is the "Headphones" card
(`aplay -l`); HDMI is "vc4hdmi".

Parameters
    sound_file    str    "../audio/pig_sound_effect.mp3"  relative to cwd
    volume        float  80.0   initial volume, 0-100
    loop          bool   False  repeat until stopped
    audio_device  str    "alsa/plughw:CARD=Headphones"  mpv --audio-device
                                value; "" = system default output
"""

import json
import os
import socket
import subprocess
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

IPC_SOCKET = '/tmp/sound_player_mpv.sock'


class SoundPlayerNode(Node):

    def __init__(self):
        super().__init__('sound_player_node')

        self.declare_parameter('sound_file', '../audio/pig_sound_effect.mp3')
        self.declare_parameter('volume', 80.0)
        self.declare_parameter('loop', False)
        self.declare_parameter('audio_device', 'alsa/plughw:CARD=Headphones')

        self.sound_file = os.path.abspath(
            os.path.expanduser(str(self.get_parameter('sound_file').value)))
        self.volume = self.clamp_volume(float(self.get_parameter('volume').value))
        self.loop = bool(self.get_parameter('loop').value)
        self.audio_device = str(self.get_parameter('audio_device').value).strip()

        self.lock = threading.Lock()
        self.proc = None

        self.create_subscription(Bool, 'play_sound', self.play_callback, 10)
        self.create_subscription(Float32, 'sound_volume', self.volume_callback, 10)

        if not os.path.isfile(self.sound_file):
            self.get_logger().warn(
                f'Sound file {self.sound_file} not found -- run from ros2_ws/ '
                'or set the sound_file parameter')
        self.get_logger().info(
            f'Ready: {self.sound_file} at volume {self.volume:.0f}% '
            f'(device: {self.audio_device or "default"})')

    @staticmethod
    def clamp_volume(v):
        return max(0.0, min(100.0, v))

    # ---- callbacks -----------------------------------------------------

    def play_callback(self, msg: Bool):
        if msg.data:
            self.play()
        else:
            self.stop()

    def volume_callback(self, msg: Float32):
        self.volume = self.clamp_volume(msg.data)
        if self.is_playing():
            self.send_ipc(['set_property', 'volume', self.volume])
        self.get_logger().info(f'Volume set to {self.volume:.0f}%')

    # ---- mpv control ---------------------------------------------------

    def is_playing(self):
        with self.lock:
            return self.proc is not None and self.proc.poll() is None

    def play(self):
        if not os.path.isfile(self.sound_file):
            self.get_logger().error(f'Cannot play: {self.sound_file} does not exist')
            return
        self.stop()

        cmd = [
            'mpv', '--no-video', '--no-terminal', '--really-quiet',
            f'--volume={self.volume}',
            f'--input-ipc-server={IPC_SOCKET}',
        ]
        if self.audio_device:
            cmd.append(f'--audio-device={self.audio_device}')
        if self.loop:
            cmd.append('--loop-file=inf')
        cmd.append(self.sound_file)

        try:
            proc = subprocess.Popen(
                cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
        except FileNotFoundError:
            self.get_logger().error('mpv not installed -- run: sudo apt install mpv')
            return
        with self.lock:
            self.proc = proc
        threading.Thread(target=self.watch, args=(proc,), daemon=True).start()
        self.get_logger().info(f'Playing {os.path.basename(self.sound_file)}')

    def watch(self, proc):
        # Surface mpv errors (bad device, unreadable file) in the ROS log.
        _, err = proc.communicate()
        if proc.returncode not in (0, None, -15) and err.strip():
            self.get_logger().error(f'mpv exited {proc.returncode}: {err.strip()}')

    def stop(self):
        with self.lock:
            proc, self.proc = self.proc, None
        if proc is not None and proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                proc.kill()
            self.get_logger().info('Stopped')

    def send_ipc(self, command):
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as s:
                s.settimeout(0.5)
                s.connect(IPC_SOCKET)
                s.sendall((json.dumps({'command': command}) + '\n').encode())
        except OSError as e:
            # mpv may not have opened the socket yet if it was just started;
            # the new volume is still used for the next play.
            self.get_logger().warn(f'Could not reach mpv to change volume: {e}')

    def destroy_node(self):
        self.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SoundPlayerNode()
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
