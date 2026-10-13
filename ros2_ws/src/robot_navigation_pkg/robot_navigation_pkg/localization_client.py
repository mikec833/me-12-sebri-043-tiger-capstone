"""
Client side of ballrobot_localization_pkg for the navigation nodes.

Keeps the latest /odometry/filtered pose, the time of the latest fused UWB
fix, and the distance travelled from wheel odometry, and publishes heading
corrections to the EKF set_pose topic.
"""

import math
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


class LocalizationClient:

    def __init__(self, node, set_pose_topic='/set_pose', on_fix=None):
        self._node = node
        self.x = self.y = self.yaw = None
        self.var_x = self.var_y = math.inf
        self.v = self.omega = 0.0
        self.pose_time = None      # monotonic receipt time of /odometry/filtered
        self.fix_time = None       # monotonic receipt time of the last UWB fix
        self.travelled = 0.0       # m, integrated |v_x| from wheel odometry
        self._wheel_stamp = None
        self._on_fix = on_fix
        node.create_subscription(Odometry, '/odometry/filtered', self._odom_cb, 10)
        node.create_subscription(
            PoseWithCovarianceStamped, '/localization/uwb_pose', self._uwb_cb, 10)
        node.create_subscription(Odometry, '/localization/wheel_odom', self._wheel_cb, 20)
        self._set_pose = node.create_publisher(PoseWithCovarianceStamped, set_pose_topic, 1)

    @property
    def has_pose(self):
        return self.pose_time is not None

    def pose_age(self):
        return math.inf if self.pose_time is None else time.monotonic() - self.pose_time

    def fix_age(self):
        return math.inf if self.fix_time is None else time.monotonic() - self.fix_time

    def position_std(self):
        return math.sqrt(max(self.var_x, self.var_y))

    def _odom_cb(self, msg):
        p = msg.pose.pose
        q = p.orientation
        self.x, self.y = p.position.x, p.position.y
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.var_x, self.var_y = msg.pose.covariance[0], msg.pose.covariance[7]
        self.v, self.omega = msg.twist.twist.linear.x, msg.twist.twist.angular.z
        self.pose_time = time.monotonic()

    def _uwb_cb(self, msg):
        self.fix_time = time.monotonic()
        if self._on_fix:
            self._on_fix(msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _wheel_cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._wheel_stamp is not None and 0.0 < t - self._wheel_stamp < 0.5:
            self.travelled += abs(msg.twist.twist.linear.x) * (t - self._wheel_stamp)
        self._wheel_stamp = t

    def set_heading(self, yaw, yaw_variance=0.05):
        """Reset the EKF to the current position with heading yaw (brief 4.1)."""
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.pose.position.x = float(self.x)
        msg.pose.pose.position.y = float(self.y)
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        cov = [0.0] * 36
        cov[0] = self.var_x if math.isfinite(self.var_x) else 1.0
        cov[7] = self.var_y if math.isfinite(self.var_y) else 1.0
        cov[14] = cov[21] = cov[28] = 1e-3
        cov[35] = yaw_variance
        msg.pose.covariance = cov
        self._set_pose.publish(msg)
