"""
Client side of the teensy_interface_node drive interface (firmware v4.7).

Tracks /drive/armed, /drive/owner and /drive/status, and calls the ~/arm
and ~/disarm services asynchronously so callers never block an executor.
"""

from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class DriveClient:

    def __init__(self, node, interface_node='/teensy_interface_node', on_status=None):
        self._node = node
        self.armed = False
        self.owner = 'UNKNOWN'
        self.last_fault = None
        self._on_status = on_status
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(Bool, '/drive/armed', self._armed_cb, latched)
        node.create_subscription(String, '/drive/owner', self._owner_cb, latched)
        node.create_subscription(String, '/drive/status', self._status_cb, 50)
        self._arm = node.create_client(Trigger, f'{interface_node}/arm')
        self._disarm = node.create_client(Trigger, f'{interface_node}/disarm')

    @property
    def driving_ok(self) -> bool:
        """True while this Pi owns an armed robot."""
        return self.armed and self.owner == 'SERIAL'

    def _armed_cb(self, msg):
        self.armed = msg.data

    def _owner_cb(self, msg):
        self.owner = msg.data

    def _status_cb(self, msg):
        if msg.data.startswith('FAULT'):
            self.last_fault = msg.data
        elif msg.data == 'STATE,FAULT_RESET':
            self.last_fault = None
        if self._on_status:
            self._on_status(msg.data)

    def arm(self, done):
        """Request arming; done(success: bool, message: str) is called later."""
        self._call(self._arm, done)

    def disarm(self, done=None):
        self._call(self._disarm, done)

    def _call(self, client, done):
        if not client.service_is_ready():
            if done:
                done(False, f'{client.srv_name} not available')
            return
        future = client.call_async(Trigger.Request())

        def finished(f):
            if done is None:
                return
            try:
                result = f.result()
                done(result.success, result.message)
            except Exception as exc:  # service died mid call
                done(False, str(exc))
        future.add_done_callback(finished)
