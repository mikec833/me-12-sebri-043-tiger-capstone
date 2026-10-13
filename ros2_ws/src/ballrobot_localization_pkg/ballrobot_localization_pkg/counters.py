"""Per adapter counters, published once a second on /diagnostics."""

from collections import OrderedDict

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


class CounterPublisher:
    """Counts named events and publishes them as one DiagnosticStatus.

    The header stamp of the DiagnosticArray is the node clock (sim time in
    offline replay). That is a report time, not sensor data, so it is the
    one place in this package where now() is used for a stamp.
    """

    def __init__(self, node, keys, period_s: float = 1.0):
        self._node = node
        self.counts = OrderedDict((k, 0) for k in keys)
        self._pub = node.create_publisher(DiagnosticArray, '/diagnostics', 10)
        node.create_timer(period_s, self.publish)

    def inc(self, key: str) -> None:
        self.counts[key] += 1

    def publish(self) -> None:
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.OK
        status.name = f'ballrobot_localization_pkg: {self._node.get_name()}'
        status.hardware_id = self._node.get_name()
        status.message = 'counters'
        status.values = [KeyValue(key=k, value=str(v)) for k, v in self.counts.items()]
        msg = DiagnosticArray()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.status = [status]
        self._pub.publish(msg)


def stamp_is_zero(stamp) -> bool:
    return stamp.sec == 0 and stamp.nanosec == 0


def stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9
