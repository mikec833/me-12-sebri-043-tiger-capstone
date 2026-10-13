"""
Minimal stand ins for rclpy and the message packages, so the navigation
nodes can be driven in a closed loop simulation without ROS.

install() puts fake modules into sys.modules. Publishers record messages,
subscriptions and services are reachable through the node's `bus`, and
timers are stepped by the test.
"""

import sys
import types
from collections import defaultdict


class Msg:
    """Attribute bag that grows nested attributes on demand."""

    def __init__(self, **kw):
        self.__dict__.update(kw)

    def __getattr__(self, name):
        if name.startswith('__'):
            raise AttributeError(name)
        value = Msg()
        setattr(self, name, value)
        return value


def msg_type(name, **defaults):
    def make(**kw):
        m = Msg(**{k: (v() if callable(v) else v) for k, v in defaults.items()})
        m.__dict__.update(kw)
        return m
    make.__name__ = name
    return make


class Bus:
    def __init__(self):
        self.subs = defaultdict(list)
        self.published = defaultdict(list)
        self.services = {}
        self.timers = []

    def deliver(self, topic, msg):
        for cb in self.subs[topic]:
            cb(msg)


class _Pub:
    def __init__(self, bus, topic):
        self.bus, self.topic = bus, topic

    def publish(self, msg):
        self.bus.published[self.topic].append(msg)
        self.bus.deliver(self.topic, msg)


class _Future:
    def __init__(self, result):
        self._result = result

    def result(self):
        return self._result

    def add_done_callback(self, cb):
        cb(self)


class _Client:
    def __init__(self, bus, name):
        self.bus, self.srv_name = bus, name

    def service_is_ready(self):
        return self.srv_name in self.bus.services

    def call_async(self, request):
        return _Future(self.bus.services[self.srv_name](request))


class _Param:
    def __init__(self, value):
        self.value = value


class _Logger:
    def __init__(self):
        self.lines = []

    def _log(self, text, **_):
        self.lines.append(text)
    info = warn = error = debug = _log


class _Clock:
    def __init__(self, t):
        self._t = t

    def now(self):
        t = self._t()
        return Msg(nanoseconds=int(t * 1e9),
                   to_msg=lambda: Msg(sec=int(t), nanosec=int((t % 1) * 1e9)))


class Node:
    bus = None            # set by install()
    clock_fn = None
    overrides = {}

    def __init__(self, name):
        self._name = name
        self._params = {}
        self.logger = _Logger()

    def get_name(self):
        return self._name

    def declare_parameter(self, name, default=None, *_):
        self._params[name] = Node.overrides.get(name, default)

    def get_parameter(self, name):
        return _Param(self._params[name])

    def create_publisher(self, _type, topic, *_a, **_k):
        return _Pub(Node.bus, topic)

    def create_subscription(self, _type, topic, cb, *_a, **_k):
        Node.bus.subs[topic].append(cb)

    def _service_name(self, name):
        return f'/{self._name}/{name[2:]}' if name.startswith('~/') else name

    def create_service(self, _type, name, cb, **_k):
        def call(req):
            return cb(req, Msg(success=False, message=''))
        Node.bus.services[self._service_name(name)] = call

    def create_client(self, _type, name, **_k):
        return _Client(Node.bus, name)

    def create_timer(self, period, cb, **_k):
        Node.bus.timers.append((period, cb))

    def get_logger(self):
        return self.logger

    def get_clock(self):
        return _Clock(Node.clock_fn)

    def destroy_node(self):
        pass


def install(clock_fn, overrides=None):
    bus = Bus()
    Node.bus, Node.clock_fn, Node.overrides = bus, clock_fn, dict(overrides or {})

    def module(name, **attrs):
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        sys.modules[name] = m
        return m

    module('rclpy', init=lambda **_: None, shutdown=lambda: None, ok=lambda: True,
           spin=lambda n: None, spin_once=lambda *a, **k: None)
    module('rclpy.node', Node=Node)
    module('rclpy.qos', QoSProfile=lambda **k: Msg(**k),
           DurabilityPolicy=Msg(TRANSIENT_LOCAL='transient_local'),
           qos_profile_sensor_data=Msg())
    module('rclpy.executors', MultiThreadedExecutor=object)
    module('geometry_msgs')
    module('geometry_msgs.msg', Twist=msg_type('Twist'), PoseStamped=msg_type('PoseStamped'),
           PoseWithCovarianceStamped=msg_type('PoseWithCovarianceStamped'),
           Vector3Stamped=msg_type('Vector3Stamped'))
    module('nav_msgs')
    module('nav_msgs.msg', Odometry=msg_type('Odometry'))
    module('std_msgs')
    module('std_msgs.msg', String=msg_type('String', data=''), Bool=msg_type('Bool', data=False))
    pkg_dir = __import__('os').path.join(__import__('os').path.dirname(__file__), '..')
    module('ament_index_python')
    module('ament_index_python.packages', get_package_share_directory=lambda name: pkg_dir)
    module('std_srvs')
    trigger = Msg(Request=lambda: Msg())
    module('std_srvs.srv', Trigger=trigger)
    for name in list(sys.modules):
        if name.startswith('robot_navigation_pkg.') and name != 'robot_navigation_pkg.nav_math':
            del sys.modules[name]
    return bus
