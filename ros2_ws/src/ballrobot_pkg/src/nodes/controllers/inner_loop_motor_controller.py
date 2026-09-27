# Import the ROS2-Python package
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Import numpy
import numpy as np

# Import package-specific messages

# DEFINE THE PARAMETERS
# > For the verbosity level of displaying info
DEFAULT_CONTROL_POLICY_VERBOSITY = 1
# Note: the levels of increasing verbosity are defined as:
# 0 : Info is not displayed. Warnings and errors are still displayed.
# 1 : Startup info is displayed.
# 2 : Info each control action and state estimate update is displayed.

# > For the wheel base of the robot [in meters]
DEFAULT_ROBOT_WHEEL_BASE = 0.22

# > For the wheel radius of the robot [in meters]
DEFAULT_ROBOT_WHEEL_RADIUS = 0.072

# > For the number of encoder counts per revolution of a wheel
# Pololu 70:1 37D motor with 64 CPR encoder, quadrature-decoded by the RoboClaw:
#   64 counts/motor-rev x 70 (gear ratio) = 4480 counts/wheel-rev
DEFAULT_ENCODER_COUNTS_PER_WHEEL_REVOLUTION = 4480

class InnerLoopMotorController(Node):
    """
    This class implements the inner-loop motor controller for the ballbot robot.
    It subscribes to the desired wheel velocities and publishes the corresponding motor commands.
    """

    def __init__(self):
        super().__init__('inner_loop_motor_controller')

        # Declare parameters
        self.declare_parameter('control_policy_verbosity', DEFAULT_CONTROL_POLICY_VERBOSITY)
        self.declare_parameter('robot_wheel_base', DEFAULT_ROBOT_WHEEL_BASE)
        self.declare_parameter('robot_wheel_radius', DEFAULT_ROBOT_WHEEL_RADIUS)
        self.declare_parameter('encoder_counts_per_wheel_revolution', DEFAULT_ENCODER_COUNTS_PER_WHEEL_REVOLUTION)

        # Get parameters
        self.control_policy_verbosity = self.get_parameter('control_policy_verbosity').get_parameter_value().integer_value
        self.robot_wheel_base = self.get_parameter('robot_wheel_base').get_parameter_value().double_value
        self.robot_wheel_radius = self.get_parameter('robot_wheel_radius').get_parameter_value().double_value
        self.encoder_counts_per_wheel_revolution = self.get_parameter('encoder_counts_per_wheel_revolution').get_parameter_value().integer_value

        # Set up QoS profile for subscriptions and publications
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # publishers and subscribers
        # > Initialise a publisher for the motor duty cycle requests

        # > Initialise a subscriber for the current motor duty cycle

        # > Initialise a subscriber for the encoder counts sensor measurements

        