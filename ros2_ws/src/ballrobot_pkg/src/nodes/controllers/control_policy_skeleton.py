#!/usr/bin/env python3

# Copyright (C) 2026, The University of Melbourne, Department of Electrical and Electronic Engineering (EEE)
#
# This file is part of ASClinic-System.
#    
# See the root of the repository for license details.
#
# ----------------------------------------------------------------------------
#     _    ____   ____ _ _       _          ____            _                 
#    / \  / ___| / ___| (_)____ (_) ___    / ___| _   _ ___| |_ ___ ________  
#   / _ \ \___ \| |   | | |  _ \| |/ __|___\___ \| | | / __| __/ _ \  _   _ \ 
#  / ___ \ ___) | |___| | | | | | | (_|_____|__) | |_| \__ \ ||  __/ | | | | |
# /_/   \_\____/ \____|_|_|_| |_|_|\___|   |____/ \__, |___/\__\___|_| |_| |_|
#                                                 |___/                       
#
# DESCRIPTION:
# Python node as a skeleton for implementing a control policy.
# This node is provided to exemplify bringing together the
# encoder counts and ArUco detection measurements into one node,
# and using those measurements to set a motor duty cycle action.
#
# Motor driver : roboclaw_for_motors.py (RoboClaw via basicmicro library)
# Encoder source: roboclaw_for_motors.py (RoboClaw quadrature decoding)
#   The RoboClaw publishes signed delta counts per timer interval.
#   Positive counts = forward motion (after direction-multiplier correction
#   applied in roboclaw_for_motors.py)
# ----------------------------------------------------------------------------

# Import the ROS2-Python package
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Import numpy
import numpy as np

# Import package-specific messages
from asclinic_pkg.msg import LeftRightFloat32
from asclinic_pkg.msg import LeftRightInt32
from asclinic_pkg.msg import FiducialMarkerArray

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


class ControlPolicySkeleton(Node):

    def __init__(self):
        super().__init__('control_policy_skeleton')

        # DECLARE AND GET PARAMETERS:
        # > For the verbosity level of displaying info
        self.declare_parameter('control_policy_verbosity', DEFAULT_CONTROL_POLICY_VERBOSITY)
        self.control_policy_verbosity = self.get_parameter('control_policy_verbosity').get_parameter_value().integer_value

        # > For the wheel base dimension of the robot
        self.declare_parameter('robot_wheel_base', DEFAULT_ROBOT_WHEEL_BASE)
        self.robot_wheel_base = self.get_parameter('robot_wheel_base').get_parameter_value().double_value

        # > For the wheel radius dimension of the robot
        self.declare_parameter('robot_wheel_radius', DEFAULT_ROBOT_WHEEL_RADIUS)
        self.robot_wheel_radius = self.get_parameter('robot_wheel_radius').get_parameter_value().double_value

        # > For the number of encoder counts per revolution of a wheel
        self.declare_parameter('encoder_counts_per_wheel_revolution', DEFAULT_ENCODER_COUNTS_PER_WHEEL_REVOLUTION)
        self.encoder_counts_per_wheel_revolution = self.get_parameter('encoder_counts_per_wheel_revolution').get_parameter_value().integer_value

        # Define QoS profiles
        qos_profile_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        qos_profile_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # PUBLISHERS AND SUBSCRIBERS:
        # > Initialise a publisher for the motor duty cycle requests
        self.motor_duty_cycle_request_publisher = self.create_publisher(
            LeftRightFloat32, 
            'set_motor_duty_cycle', 
            qos_profile_reliable
        )

        # > Initialise a subscriber for the current motor duty cycle
        self.current_motor_duty_cycle_subscriber = self.create_subscription(
            LeftRightFloat32,
            'current_motor_duty_cycle',
            self.current_motor_duty_cycle_subscriber_callback,
            qos_profile_reliable
        )

        # > Initialise a subscriber for the encoder counts sensor measurements
        self.encoder_counts_subscriber = self.create_subscription(
            LeftRightInt32,
            'encoder_counts',
            self.encoder_counts_subscriber_callback,
            qos_profile_sensor
        )

        # > Initialise a subscriber for the ArUco marker vision-detection measurements
        self.aruco_detections_subscriber = self.create_subscription(
            FiducialMarkerArray,
            'aruco_detections',
            self.aruco_detections_subscriber_callback,
            qos_profile_sensor
        )

        # CLASS VARIABLES:
        # > For world frame state estimate [xW, yW, phiW2R]
        self.xW_estimate = 0.0
        self.yW_estimate = 0.0
        self.phiW2R_estimate = 0.0

        # > Compute the rotation of the wheel per encoder count
        self.wheel_rotation_per_count = (2.0 * np.pi / self.encoder_counts_per_wheel_revolution) * self.robot_wheel_radius

        # > Initialize a sequence number for the motor duty cycle request actions
        self.motor_action_sequence_number = 1

        # Display the status
        if self.control_policy_verbosity >= 1:
            self.get_logger().info("[CONTROL POLICY SKELETON] Node initialisation complete.")



    # Function that computes and publishes motor duty-cycle request actions
    # based on the most recent state estimate
    def execute_control_policy(self):
        # NOTE: this skeleton does NOT include any meaningful
        #       control policy implementation (for you to edit)

        # Prepare a message to send the motor duty-cycle request action
        msg_for_motors = LeftRightFloat32()
        msg_for_motors.left    = 0.0
        msg_for_motors.right   = 0.0
        msg_for_motors.seq_num = self.motor_action_sequence_number

        # Publish the message
        self.motor_duty_cycle_request_publisher.publish(msg_for_motors)

        # Increment the sequence number
        self.motor_action_sequence_number = self.motor_action_sequence_number + 1

    # Current motor duty cycle subscriber callback
    def current_motor_duty_cycle_subscriber_callback(self, msg):
        # Display the data received
        if self.control_policy_verbosity >= 2:
            self.get_logger().info(
                f"[CONTROL POLICY SKELETON] Received current motor duty cycle "
                f"(left,right,seq_num) = ( {msg.left:6.1f} , {msg.right:6.1f} , {msg.seq_num} )"
            )

    # Encoder counts subscriber callback
    # NOTE: this skeleton lets the receiving of the encoder counts
    #       messages determine the frequency at which the control
    #       policy runs.
    # NOTE: msg.left and msg.right are signed delta counts per timer interval.
    #       Positive = forward, negative = backward. 
    def encoder_counts_subscriber_callback(self, msg):
        # Display the data received
        if self.control_policy_verbosity >= 2:
            self.get_logger().info(
                f"[CONTROL POLICY SKELETON] Received encoder counts "
                f"(left,right,seq_num) = ( {msg.left:6} , {msg.right:6} , {msg.seq_num} )"
            )

        # Compute the angular change of the wheels
        delta_theta_left  = msg.left  * self.wheel_rotation_per_count
        delta_theta_right = msg.right * self.wheel_rotation_per_count

        # Compute the change in displacement (delta s) and change in rotation (delta phi) resulting from the wheel rotations
        delta_s   = (delta_theta_right + delta_theta_left) * 0.5 * self.robot_wheel_radius
        delta_phi = (delta_theta_right - delta_theta_left) * 0.5 * self.robot_wheel_radius / self.robot_wheel_base

        # Get the current state estimate in local variables to avoid errors related to the sequence of updates
        xW_current     = self.xW_estimate
        yW_current     = self.yW_estimate
        phiW2R_current = self.phiW2R_estimate

        # Compute the sine and cosine of the "halfway" angle for the wheel odometry mean and covariance formulas
        sin_phi_plus_half = np.sin(phiW2R_current + 0.5 * delta_phi)
        cos_phi_plus_half = np.cos(phiW2R_current + 0.5 * delta_phi)

        # Update the mean of the state estimate
        self.xW_estimate     = xW_current + delta_s * cos_phi_plus_half
        self.yW_estimate     = yW_current + delta_s * sin_phi_plus_half
        self.phiW2R_estimate = phiW2R_current + delta_phi

        # Update the covariance of the state estimate
        # > NOTE: this skeleton does not include co-variance

        # Call the function to execute the control policy
        self.execute_control_policy()

    # ArUco Detections subscriber callback
    # Details about the ArUco detection data:
    # > The properties "rvec" and "tvec" respectively
    #   describe the rotation and translation of the
    #   marker frame relative to the camera frame, i.e.:
    #   tvec - is a vector of length 3 expressing the
    #          (x,y,z)-coordinates of the marker's center
    #          in the coordinate frame of the camera.
    #   rvec - is a vector of length 3 expressing the
    #          rotation of the marker's frame relative to
    #          the frame of the camera. This vector is an
    #          "axis angle" representation of the rotation.
    # > Hence, a vector expressed in maker-frame coordinates
    #   can be transformed to camera-frame coordinates as:
    #   - Rmat = cv2.Rodrigues(rvec)
    #   - [x,y,z]_{in camera frame} = tvec + Rmat * [x,y,z]_{in marker frame}
    # > Note: the camera frame convention is:
    #   - z-axis points along the optical axis, i.e., straight out of the lens
    #   - x-axis points to the right when looking out of the lens along the z-axis
    #   - y-axis points to the down  when looking out of the lens along the z-axis
    def aruco_detections_subscriber_callback(self, msg):
        # Display the data received
        if self.control_policy_verbosity >= 2:
            self.get_logger().info(
                f"[CONTROL POLICY SKELETON] Received aruco detections data for {msg.num_markers} markers."
            )

        # Iterate through the array of detected markers
        for i_marker in range(msg.num_markers):
            # Get the ID, tvec, and rvec for this marker
            this_id   = msg.markers[i_marker].id
            this_tvec = msg.markers[i_marker].tvec
            this_rvec = msg.markers[i_marker].rvec

            # Convert to a World frame estimate
            # NOTE: this skeleton does not include this transformation

        # Fuse with the current state estimate
        # NOTE: this skeleton does not include sensor fusion


def main(args=None):
    # Initialize the ROS2 Python client library
    rclpy.init(args=args)

    # Create an instance of the node
    control_policy_node = ControlPolicySkeleton()

    # Spin the node to handle callbacks
    rclpy.spin(control_policy_node)

    # Clean up
    control_policy_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
