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
# PID controller line tracking
#
# Motor driver : roboclaw_for_motors.py (RoboClaw via basicmicro library)
# Encoder source: roboclaw_for_motors.py (RoboClaw quadrature decoding)
#   The RoboClaw publishes signed delta counts per timer interval.
#   Positive counts = forward motion (after direction-multiplier correction
#   applied in roboclaw_for_motors.py)
# ----------------------------------------------------------------------------

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from pathlib import Path
import csv, os
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
import math
import cv2

# Import package-specific messages
from asclinic_pkg.msg import LeftRightFloat32
from asclinic_pkg.msg import LeftRightInt32
from asclinic_pkg.msg import FiducialMarkerArray
from geometry_msgs.msg import Pose2D

# ----------------------------------------------------------------------------
# Controller Parameters
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

# velocity command (m/s)
VELOCITY = 0.2

# threshold for determining whether reached goal (m)
THRESHOLD = 0.1

DEFAULT_ROTATION_THRESHOLD = np.deg2rad(2)

# proportional gain for motor control
DEFAULT_MOTOR_KP = 0.0
# integral gain for motor control
DEFAULT_MOTOR_KI = 0.0
# derivative gain for motor control
DEFAULT_MOTOR_KD = 0.0
DEFAULT_KP_ROTATION = 0.0
DEFAULT_KP_LINE_DEVIATION = 0.0

# odometry measurement covariance constants
K_LEFT = 0.19
K_RIGHT = 0.13
# ----------------------------------------------------------------------------
# CV Parameters
MARKER_COORDINATES = {
    1:  {"x": 0.00,  "y": 1.00,  "z": 0.30, "phi_deg": 0.0},
    2:  {"x": 0.00,  "y": 3.60,  "z": 0.30, "phi_deg": 0.0},
    3:  {"x": 0.00,  "y": 6.60,  "z": 0.30, "phi_deg": 0.0},
    4:  {"x": 0.00,  "y": 9.60,  "z": 0.30, "phi_deg": 0.0},

    5:  {"x": 15.00, "y": 1.00,  "z": 0.30, "phi_deg": 180.0},
    6:  {"x": 15.00, "y": 3.60,  "z": 0.30, "phi_deg": 180.0},
    7:  {"x": 15.00, "y": 6.60,  "z": 0.30, "phi_deg": 180.0},
    8:  {"x": 15.00, "y": 9.60,  "z": 0.30, "phi_deg": 180.0},

    9:  {"x": 5.00,  "y": 0.00,  "z": 0.30, "phi_deg": 90.0},
    10: {"x": 8.00,  "y": 0.00,  "z": 0.30, "phi_deg": 90.0},
    11: {"x": 11.00, "y": 0.00,  "z": 0.30, "phi_deg": 90.0},
    12: {"x": 14.00, "y": 0.00,  "z": 0.30, "phi_deg": 90.0},

    13: {"x": 5.00,  "y": 10.80, "z": 0.30, "phi_deg": -90.0},
    14: {"x": 8.00,  "y": 10.80, "z": 0.30, "phi_deg": -90.0},
    15: {"x": 11.00, "y": 10.80, "z": 0.30, "phi_deg": -90.0},
    16: {"x": 14.00, "y": 10.80, "z": 0.30, "phi_deg": -90.0},

    18: {"x": 8.20,  "y": 5.20,  "z": 0.30, "phi_deg": 0.0},
    19: {"x": 7.80,  "y": 5.20,  "z": 0.30, "phi_deg": 180.0},
    20: {"x": 8.00,  "y": 5.40,  "z": 0.30, "phi_deg": 90.0},
    22: {"x": 8.00,  "y": 5.00,  "z": 0.30, "phi_deg": -90.0},
    
    # Optional Arucos added
    
    23: {"x": 0.80,  "y": 10.00, "z": 0.30, "phi_deg": -90.0},
    24: {"x": 4.60,  "y": 4.00,  "z": 0.30, "phi_deg": 90.0},
    25: {"x": 8.80,  "y": 6.80,  "z": 0.30, "phi_deg": 180.0},
}
# ----------------------------------------------------------------------------
# Path Planning Parameters

# Z-shaped path around the arena (15 m × 10.8 m, origin at bottom-right corner).
# Segment 1: right aisle  — Y = 0.3 m,  X: 0.5 → 13.5  (clear of all tables and P5)
# Segment 2: top crossing — X = 13.5 m, Y: 0.3 → 10.5  (passes ~1 m from P6)
# Segment 3: left aisle   — Y = 10.5 m, X: 13.5 → 0.5  (passes ~0.5 m from P1)
WAYPOINTS = np.array([
    [ 0,  0.5],   # 1: start  (near origin)
    [ 1,  0.8],   # 2: end of right aisle
    [ 0.8, 9.4],   # 3: end of top crossing
    [ 1,    9.4],   # 4: end of left aisle (goal)
    [ 4.6,      10], # 5
    [ 4.6, 5], # 6
    [5, 5], # 7
    [5, 7], # 8
    [8.4, 7], # 9
    [10, 7], # 10
    [9, 7], # 11
    [11, 7], # 12
    [11, 1], # 13
    [11.8, 1], # 14
    [11.8, 3.6], # 15
    [14.6, 3.6], # 16
    [14.6, 7], # 17
    [0.8, 7], # 18
    [1, 0.8], # 19
])

# WAYPOINTS = np.array([[0, 0],
#                       [1, 0],
#                       [0, 0]])

# ----------------------------------------------------------------------------
# Sensor Fusion Parameters
# Process noise Q: uncertainty added per odometry step.
# Larger → trust odometry less → KF relies more on CV corrections.
# Symptom requiring increase: estimate drifts away from truth between CV updates.
Q = np.diag([0.005, 0.005, 0.002])  # [x_var, y_var, phi_var] per step (m², m², rad²)

# Measurement noise R: assumed uncertainty in the CV pose estimate.
# Larger → trust CV less → KF weights odometry more.
# Symptom requiring increase: estimate jumps/is jittery each time a marker is seen.
CV_MEASUREMENT_COVARIANCE = 1e-9 * np.eye(3)     # [x_var, y_var, phi_var] for CV (m², m², rad²)


class controller(Node):

    def __init__(self):
        super().__init__('controller')

        # Controller Parameters
        # > For the verbosity level of displaying info
        self.declare_parameter('control_policy_verbosity', DEFAULT_CONTROL_POLICY_VERBOSITY)
        self.control_policy_verbosity = self.get_parameter('control_policy_verbosity').get_parameter_value().integer_value

        # > For the wheel base dimension of the robot
        self.declare_parameter('robot_wheel_base', DEFAULT_ROBOT_WHEEL_BASE)
        self.robot_wheel_base = self.get_parameter('robot_wheel_base').get_parameter_value().double_value

        # > For the wheel radius dimension of the robot
        self.declare_parameter('robot_wheel_radius', DEFAULT_ROBOT_WHEEL_RADIUS)
        self.robot_wheel_radius = self.get_parameter('robot_wheel_radius').get_parameter_value().double_value
        
        self.declare_parameter('delta_t_for_publishing_encoder_counts', 0.1)
        self.m_encoder_delta_t = self.get_parameter('delta_t_for_publishing_encoder_counts').value

        # > For the number of encoder counts per revolution of a wheel
        self.declare_parameter('encoder_counts_per_wheel_revolution', DEFAULT_ENCODER_COUNTS_PER_WHEEL_REVOLUTION)
        self.encoder_counts_per_wheel_revolution = self.get_parameter('encoder_counts_per_wheel_revolution').get_parameter_value().integer_value

        self.declare_parameter('motor_kp', DEFAULT_MOTOR_KP)
        self.motor_kp = self.get_parameter('motor_kp').get_parameter_value().double_value
        
        self.declare_parameter('motor_kd', DEFAULT_MOTOR_KD)
        self.motor_kd = self.get_parameter('motor_kd').get_parameter_value().double_value

        self.declare_parameter('motor_ki', DEFAULT_MOTOR_KI)
        self.motor_ki = self.get_parameter('motor_ki').get_parameter_value().double_value

        self.declare_parameter('kp_rotation', DEFAULT_KP_ROTATION)
        self.kp_rotation = self.get_parameter('kp_rotation').get_parameter_value().double_value

        self.declare_parameter('kp_line_deviation', DEFAULT_KP_LINE_DEVIATION)
        self.kp_line_deviation = self.get_parameter('kp_line_deviation').get_parameter_value().double_value

        self.declare_parameter('rotation_threshold', DEFAULT_ROTATION_THRESHOLD)
        self.rotation_threshold = self.get_parameter('rotation_threshold').get_parameter_value().double_value

        # CV Parameters
        # Sensor Fusion Parameters

        # Define QoS profiles
        qos_profile_reliable = QoSProfile(
            reliability = QoSReliabilityPolicy.RELIABLE,
            history = QoSHistoryPolicy.KEEP_LAST,
            depth = 1
        )
        
        qos_profile_sensor = QoSProfile(
            reliability = QoSReliabilityPolicy.BEST_EFFORT,
            history = QoSHistoryPolicy.KEEP_LAST,
            depth = 10
        )

        # PUBLISHERS AND SUBSCRIBERS:
        # > Initialise a publisher for the motor duty cycle requests
        self.motor_duty_cycle_request_publisher = self.create_publisher(
            LeftRightFloat32, 
            'asc/set_motor_duty_cycle', 
            qos_profile_reliable
        )

        # > Initialise a subscriber for the current motor duty cycle
        self.current_motor_duty_cycle_subscriber = self.create_subscription(
            LeftRightFloat32,
            'asc/current_motor_duty_cycle',
            self.current_motor_duty_cycle_subscriber_callback,
            qos_profile_reliable
        )

        # > Initialise a subscriber for the encoder counts sensor measurements
        self.encoder_counts_subscriber = self.create_subscription(
            LeftRightInt32,
            'asc/encoder_counts',
            self.encoder_counts_subscriber_callback,
            qos_profile_sensor
        )

        # > Initialise a subscriber for the ArUco marker vision-detection measurements
        # self.aruco_detections_subscriber = self.create_subscription(
        #     FiducialMarkerArray,
        #     'aruco_detections',
        #     self.aruco_detections_subscriber_callback,
        #     qos_profile_sensor
        # )

        self.cv_state_estimate_subscriber = self.create_subscription(
            Pose2D,
            "/robot_pose_from_aruco",
            self.cv_state_estimate_subscriber_callback,
            qos_profile_reliable
        )
        
        self.motion_control_period = 0.5
        self.motion_control_timer = self.create_timer(self.motion_control_period, self.motion_control_callback)

        # CLASS VARIABLES:
        # mode 1 rotation; mode 0 line tracking
        self.state = "STOPPED"
        self.reached_goal = False
        self.controller_mode = 1
        self.idx_waypoint = 1
        self.prev_waypoint = WAYPOINTS[self.idx_waypoint - 1, :]
        self.current_waypoint = WAYPOINTS[self.idx_waypoint, :]
        # > For world frame state estimate [xW, yW, phiW2R]
        self.x_world_estimate = 0.5
        self.y_world_estimate = 1
        self.phi_world_estimate = np.pi/2

        self.pose_covariance = 1e-5 * np.eye(3)
        self.cv_state_estimate = np.zeros(3)
        # motor control variables
        self.dt = self.m_encoder_delta_t
        # theta_dot_left_ref: reference angular velocity left wheel (rad/s)
        self.theta_dot_left_ref = 0.0
        # theta_dot_right_ref: reference angular velocity left wheel (rad/s)
        self.theta_dot_right_ref = 0.0
        self.theta_dot_left_estimate = 0.0
        self.theta_dot_right_estimate = 0.0
        # theta_dot_left_integral_error: integral of left wheel angular velocity error signal over time
        self.theta_dot_left_integral_error = 0.0
        # theta_dot_right_integral_error: integral of right wheel angular velocity error signal over time
        self.theta_dot_right_integral_error = 0.0
        # > Initialize a sequence number for the motor duty cycle request actions
        self.motor_action_sequence_number = 1
        self.prev_theta_dot_left_error = 0.0
        self.prev_theta_dot_right_error = 0.0
        self.prev_duty_cycle_left = 0.0
        self.prev_duty_cycle_right = 0.0
        self.max_duty_cycle = 50.0
        self.encoder_counts_to_radians = 2.0 * np.pi / self.encoder_counts_per_wheel_revolution

        # outer controller variables
        # prev_idx: index of previous waypoint
        # self.prev_idx = 0
        # line_deviation_integral_error: integral of line deviation error signal over time
        self.line_deviation_integral_error = 0.0
        self.prev_y_error = 0.0

        # logging and plotting        
        # id = datetime.today().strftime('%Y-%m-%d %H:%M:%S')
        self.motor_control_log_dir = "/home/asc/controller_validation/motor_control"
        self.motion_control_log_dir = "/home/asc/controller_validation/motion_control"
        self.state_estimates_log_dir = "/home/asc/controller_validation/state_estimates"

        if not os.path.exists(self.motor_control_log_dir):
            Path(self.motor_control_log_dir).mkdir(parents = True, exist_ok = True)
        if not os.path.exists(self.motion_control_log_dir):
            Path(self.motion_control_log_dir).mkdir(parents = True, exist_ok = True)
        if not os.path.exists(self.state_estimates_log_dir):
            Path(self.state_estimates_log_dir).mkdir(parents = True, exist_ok = True)

        self.motor_control_csv_filename = f'{self.motor_control_log_dir}/motor_response_Kp_{self.motor_kp}_Ki_{self.motor_ki}_Kd_{self.motor_kd}_Ts_{self.m_encoder_delta_t:.2f}.csv'
        with open(self.motor_control_csv_filename, mode='w', newline='') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(['left wheel velocity', 'right wheel velocity', 'left wheel velocity reference', 'right wheel velocity reference'])

        self.odometry_state_estimates_csv_filename = f'{self.state_estimates_log_dir}/odometry_state_estimates.csv'
        with open(self.odometry_state_estimates_csv_filename, mode='w', newline='') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(['x_world_estimate', 'y_world_estimate', 'phi_world_estimate'])

        self.cv_state_estimates_csv_filename = f'{self.state_estimates_log_dir}/cv_state_estimates.csv'
        with open(self.cv_state_estimates_csv_filename, mode='w', newline='') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(['x_world_estimate', 'y_world_estimate', 'phi_world_estimate'])

        self.fused_state_estimates_csv_filename = f'{self.state_estimates_log_dir}/fused_state_estimates.csv'
        with open(self.fused_state_estimates_csv_filename, mode='w', newline='') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(['x_world_estimate', 'y_world_estimate', 'phi_world_estimate'])


        self.motor_control_plot_filename = f'{self.motor_control_log_dir}/motor_response_Kp_{self.motor_kp}_KI_{self.motor_ki}_Kd_{self.motor_kd}_Ts_{self.m_encoder_delta_t:.2f}_outer_Ts_{self.motion_control_period:.2f}.jpg'

        self.motor_control_fig, self.motor_control_axs = plt.subplots(4, sharex = True)
        for ax in self.motor_control_axs:
            ax.grid('True')
        self.motor_control_axs[0].set_title('Left Motor')
        self.motor_control_axs[1].set_title('Right Motor')
        self.motor_control_axs[2].set_title('Left Motor Duty Cycle')
        self.motor_control_axs[3].set_title('Right Motor Duty Cycle')
        self.motor_control_fig.supxlabel('Time (s)')
        self.motor_control_fig.supylabel('Angular Velocity (rad/s)')

        self.state_estimates_plot_filename = f'{self.state_estimates_log_dir}/state_estimates.jpg'

        self.state_estimates_fig, self.state_estimates_axs = plt.subplots(3, sharex = True)
        for ax in self.state_estimates_axs:
            ax.grid('True')
        self.state_estimates_axs[0].set_title('x_world_estimate')
        self.state_estimates_axs[1].set_title('y_world_estimate')
        self.state_estimates_axs[2].set_title('phi_world_estimate')
        self.state_estimates_fig.supxlabel('Time (s)')
        self.state_estimates_fig.suptitle('State Estimates')



        self.time_history = [0.0]
        self.theta_dot_left_history = [0.0]
        self.theta_dot_right_history = [0.0]
        self.theta_dot_left_ref_history = [0.0]
        self.theta_dot_right_ref_history = [0.0]
        self.left_duty_cycle_history = [0.0]
        self.right_duty_cycle_history = [0.0]

        self.x_world_estimate_history = [0.0]
        self.y_world_estimate_history = [0.0]
        self.phi_world_estimate_history = [0.0]

        self.fused_estimates_time_history = [0.0]
        self.cv_x_world_estimate_history = [0.0]
        self.cv_y_world_estimate_history = [0.0]
        self.cv_phi_world_estimate_history = [0.0]
        self.fused_x_world_estimate_history = [0.0]
        self.fused_y_world_estimate_history = [0.0]
        self.fused_phi_world_estimate_history = [0.0]

        # Display the status
        if self.control_policy_verbosity >= 1:
            self.get_logger().info("[CONTROL POLICY] Node initialisation complete.")
        
        self.start_time = self.get_clock().now().nanoseconds

    def drive_steer_to_wheel_velocities(self, v, omega):
        # Convert drive and steer commands to wheel velocities
        # v: drive command
        # omega: steer command
        # theta_dot_left: angular velocity of left wheel (rad/s)
        theta_dot_left = (-self.robot_wheel_base / (2 * self.robot_wheel_radius)) *  omega + (1 / self.robot_wheel_radius) * v
        # theta_dot_right: angular velocity of right wheel (rad/s)
        theta_dot_right = (self.robot_wheel_base / (2 * self.robot_wheel_radius)) *  omega + (1 / self.robot_wheel_radius) * v
        return theta_dot_left, theta_dot_right


    def inertial_frame_to_line_frame(self, robot_pose_world):
        # angle line_frame_x relative to world_frame_x
        angle_world_to_line = np.arctan2(self.current_waypoint[1] - self.prev_waypoint[1], self.current_waypoint[0] - self.prev_waypoint[0])
        cos_phi = np.cos(angle_world_to_line)
        sin_phi = np.sin(angle_world_to_line)
        rotation_matrix = np.array([[cos_phi, sin_phi], [-sin_phi, cos_phi]])
        robot_pose_line_frame = np.dot(rotation_matrix, (robot_pose_world[0:2] - self.prev_waypoint[0:2]))
        return robot_pose_line_frame

    # def convert_waypoints_body_frame(self, pose_inertial, waypoints):
    #     # Convert waypoints in inertial/world frame coordinates to body frame coordinates
    #     robot_x = pose_inertial[0]
    #     robot_y = pose_inertial[1]
    #     robot_phi = pose_inertial[2]
    #     rotation_matrix = np.array([[np.cos(robot_phi), np.sin(robot_phi)], [-np.sin(robot_phi), np.cos(robot_phi)]])
    #     waypoints_body_frame = waypoints - np.array([robot_x, robot_y])
    #     waypoints_body_frame = np.dot(rotation_matrix, waypoints_body_frame.T).T
    #     return waypoints_body_frame

    def motion_control_callback(self):
        pose_world_estimate = np.array([self.x_world_estimate, self.y_world_estimate, self.phi_world_estimate])
        angle_world_to_line = np.arctan2(self.current_waypoint[1] - self.prev_waypoint[1], self.current_waypoint[0] - self.prev_waypoint[0])
        not_aligned = np.abs(pose_world_estimate[-1] - angle_world_to_line) > self.rotation_threshold
        not_reached_current_target = np.linalg.norm(pose_world_estimate[0:2] - self.current_waypoint[0:2]) > THRESHOLD
        # self.get_logger().info(f'State = {self.state}')
        # self.get_logger().info(f'at goal = {self.reached_goal}')
        # self.get_logger().info(f'not_aligned = {not_aligned}')
        # self.get_logger().info(f'not_reached_current_target = {not_reached_current_target}')
        if (self.state == "STOPPED") and (not self.reached_goal) and not_aligned and not_reached_current_target:
            self.state = "ROTATION"
            self.theta_dot_left_ref = 0.0
            self.theta_dot_right_ref = 0.0
            return
        if (self.state == "ROTATION") and (not_aligned):
            phi_error = angle_world_to_line - pose_world_estimate[-1]
            omega = self.kp_rotation * phi_error
            self.theta_dot_left_ref, self.theta_dot_right_ref = self.drive_steer_to_wheel_velocities(0, omega)
            return
        if (self.state == "ROTATION") and (not not_aligned) and (not self.reached_goal):
            self.state = "LINE_TRACKING"
            self.theta_dot_left_ref = 0.0
            self.theta_dot_right_ref = 0.0
            return
        if (self.state == "LINE_TRACKING") and (not self.reached_goal) and not_reached_current_target:
            # straight line tracking mode
            pose_line_estimate = self.inertial_frame_to_line_frame(pose_world_estimate)
            error = -pose_line_estimate[1]
            # self.get_logger().info(f'y_error = {y_error}')
            # self.line_deviation_integral_error += y_error * self.motion_control_period
            # omega = self.kp_line_deviation * y_error + self.kd_line_deviation * (y_error - self.prev_y_error) / self.motion_control_period + self.Ki_line_deviation * self.line_deviation_integral_error
            omega = self.kp_line_deviation * error
            # update previous error
            self.prev_y_error = error            
            self.theta_dot_left_ref, self.theta_dot_right_ref = self.drive_steer_to_wheel_velocities(VELOCITY, omega)
            return
        if (self.state == "LINE_TRACKING") and (not not_reached_current_target):
            self.state = "STOPPED"
            self.theta_dot_left_ref = 0.0
            self.theta_dot_right_ref = 0.0
            if (self.idx_waypoint == (WAYPOINTS.shape[0] - 1)):
                self.reached_goal = True
            else:
                self.prev_waypoint = self.current_waypoint
                self.idx_waypoint += 1
                self.current_waypoint = WAYPOINTS[self.idx_waypoint, :]
            return
        if (self.state == "STOPPED") and (self.reached_goal):
            self.theta_dot_left_ref = 0.0
            self.theta_dot_right_ref = 0.0
            return
        if (self.state == "STOPPED") and (not not_aligned) and (not self.reached_goal) and (not_reached_current_target):
            self.state = "LINE_TRACKING"
            self.theta_dot_left_ref = 0.0
            self.theta_dot_right_ref = 0.0
            return

    # def motion_control_callback(self):
    #     #  test rotation
    #     pose_world_estimate = np.array([self.x_world_estimate, self.y_world_estimate, self.phi_world_estimate])
    #     angle_world_to_line = np.arctan2(self.current_waypoint[1] - self.prev_waypoint[1], self.current_waypoint[0] - self.prev_waypoint[0])
    #     self.get_logger().info(f'angle_world_to_line = {np.rad2deg(angle_world_to_line)} degrees')
    #     if (self.controller_mode == 1) and (np.abs(pose_world_estimate[-1] - angle_world_to_line) > self.rotation_threshold):
    #         # rotation mode
    #         phi_error = angle_world_to_line - pose_world_estimate[-1]
    #         omega = self.kp_rotation * phi_error
    #         self.theta_dot_left_ref, self.theta_dot_right_ref = self.drive_steer_to_wheel_velocities(0, omega)
    #     else:
    #         self.get_logger().info(f'Reached desired orientation')
    #         self.theta_dot_left_ref = 0.0
    #         self.theta_dot_right_ref = 0.0
 
    # def motion_control_callback(self):
    # #   test straight line
    #     pose_world_estimate = np.array([self.x_world_estimate, self.y_world_estimate, self.phi_world_estimate])
    #     if (np.linalg.norm(pose_world_estimate[0:2] - WAYPOINTS[1, :]) > THRESHOLD):
    #         y_error = -self.y_world_estimate
    #         # self.get_logger().info(f'y_error = {y_error}')
    #         self.line_deviation_integral_error += y_error * self.motion_control_period
    #         omega = self.kp_line_deviation * y_error
    #         self.prev_y_error = y_error            
    #         self.theta_dot_left_ref, self.theta_dot_right_ref = self.drive_steer_to_wheel_velocities(VELOCITY, omega)
    #     else:
    #         self.get_logger().info('Reached target, stopping robot')
    #         self.theta_dot_left_ref = 0.0
    #         self.theta_dot_right_ref = 0.0

    # Current motor duty cycle subscriber callback
    def current_motor_duty_cycle_subscriber_callback(self, msg):
        # Display the data received
        if self.control_policy_verbosity >= 2:
            self.get_logger().info(
                f"[CONTROL POLICY] Received current motor duty cycle "
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
                f"[CONTROL POLICY] Received encoder counts "
                f"(left,right,seq_num) = ( {msg.left:6} , {msg.right:6} , {msg.seq_num} )"
            )

        # delta_theta_left: rotation left wheel (radians)
        delta_theta_left  = msg.left  * self.encoder_counts_to_radians
        # delta_theta_right: rotation right wheel (radians)
        delta_theta_right = msg.right * self.encoder_counts_to_radians

        # odometry state estimation
        delta_s   = (delta_theta_right + delta_theta_left) * 0.5 * self.robot_wheel_radius
        delta_phi = (delta_theta_right - delta_theta_left) * self.robot_wheel_radius / self.robot_wheel_base

        # Get the current state estimate in local variables to avoid errors related to the sequence of updates
        x_world_estimate_current = self.x_world_estimate
        y_world_estimate_current = self.y_world_estimate
        phi_world_estimate_current = self.phi_world_estimate

        cos_phi_plus_half = np.cos(phi_world_estimate_current + 0.5 * delta_phi)
        sin_phi_plus_half = np.sin(phi_world_estimate_current + 0.5 * delta_phi)
        
        # Update the mean of the state estimate
        self.x_world_estimate = x_world_estimate_current + delta_s * cos_phi_plus_half
        self.y_world_estimate = y_world_estimate_current + delta_s * sin_phi_plus_half
        self.phi_world_estimate = self.wrap_angle(phi_world_estimate_current + delta_phi)

        # Update the covariance of the state estimate
        odometry_measurement_covariance = np.diag([K_LEFT*np.abs(delta_theta_left), K_RIGHT*np.abs(delta_theta_right)])
        grad_p_odometry_update = np.array([[1, 0, -delta_s*sin_phi_plus_half],
                                           [0, 1, delta_s*cos_phi_plus_half],
                                           [0, 0, 1]])
        grad_theta_11 = 0.5*self.robot_wheel_radius*cos_phi_plus_half + (0.5*self.robot_wheel_radius/self.robot_wheel_base)*delta_s*sin_phi_plus_half
        grad_theta_12 = 0.5*self.robot_wheel_radius*cos_phi_plus_half - (0.5*self.robot_wheel_radius/self.robot_wheel_base)*delta_s*sin_phi_plus_half
        grad_theta_21 = 0.5*self.robot_wheel_radius*sin_phi_plus_half - (0.5*self.robot_wheel_radius/self.robot_wheel_base)*delta_s*cos_phi_plus_half
        grad_theta_22 = 0.5*self.robot_wheel_radius*sin_phi_plus_half + (0.5*self.robot_wheel_radius/self.robot_wheel_base)*delta_s*cos_phi_plus_half
        grad_theta_31 = -self.robot_wheel_radius/self.robot_wheel_base
        grad_theta_32 = self.robot_wheel_radius/self.robot_wheel_base
        grad_theta_odometry_update = np.array([[grad_theta_11, grad_theta_12],
                                               [grad_theta_21, grad_theta_22],
                                               [grad_theta_31, grad_theta_32]])
        self.pose_covariance = grad_p_odometry_update @ self.pose_covariance @ grad_p_odometry_update.T + grad_theta_odometry_update @ odometry_measurement_covariance @ grad_theta_odometry_update.T

        # motor control
        self.theta_dot_left_estimate = delta_theta_left / self.dt
        self.theta_dot_right_estimate = delta_theta_right / self.dt
        theta_dot_left_error = self.theta_dot_left_ref - self.theta_dot_left_estimate
        theta_dot_right_error = self.theta_dot_right_ref - self.theta_dot_right_estimate
        
        self.theta_dot_left_integral_error += theta_dot_left_error * self.dt
        self.theta_dot_right_integral_error += theta_dot_right_error * self.dt
        # left_derivative_error = ALPHA * (theta_dot_left_error - self.prev_theta_dot_left_error) / self.dt + (1 - ALPHA) * self.prev_derivative_left_error
        # right_derivative_error = ALPHA * (theta_dot_right_error - self.prev_theta_dot_right_error) / self.dt + (1 - ALPHA) * self.prev_derivative_right_error
        duty_cycle_left_motor = self.motor_kp * theta_dot_left_error + self.motor_kd * (theta_dot_left_error - self.prev_theta_dot_left_error) / self.dt + self.motor_ki * self.theta_dot_left_integral_error
        duty_cycle_right_motor = self.motor_kp * theta_dot_right_error + self.motor_kd * (theta_dot_right_error - self.prev_theta_dot_right_error) / self.dt + self.motor_ki * self.theta_dot_right_integral_error
        duty_cycle_left_motor = max(min(duty_cycle_left_motor, self.max_duty_cycle), -self.max_duty_cycle)
        duty_cycle_right_motor = max(min(duty_cycle_right_motor, self.max_duty_cycle), -self.max_duty_cycle)
        self.prev_theta_dot_left_error = theta_dot_left_error
        self.prev_theta_dot_right_error = theta_dot_right_error

        motor_duty_cycle_command = LeftRightFloat32()
        motor_duty_cycle_command.left    = duty_cycle_left_motor
        motor_duty_cycle_command.right   = duty_cycle_right_motor
        motor_duty_cycle_command.seq_num = self.motor_action_sequence_number

        # Publish the message
        self.motor_duty_cycle_request_publisher.publish(motor_duty_cycle_command)

        # Increment the sequence number
        self.motor_action_sequence_number += 1

        # logging
        # self.time_history.append(self.time_history[-1] + self.dt)
        self.time_history.append((self.get_clock().now().nanoseconds - self.start_time) * 1e-9)
        self.theta_dot_left_history.append(self.theta_dot_left_estimate)
        self.theta_dot_right_history.append(self.theta_dot_left_estimate)
        self.theta_dot_left_ref_history.append(self.theta_dot_left_ref)
        self.theta_dot_right_ref_history.append(self.theta_dot_right_ref)
        self.left_duty_cycle_history.append(duty_cycle_left_motor)
        self.right_duty_cycle_history.append(duty_cycle_right_motor)

        self.x_world_estimate_history.append(self.x_world_estimate)
        self.y_world_estimate_history.append(self.y_world_estimate)
        self.phi_world_estimate_history.append(self.phi_world_estimate)


        # self.axs[0].plot(self.time_history, self.theta_dot_left_history)
        # self.axs[0].plot(self.time_history, self.theta_dot_left_ref_history)
        # self.axs[1].plot(self.time_history, self.theta_dot_right_history)
        # self.axs[1].plot(self.time_history, self.theta_dot_right_ref_history)
        # self.fig.savefig(self.motor_control_plot_filename)

        # self.odometry_axs[0].plot(self.time_history, self.x_world_estimate_history)
        # self.odometry_axs[1].plot(self.time_history, self.y_world_estimate_history)
        # self.odometry_axs[2].plot(self.time_history, self.phi_world_estimate_history)
        # self.odometry_fig.savefig(self.odometry_plot_filename)

        # with open(self.motor_control_csv_filename, mode = 'a', newline = '') as csv_file:
        #     writer = csv.writer(csv_file)
        #     writer.writerow([self.theta_dot_left_estimate, self.theta_dot_right_estimate, self.theta_dot_left_ref, self.theta_dot_right_ref])
        # with open(self.odometry_csv_filename, mode = 'a', newline = '') as csv_file:
        #     writer = csv.writer(csv_file)
        #     writer.writerow([self.x_world_estimate, self.y_world_estimate, self.phi_world_estimate])

# ------------------------------------------------------------------------------------------------------------------------------------------------
    # CV
    # 
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
    def cv_state_estimate_subscriber_callback(self, msg):
        self.get_logger().info('Received CV state estimates.')
        self.fused_estimates_time_history.append((self.get_clock().now().nanoseconds - self.start_time) * 1e-9)
        self.cv_x_world_estimate_history.append(msg.x)
        self.cv_y_world_estimate_history.append(msg.y)
        self.cv_phi_world_estimate_history.append(msg.theta)

        self.cv_state_estimate = np.array([msg.x, msg.y, msg.theta])
        
        # self.fuse_estimates()

    # def aruco_detections_subscriber_callback(self, msg):
    #     # Display the data received
    #     if self.control_policy_verbosity >= 2:
    #         self.get_logger().info(
    #             f"[CONTROL POLICY] Received aruco detections data for {msg.num_markers} markers."
    #         )

    #     if msg.num_markers == 0:
    #         return

    #     valid_results = []

    #     for marker in msg.markers:
    #         marker_id = marker.id

    #         if marker_id not in MARKER_COORDINATES:
    #             continue

    #         tx = float(marker.tvec[0])
    #         ty = float(marker.tvec[1])
    #         tz = float(marker.tvec[2])

    #         # 1) Estimate camera position from this marker
    #         x_cam, y_cam, phi_deg, phi = self.estimate_camera_position(marker_id, tx, tz)

    #         # 2) Estimate heading and frontality from full rvec
    #         theta_robot, yaw_marker_in_camera, frontal_angle, marker_normal_cam = self.estimate_robot_heading(marker_id, marker.rvec)

    #         # 3) Convert camera position to robot origin
    #         x_robot, y_robot = self.camera_to_robot_planar(
    #             x_cam,
    #             y_cam,
    #             theta_robot
    #         )

    #         # 4) Full 3D distance from full tvec
    #         distance_3d = self.compute_distance_3d(tx, ty, tz)

    #         valid_results.append({
    #             "marker_id": marker_id,
    #             "tx": tx,
    #             "ty": ty,
    #             "tz": tz,
    #             "distance_3d": distance_3d,
    #             "x_cam": x_cam,
    #             "y_cam": y_cam,
    #             "x_robot": x_robot,
    #             "y_robot": y_robot,
    #             "theta_robot": theta_robot,
    #             "yaw_marker_in_camera": yaw_marker_in_camera,
    #             "frontal_angle": frontal_angle,
    #             "marker_normal_cam": marker_normal_cam,
    #             "phi_deg": phi_deg,
    #         })

    #     if len(valid_results) == 0:
    #         self.get_logger().warn(
    #             "[ROBOT MAP LOCALIZER] No detected marker matches the hardcoded map layout."
    #         )
    #         return

    #     # Position marker = smallest full 3D tvec norm
    #     position_marker = min(
    #         valid_results,
    #         key=lambda item: item["distance_3d"]
    #     )

    #     # Heading marker = smallest full 3D frontal angle from rvec
    #     heading_marker = min(
    #         valid_results,
    #         key=lambda item: item["frontal_angle"]
    #     )

    #     # Final fused pose
    #     x_robot_final = position_marker["x_robot"]
    #     y_robot_final = position_marker["y_robot"]
    #     theta_robot_final = heading_marker["theta_robot"]

    #     pose_msg = Pose2D()
    #     pose_msg.x = x_robot_final
    #     pose_msg.y = y_robot_final
    #     pose_msg.theta = theta_robot_final

    #     # Save final fused pose to memory and CSV
    #     self.record_pose_history(
    #         x_robot_final,
    #         y_robot_final,
    #         theta_robot_final,
    #         position_marker["marker_id"],
    #         heading_marker["marker_id"]
    #     )

    #     debug_lines = [
    #         f"[ROBOT MAP LOCALIZER] visible_mapped_markers={len(valid_results)}",
    #         f"[ROBOT MAP LOCALIZER] fused_xy=({x_robot_final:.3f}, {y_robot_final:.3f}) "
    #         f"from marker_id={position_marker['marker_id']}",
    #         f"[ROBOT MAP LOCALIZER] fused_theta={math.degrees(theta_robot_final):.2f} deg "
    #         f"from marker_id={heading_marker['marker_id']}",
    #         f"[ROBOT MAP LOCALIZER] csv_saved_to={self.csv_file_path}",
    #         f"[ROBOT MAP LOCALIZER] graph_saved_to={self.graph_file_path}"
    #     ]

    #     for item in valid_results:
    #         role_list = []

    #         if item["marker_id"] == position_marker["marker_id"]:
    #             role_list.append("POSITION")

    #         if item["marker_id"] == heading_marker["marker_id"]:
    #             role_list.append("HEADING")

    #         role_str = ",".join(role_list) if role_list else "IGNORED"

    #         nx, ny, nz = item["marker_normal_cam"]

    #         debug_lines.append(
    #             f"[ROBOT MAP LOCALIZER] "
    #             f"marker_id={item['marker_id']} | "
    #             f"role={role_str} | "
    #             f"marker_phi_world={item['phi_deg']:.1f} deg | "
    #             f"tvec=[{item['tx']:.3f}, {item['ty']:.3f}, {item['tz']:.3f}] | "
    #             f"distance_3d={item['distance_3d']:.3f} m | "
    #             f"camera_xy=({item['x_cam']:.3f}, {item['y_cam']:.3f}) | "
    #             f"robot_xy=({item['x_robot']:.3f}, {item['y_robot']:.3f}) | "
    #             f"theta={math.degrees(item['theta_robot']):.2f} deg | "
    #             f"yaw={math.degrees(item['yaw_marker_in_camera']):.2f} deg | "
    #             f"frontal_angle={math.degrees(item['frontal_angle']):.2f} deg | "
    #             f"marker_normal_cam=({nx:.3f}, {ny:.3f}, {nz:.3f})"
    #         )

    #     self.get_logger().info("\n" + "\n".join(debug_lines))

    #     # Fuse with the current state estimate
    #     # NOTE: this skeleton does not include sensor fusion
    # def estimate_camera_position(self, marker_id, tx, tz):
    #     marker_world = self.marker_layout[marker_id]
    #     xm = marker_world["x"]
    #     ym = marker_world["y"]
    #     phi_deg = marker_world["phi_deg"]
    #     phi = math.radians(phi_deg)

    #     x_cam = xm + tz * math.cos(phi) + tx * math.sin(phi)
    #     y_cam = ym + tz * math.sin(phi) - tx * math.cos(phi)

    #     return x_cam, y_cam, phi_deg, phi
    
    # def estimate_robot_heading(self, marker_id, rvec):
    #     marker_world = self.marker_layout[marker_id]
    #     phi_deg = marker_world["phi_deg"]
    #     phi = math.radians(phi_deg)

    #     rvec_np = np.array(rvec, dtype=np.float64).reshape(3, 1)

    #     # Rotation from marker frame to camera frame
    #     R_cm, _ = cv2.Rodrigues(rvec_np)

    #     # Marker z-axis expressed in camera frame = third column of R_cm
    #     nx = R_cm[0, 2]
    #     ny = R_cm[1, 2]
    #     nz = R_cm[2, 2]

    #     # Current validated heading logic
    #     yaw_marker_in_camera = math.atan2(nx, nz)
    #     theta_robot = self.wrap_to_pi(phi + yaw_marker_in_camera)

    #     # Full 3D frontality score:
    #     # frontal view is treated as marker normal close to -camera z
    #     frontal_angle = math.acos(self.clamp(-nz, -1.0, 1.0))

    #     return theta_robot, yaw_marker_in_camera, frontal_angle, (nx, ny, nz)

    # def camera_to_robot_planar(self, x_cam, y_cam, theta_robot):
    #     dx = self.camera_offset_x_in_robot
    #     dy = self.camera_offset_y_in_robot

    #     x_robot = x_cam - dx * math.cos(theta_robot) + dy * math.sin(theta_robot)
    #     y_robot = y_cam - dx * math.sin(theta_robot) - dy * math.cos(theta_robot)

    #     return x_robot, y_robot
# ------------------------------------------------------------------------------------------------------------------------------------------------
# sensor fusion

    def fuse_estimates(self):
        current_pose_estimate = np.array([self.x_world_estimate, self.y_world_estimate, self.phi_world_estimate])
        current_pose_covariance = self.pose_covariance
        # --- Kalman gain ---------------------------------------------------
        # K = Σ_p * (Σ_p + Σ_z)^{-1}
        kalman_gain = self.pose_covariance @ np.linalg.inv(self.pose_covariance + CV_MEASUREMENT_COVARIANCE)
        # kalman_gain = self.pose_covariance @ np.linalg.inv(self.pose_covariance)

        # --- Innovation ----------------------------------------------------
        innovation    = self.cv_state_estimate - current_pose_estimate
        innovation[2] = self.wrap_angle(innovation[2])

        # --- State update --------------------------------------------------
        fused_pose = current_pose_estimate + kalman_gain @ innovation
        fused_pose[2] = self.wrap_angle(fused_pose[2])

        self.x_world_estimate = fused_pose[0]
        self.y_world_estimate = fused_pose[1]
        self.phi_world_estimate = fused_pose[2]

        # --- Covariance update ---------------------------------------------
        # Σ_{t|t} = Σ_{t|t-1} − K (Σ_{t|t-1} + Σ_z) K^T
        self.pose_covariance = current_pose_covariance - kalman_gain @ (current_pose_covariance + CV_MEASUREMENT_COVARIANCE) @ kalman_gain.T
        # self.pose_covariance = current_pose_covariance - kalman_gain @ (current_pose_covariance) @ kalman_gain.T

        # logging
        self.fused_x_world_estimate_history.append(self.x_world_estimate)
        self.fused_y_world_estimate_history.append(self.y_world_estimate)
        self.fused_phi_world_estimate_history.append(self.phi_world_estimate)


    def wrap_angle(self, a):
        """Wrap an angle (or array element) to [-pi, pi]."""
        return (a + np.pi) % (2.0 * np.pi) - np.pi

# ------------------------------------------------------------------------------------------------------------------------------------------------

    # logging and plotting
    def log_and_plot_data(self):
        theta_dot_left_estimate_history = np.array(self.theta_dot_left_history)
        theta_dot_right_estimate_history = np.array(self.theta_dot_left_history)
        theta_dot_left_ref_history = np.array(self.theta_dot_left_ref_history)
        theta_dot_right_ref_history = np.array(self.theta_dot_right_ref_history)
        left_duty_cycle_history = np.array(self.left_duty_cycle_history)
        right_duty_cycle_history = np.array(self.right_duty_cycle_history)
        motor_data = np.column_stack((theta_dot_left_estimate_history, theta_dot_right_estimate_history, theta_dot_left_ref_history, theta_dot_right_ref_history))

        x_world_estimate_history = np.array(self.x_world_estimate_history)
        y_world_estimate_history = np.array(self.y_world_estimate_history)
        phi_world_estimate_history = np.array(self.phi_world_estimate_history)
        odometry_state_estimates_data = np.column_stack((x_world_estimate_history, y_world_estimate_history, phi_world_estimate_history))

        cv_x_world_estimate_history = np.array(self.cv_x_world_estimate_history)
        cv_y_world_estimate_history = np.array(self.cv_y_world_estimate_history)
        cv_phi_world_estimate_history = np.array(self.cv_phi_world_estimate_history)
        cv_state_estimates_data = np.column_stack((cv_x_world_estimate_history, cv_y_world_estimate_history, cv_phi_world_estimate_history))

        fused_x_world_estimate_history = np.array(self.fused_x_world_estimate_history)
        fused_y_world_estimate_history = np.array(self.fused_y_world_estimate_history)
        fused_phi_world_estimate_history = np.array(self.fused_phi_world_estimate_history)
        fused_state_estimates_data = np.column_stack((fused_x_world_estimate_history, fused_y_world_estimate_history, fused_phi_world_estimate_history))

        with open(self.motor_control_csv_filename, mode = 'a', newline = '') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerows(motor_data)

        with open(self.cv_state_estimates_csv_filename, mode = 'a', newline = '') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerows(cv_state_estimates_data)

        with open(self.fused_state_estimates_csv_filename, mode = 'a', newline = '') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerows(fused_state_estimates_data)

        with open(self.odometry_state_estimates_csv_filename, mode = 'a', newline = '') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerows(odometry_state_estimates_data)

        self.motor_control_axs[0].plot(self.time_history, self.theta_dot_left_history)
        self.motor_control_axs[0].plot(self.time_history, self.theta_dot_left_ref_history)
        self.motor_control_axs[1].plot(self.time_history, self.theta_dot_right_history)
        self.motor_control_axs[1].plot(self.time_history, self.theta_dot_right_ref_history)
        self.motor_control_axs[2].plot(self.time_history, self.left_duty_cycle_history)
        self.motor_control_axs[3].plot(self.time_history, self.right_duty_cycle_history)
        self.motor_control_fig.savefig(self.motor_control_plot_filename)

        self.state_estimates_axs[0].plot(self.time_history, self.x_world_estimate_history, label = 'Odometry')
        self.state_estimates_axs[0].plot(self.fused_estimates_time_history, self.cv_x_world_estimate_history, label = 'CV')
        # self.state_estimates_axs[0].plot(self.fused_estimates_time_history, self.fused_x_world_estimate_history, label = 'Fused')
        self.state_estimates_axs[0].legend()
        
        self.state_estimates_axs[1].plot(self.time_history, self.y_world_estimate_history, label = 'Odometry')
        self.state_estimates_axs[1].plot(self.fused_estimates_time_history, self.cv_y_world_estimate_history, label = 'CV')
        # self.state_estimates_axs[1].plot(self.fused_estimates_time_history, self.fused_y_world_estimate_history, label = 'Fused')
        self.state_estimates_axs[1].legend()

        self.state_estimates_axs[2].plot(self.time_history, self.phi_world_estimate_history, label = 'Odometry')
        self.state_estimates_axs[2].plot(self.fused_estimates_time_history, self.cv_phi_world_estimate_history, label = 'CV')
        # self.state_estimates_axs[2].plot(self.fused_estimates_time_history, self.fused_phi_world_estimate_history, label = 'Fused')
        self.state_estimates_axs[2].legend()

        self.state_estimates_fig.savefig(self.state_estimates_plot_filename)


def main(args=None):
    # Initialize the ROS2 Python client library
    rclpy.init(args=args)

    # Create an instance of the node
    control_policy_node = controller()
    try:
        rclpy.spin(control_policy_node)
    finally:
        control_policy_node.log_and_plot_data()
        control_policy_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
