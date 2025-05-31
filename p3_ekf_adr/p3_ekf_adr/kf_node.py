from matplotlib.typing import JoinStyleType
from rclpy.node import Node

from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

import numpy as np

from .common_utils.sensor_utils import odom_to_pose2D, get_normalized_pose2D, rotate_pose2D, get_yaw_from_quaternion, Odom2DDriftSimulator
from .common_utils.helper_utils import normalize_angle
from .common_utils.visualization import Visualizer

from .filters.ekf import ExtendedKalmanFilter


class KalmanFilterBaseNode(Node):
    def __init__(self, kf):
        super().__init__('kalman_filter_node')

        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',  # 'wheel_odom',  # Ground Truth
            self.odom_callback,
            10)

        #        self.wheel_encod_subscription = self.create_subscription(
        #            JointState,
        #            'joint_states', # which contains the position and velocity of the wheels in rad and rad/s
        #            self.wheel_encod_callback,
        #            10)

        self.imu_subscriber = self.create_subscription(
            Imu,
            '/imu',  # '/imu',
            self.imu_callback,
            10)

        self.kf = kf
        
        # Initialize the visualizer to see the results
        self.visualizer = Visualizer()

        # Initialize the drift simulator, to corrupt the perfect simulation odometry
        self.odom_simulator = Odom2DDriftSimulator()    

        # Create a ROS 2 timer for the visualizer updates
        self.visualizer_timer = self.create_timer(0.1, self.update_visualizer)

        self.mu = None
        self.Sigma = None
        self.u = None
        self.z = None
        self.prev_time = None  # previous prediction time, used to compute the delta_t

        # Variables to normalize the pose (always start at the origin)
        self.initial_pose = None
        self.normalized_pose = (0.0, 0.0, 0.0) 

        # IMU data
        self.initial_imu_theta = None
        self.normalized_imu_theta = 0.0
        self.imu_w = 0.0
        self.imu_a_x = 0.0
        self.imu_a_y = 0.0

        self.prev_normalized_pose = (0.0, 0.0, 0.0)
        self.prev_pose_set = None 

        self.initial_gt_pose = None
        self.normalized_gt_pose = (0.0, 0.0, 0.0)

        print("EKF ready!")

    def update_visualizer(self):
        # Call the visualizer update asynchronously
        if self.mu is not None and self.Sigma is not None:
            if self.z is not None:
                self.visualizer.update(self.normalized_gt_pose, self.mu, self.Sigma, self.z, step="update")
            else:
                self.visualizer.update(self.normalized_gt_pose, self.mu, self.Sigma, step="predict")


    def imu_callback(self, msg):

        self.imu_msg = msg

        # Extract the linear acceleration data from the IMU message
        self.imu_a_x = msg.linear_acceleration.x
        self.imu_a_y = msg.linear_acceleration.y

        # Extract the angular velocity data from the IMU message
        self.imu_w = msg.angular_velocity.z

        # Compute the yaw from the IMU quaternion
        imu_theta = get_yaw_from_quaternion(msg.orientation)

        # Calculate the fake theta
        if not self.initial_imu_theta:
            self.initial_imu_theta = imu_theta
        else:
            # Calculate the difference in yaw
            delta_theta = imu_theta - self.initial_imu_theta

            # Unwrap the delta_yaw to avoid issues with angles wrapping around at 2*pi radians
            self.normalized_imu_theta = normalize_angle(delta_theta)

    def odom_callback(self, msg):
        # Set the initial pose
        if not self.initial_gt_pose:

            initial_pose = odom_to_pose2D(msg)  

            self.initial_gt_pose = initial_pose

        # Get and normalize the pose
        current_pose = odom_to_pose2D(msg)
        self.normalized_gt_pose = np.array(get_normalized_pose2D(self.initial_gt_pose, current_pose))

        # To use the odometry as fake sensors
        # Set the initial pose
        if not self.initial_pose:
            self.initial_pose = odom_to_pose2D(msg)  

            #self.initial_pose = rotate_pose2D(initial_pose, -90)
        # Extract real velocities
        v_x = msg.twist.twist.linear.x
        omega_z = msg.twist.twist.angular.z

        # Add noise to velocities (controls)
        v_x_noisy = v_x + np.random.normal(0, 0.02)
        omega_z_noisy = omega_z + np.random.normal(0, 0.01)
        #self.u = np.array([v_x_noisy, omega_z_noisy])
        self.control = msg.twist.twist
        self.set_control()

        # Predict step
        curr_time = self.get_clock().now().nanoseconds
        if self.prev_time:
            dt = (curr_time - self.prev_time) / 1e9
        else:
            dt = 0.01
        self.mu, self.Sigma = self.kf.predict(self.u, dt)
        self.prev_time = curr_time

        print(f"[PREDICT] u = {self.u}, dt = {dt}, mu = {self.mu}")

        # Extract real position
        pose = odom_to_pose2D(msg)

        # Add noise to position (observations)
        # We include some errors in the measurements
        curr_time_secs = self.get_clock().now().nanoseconds / 1e9
        pose_noisy = self.odom_simulator.add_drift(pose, curr_time_secs) 
        #pose_noisy = np.array(pose) + np.random.normal(0, [0.02, 0.02, 0.01])

        #self.z = pose_noisy

        self.normalized_pose = np.array(get_normalized_pose2D(self.initial_pose, pose_noisy))
        self.set_observation()

        # Update step
        self.mu, self.Sigma = self.kf.update(self.z, dt)
        print(f"[UPDATE] z: {self.z}")
        print(f"[UPDATE] mu after update: {self.mu}")
        print(f"[ODOM] Raw Pose: {pose}")
        print(f"[ODOM] Noisy Pose: {pose_noisy}")
        print(f"[ODOM] Normalized Pose: {self.normalized_pose}")
        print(f"[ODOM] Normalized GT Pose: {self.normalized_gt_pose}")
        self.prev_normalized_pose = self.normalized_pose


    def set_control(self):
        raise NotImplementedError("This function has to be implemented by a child class")

    def set_observation(self):
        raise NotImplementedError("This function has to be implemented by a child class")


class KalmanFilterNode(KalmanFilterBaseNode):
    def set_control(self):
        self.u = np.asarray([self.control.linear.x + 0.1, self.control.angular.z])

        print(f"[SET_CONTROL] u set to: {self.u}")
        # Get the control inputs for odometry model
        ''' if self.prev_pose_set:
            self.u = np.asarray([self.prev_normalized_pose, self.normalized_pose])
        else:
            self.prev_normalized_pose = self.normalized_pose
            self.prev_pose_set = True
            return
        '''

    def set_observation(self):
        self.z = self.normalized_pose


class KalmanFilterFusionNode(KalmanFilterNode):

    def set_observation(self):
        self.z = np.array([[self.normalized_pose[0]], [self.normalized_pose[1]], [self.normalized_pose[2]], [self.normalized_imu_theta], [self.imu_w], [self.imu_a_x], [self.imu_a_y]])

