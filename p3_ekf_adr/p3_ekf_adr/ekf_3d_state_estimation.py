
import rclpy

import numpy as np

from .motion_models.velocity_motion_models import velocity_motion_model_linearized
from .observation_models.odometry_observation_models import odometry_observation_model_linearized

from .filters.ekf import ExtendedKalmanFilter
from .kf_node import KalmanFilterNode as ExtendedKalmanFilterNode


def main(args=None):
    # Initialize the Kalman Filter
    mu0 = np.zeros(3)
    Sigma0 = np.eye(3)
    # TO ADJUST
    proc_noise_std = [10.0, 10.0, 10.0] 
    obs_noise_std = [0.01, 0.01, 0.01]
    #proc_noise_std = [0.02, 0.02, 0.01] 
    #obs_noise_std = [0.02, 0.02, 0.01]

    ekf = ExtendedKalmanFilter(mu0, Sigma0, 
                               velocity_motion_model_linearized,
                               odometry_observation_model_linearized,
                               proc_noise_std = proc_noise_std,
                               obs_noise_std = obs_noise_std)

    rclpy.init(args=args)
    kalman_filter_node = ExtendedKalmanFilterNode(ekf)
    rclpy.spin(kalman_filter_node)
    kalman_filter_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
