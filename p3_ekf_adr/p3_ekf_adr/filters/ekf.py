
import numpy as np

import time

class ExtendedKalmanFilter:

	def __init__(self, initial_state, initial_covariance, motion_model, observation_model, **kwargs):
		# Process arguments
		proc_noise_std = kwargs.get('proc_noise_std', [0.02, 0.02, 0.01])
		obs_noise_std = kwargs.get('obs_noise_std', [0.02, 0.02, 0.01])

		self.mu = initial_state # Initial state estimate 
		self.sigma = initial_covariance # Initial uncertainty

		self.g, self.G, self.V = motion_model() # The action model to use.
		
		# Standard deviations of the process or action model noise
		self.proc_noise_std = np.array(proc_noise_std)
		# Process noise covariance (R)
		self.R = np.diag(self.proc_noise_std ** 2)

		self.h, self.H = observation_model() # The observation model to use

		# Standard deviations for the observation or sensor model noise
		self.obs_noise_std = np.array(obs_noise_std)
		# Observation noise covariance (Q)
		self.Q = np.diag(self.obs_noise_std ** 2)

		self.exec_times_pred = []
		self.exec_times_upd = []

		
	def predict(self, u, dt):
		start_time = time.time()
        # === TODO: Implement the EKF prediction step ===

        # 1. Predict the new mean using motion model g
		self.mu = self.g(self.mu , u , dt)

        # 2. Compute the Jacobian of the motion model G_t
		G = self.G(self.mu , u , dt)

        # 3. Update the covariance
		self.sigma = G @ self.sigma @ G.T + self.R
        # ===============================================

		end_time = time.time()
		execution_time = end_time - start_time
		self.exec_times_pred.append(execution_time)
		print(f"Execution time prediction: {execution_time} seconds")

		print("Average exec time pred: ", sum(self.exec_times_pred) / len(self.exec_times_pred))


		return self.mu, self.sigma

	def update(self, z, dt):
		start_time = time.time()

        # === TODO: Implement the EKF correction step ===

        # 1. Compute the Jacobian of the observation model H_t
		H = self.H(self.mu)

        # 2. Innovation covariance
		S = H @ self.sigma @ H.T + self.Q

        # 3. Kalman gain
		K = self.sigma @ H.T @ np.linalg.inv(S)

        # 4. Innovation (difference between actual and expected measurement)
		y = z - self.h(self.mu)

        # 5. Update the state estimate
		self.mu = self.mu + (K @ y).reshape(self.mu.shape[0], )

        # 6. Update the covariance
		I = np.eye(len(self.mu))
		self.sigma = (I - K @ H) @ self.sigma
        # ================================================

		end_time = time.time()
		self.exec_times_upd.append(end_time - start_time)
		return self.mu, self.sigma
