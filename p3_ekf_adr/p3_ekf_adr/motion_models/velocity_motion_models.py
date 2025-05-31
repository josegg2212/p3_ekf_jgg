import numpy as np

def velocity_motion_model():
    def state_transition_matrix_A():
        A = np.eye(3)
        return A

    def control_input_matrix_B(mu, delta_t):
        theta = mu[2]
        B = np.array([
            [np.cos(theta) * delta_t, 0],
            [np.sin(theta) * delta_t, 0],
            [0, delta_t]
        ])
        return B

    return state_transition_matrix_A, control_input_matrix_B


def velocity_motion_model_linearized():
    print("Using velocity_motion_model_linearized (3D EKF)")

    # TODO 1: Implement the non-linear motion model function g(mu, u, dt)
    def state_transition_function_g(mu=None, u=None, delta_t=None):
        """
        Compute the predicted state using the non-linear motion model:
        mu = [x, y, theta], u = [v, omega]
        """
        x = mu[0]
        y = mu[1]
        theta = mu[2]
        v = u[0]
        omega = u[1]

        g = np.array([
            x + v * np.cos(theta) * delta_t,
            y + v * np.sin(theta) * delta_t,
            theta + omega * delta_t
        ])


        return g
  

    # TODO 2: Implement the Jacobian of g with respect to the state mu (G)
    def jacobian_of_g_wrt_state_G(mu=None, u=None, delta_t=None):
        """
        Compute the Jacobian of g w.r.t. mu:
        G = ∂g/∂mu, shape (3x3)
        """
        theta = mu[2]
        v = u[0]

        G = np.array([
            [1, 0, -v * np.sin(theta) * delta_t],
            [0, 1, v * np.cos(theta) * delta_t],
            [0, 0, 1]
        ])

        return G
   

    # TODO 3: Implement the Jacobian of g with respect to the control u (V)
    def jacobian_of_g_wrt_control_V(mu=None, u=None, delta_t=None):
        """
        Compute the Jacobian of g w.r.t. u:
        V = ∂g/∂u, shape (3x2)
        """
        theta = mu[2]
        V = np.array([
            [np.cos(theta) * delta_t, 0],
            [np.sin(theta) * delta_t, 0],
            [0, delta_t]   
        ])

        return V

    return state_transition_function_g, jacobian_of_g_wrt_state_G, jacobian_of_g_wrt_control_V


