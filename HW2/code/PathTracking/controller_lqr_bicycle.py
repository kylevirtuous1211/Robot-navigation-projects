import sys
import numpy as np 
sys.path.append("..")
import PathTracking.utils as utils
from PathTracking.controller import Controller

class ControllerLQRBicycle(Controller):
    def __init__(self, model, Q=None, R=None, control_state='steering_angle'):
        self.path = None
        if control_state == 'steering_angle':
            self.Q = np.eye(2)
            self.R = np.eye(1)
            # TODO 4.4.1: Tune LQR Gains
            self.Q[0,0] = 1.0   # CTE penalty
            self.Q[1,1] = 10.0  # Heading error penalty (Highly sensitive in LQR)
            self.R[0,0] = 100.0 # Steering effort penalty
        elif control_state == 'steering_angular_velocity':
            self.Q = np.eye(3)
            self.R = np.eye(1)
            # TODO 4.4.4: Tune LQR Gains
            self.Q[0,0] = 1.0    # CTE penalty
            self.Q[1,1] = 10.0   # Heading error penalty
            self.Q[2,2] = 1.0    # Steering angle penalty
            self.R[0,0] = 500.0  # Steering velocity penalty
        self.pe = 0
        self.pth_e = 0
        self.pdelta = 0
        self.dt = model.dt
        self.l = model.l
        self.control_state = control_state
        #NewFeature: added current_idx to track the current index of the path
        self.current_idx = 0

    def set_path(self, path):
        super().set_path(path)
        self.pe = 0
        self.pth_e = 0
        self.pdelta = 0
        #NewFeature: added current_idx to track the current index of the path
        self.current_idx = 0

    def _solve_DARE(self, A, B, Q, R, max_iter=150, eps=0.01): # Discrete-time Algebra Riccati Equation (DARE)
        P = Q.copy()
        for i in range(max_iter):
            temp = np.linalg.inv(R + B.T @ P @ B)
            Pn = A.T @ P @ A - A.T @ P @ B @ temp @ B.T @ P @ A + Q
            if np.abs(Pn - P).max() < eps:
                break
            P = Pn
        return Pn

    # State: [x, y, yaw, delta, v]
    def feedback(self, info):
        # Check Path
        if self.path is None:
            print("No path !!")
            return None, None
        
        # Extract State 
        x, y, yaw, delta, v = info["x"], info["y"], info["yaw"], info["delta"], info["v"]
        yaw = utils.angle_norm(yaw)
        
        #NewFeature: added current_idx to track the current index of the path
        # Check if reached end of track
        if self.current_idx >= len(self.path) - 5:
            return 0.0
        #NewFeature: added current_idx to track the current index of the path               
        # Search Nearest Target Locally
        min_idx, min_dist = utils.search_nearest_local(self.path, (x,y), self.current_idx, lookahead=50)
        self.current_idx = min_idx
        target = self.path[min_idx]
        target[2] = utils.angle_norm(target[2])
        
        if self.control_state == 'steering_angle':
            # TODO 4.4.1: LQR Control for Bicycle Kinematic Model with steering angle as control input
            # State vector x = [e, th_e]^T
            # Note: Eq. dot_e = -v * th_e implies e is positive to the RIGHT of the path.
            err_e = (target[0] - x) * np.sin(np.deg2rad(yaw)) - (target[1] - y) * np.cos(np.deg2rad(yaw))
            err_th_e = np.deg2rad(utils.angle_norm(target[2] - yaw))
            X = np.array([[err_e], [err_th_e]])

            # A = [[1, -v*dt], [0, 1]]
            # B = [[0], [-v/L*dt]]
            A = np.array([[1.0, -v * self.dt], [0.0, 1.0]])
            B = np.array([[0.0], [-v / self.l * self.dt]])

            P = self._solve_DARE(A, B, self.Q, self.R)
            K = np.linalg.inv(self.R + B.T @ P @ B) @ B.T @ P @ A
            
            # Control law: u = -Kx
            u = -K @ X
            next_delta = np.rad2deg(float(u[0, 0]))
            # [end] TODO 4.4.1
        elif self.control_state == 'steering_angular_velocity':
            # TODO 4.4.4: LQR Control for Bicycle Kinematic Model with steering angular velocity as control input
            # State vector x = [e, th_e, delta]^T
            # Note: e is positive to the RIGHT to match dot_e = -v * th_e
            err_e = (target[0] - x) * np.sin(np.deg2rad(yaw)) - (target[1] - y) * np.cos(np.deg2rad(yaw))
            err_th_e = np.deg2rad(utils.angle_norm(target[2] - yaw))
            err_delta = np.deg2rad(delta)
            X = np.array([[err_e], [err_th_e], [err_delta]])

            # A = [[1, -v*dt, 0], [0, 1, -v/L*dt], [0, 0, 1]]
            # B = [[0], [0], [dt]]
            A = np.eye(3)
            A[0, 1] = -v * self.dt
            A[1, 2] = -v / self.l * self.dt
            B = np.array([[0.0], [0.0], [self.dt]])

            P = self._solve_DARE(A, B, self.Q, self.R)
            K = np.linalg.inv(self.R + B.T @ P @ B) @ B.T @ P @ A
            
            # Control law: u = dot_delta = -Kx
            dot_u = -K @ X
            dot_delta = float(dot_u[0, 0])
            
            # Update steering angle: delta_next = delta + dot_delta * dt
            # Note: navigation.py expects delta, so we provide it.
            next_delta = delta + np.rad2deg(dot_delta * self.dt)
            # [end] TODO 4.4.4
        
        return next_delta
