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
            self.Q[0,0] = 1
            self.Q[1,1] = 1 
            self.R[0,0] = 1
        elif control_state == 'steering_angular_velocity':
            self.Q = np.eye(3)
            self.R = np.eye(1)
            # TODO 4.4.4: Tune LQR Gains
            self.Q[0,0] = 1
            self.Q[1,1] = 1
            self.Q[2,2] = 1
            self.R[0,0] = 1
        self.pe = 0
        self.pth_e = 0
        self.pdelta = 0
        self.dt = model.dt
        self.l = model.l
        self.control_state = control_state

    def set_path(self, path):
        super().set_path(path)
        self.pe = 0
        self.pth_e = 0
        self.pdelta = 0

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
        
        # Search Nesrest Target
        min_idx, min_dist = utils.search_nearest(self.path, (x,y))
        target = self.path[min_idx]
        target[2] = utils.angle_norm(target[2])
        
        if self.control_state == 'steering_angle':
            # TODO 4.4.1: LQR Control for Bicycle Kinematic Model with steering angle as control input
            next_delta = 0
            # [end] TODO 4.4.1
        elif self.control_state == 'steering_angular_velocity':
            # TODO 4.4.4: LQR Control for Bicycle Kinematic Model with steering angular velocity as control input
            next_delta = 0
            # [end] TODO 4.4.4
        
        return next_delta
