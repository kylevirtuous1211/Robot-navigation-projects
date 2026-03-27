import sys
import numpy as np 
sys.path.append("..")
import PathTracking.utils as utils
from PathTracking.controller import Controller

class ControllerStanleyBicycle(Controller):
    def __init__(self, model, 
                 # TODO 4.3.1: Tune Stanley Gain
                 kp=2.0):
        self.path = None
        self.kp = kp
        self.l = model.l
        self.current_idx = 0

    def set_path(self, path):
        super().set_path(path)
        self.current_idx = 0

    # State: [x, y, yaw, delta, v]
    def feedback(self, info):
        # Check Path
        if self.path is None:
            print("No path !!")
            return None, None
        
        # Extract State 
        x, y, yaw, delta, v = info["x"], info["y"], info["yaw"], info["delta"], info["v"]

        # Check if reached end of track
        if self.current_idx >= len(self.path) - 5:
            return 0.0

        # Search Front Wheel Target Locally
        front_x = x + self.l*np.cos(np.deg2rad(yaw))
        front_y = y + self.l*np.sin(np.deg2rad(yaw))
        vf = v / np.cos(np.deg2rad(delta)) if np.cos(np.deg2rad(delta)) != 0 else v
        
        min_idx, min_dist = utils.search_nearest_local(self.path, (front_x,front_y), self.current_idx, lookahead=50)
        self.current_idx = min_idx
        target = self.path[min_idx]

        # TODO 4.3.1: Stanley Control for Bicycle Kinematic Model
        # Heading error: theta_e = theta_path - theta
        # Normalize to [-180, 180]
        theta_e = target[2] - yaw
        theta_e = (theta_e + 180) % 360 - 180

        # Cross-track error (signed distance)
        # Vector from car to path: (target[0]-front_x, target[1]-front_y)
        # Vector perpendicular to car's yaw (to the left): (-sin(yaw), cos(yaw))
        e = (target[1] - front_y) * np.cos(np.deg2rad(yaw)) - (target[0] - front_x) * np.sin(np.deg2rad(yaw))

        # Stanley formula: delta = theta_e + arctan(ke/vf)
        # Avoid division by zero by adding a small constant or using arctan2
        next_delta = theta_e + np.rad2deg(np.arctan2(self.kp * e, vf + 0.1))
        # [end] TODO 4.3.1
    
        return next_delta
