import sys
import numpy as np 
sys.path.append("..")
import PathTracking.utils as utils
from PathTracking.controller import Controller

class ControllerPIDBicycle(Controller):
    def __init__(self, model, 
                 # TODO 4.1.3: Tune PID Gains
                 kp=0.0, 
                 ki=0.0, 
                 kd=0.0):
        self.path = None
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.acc_ep = 0
        self.last_ep = 0
        self.dt = model.dt
        self.current_idx = 0
    
    def set_path(self, path):
        super().set_path(path)
        self.acc_ep = 0
        self.last_ep = 0
        self.current_idx = 0
    
    def feedback(self, info):
        # Check Path
        if self.path is None:
            print("No path !!")
            return None, None
        
        # Extract State
        x, y, yaw = info["x"], info["y"], info["yaw"]

        # Check if reached end of track
        if self.current_idx >= len(self.path) - 5:
            return 0.0, self.path[-1]

        # Search Nearest Target Locally
        min_idx, min_dist = utils.search_nearest_local(self.path, (x,y), self.current_idx, lookahead=50)
        self.current_idx = min_idx
        
        # TODO 4.1.3: PID Control for Bicycle Kinematic Model
        next_delta = 0
        # [end] TODO 4.1.3
        return next_delta
