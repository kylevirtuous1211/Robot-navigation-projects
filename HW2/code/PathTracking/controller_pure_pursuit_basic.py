import sys
import numpy as np 
sys.path.append("..")
import PathTracking.utils as utils
from PathTracking.controller import Controller

class ControllerPurePursuitBasic(Controller):
    def __init__(self, model, 
                 # Optional TODO: Tune Pure Pursuit Gain
                 kp=0.1, Lfc=5.0):
        self.path = None
        self.kp = kp
        self.Lfc = Lfc
        self.current_idx = 0

    def set_path(self, path):
        super().set_path(path)
        self.current_idx = 0

    def feedback(self, info):
        # Check Path
        if self.path is None:
            print("No path !!")
            return None, None
        
        # Extract State 
        x, y, yaw, v = info["x"], info["y"], info["yaw"], info["v"]

        # Check if reached end of track
        if self.current_idx >= len(self.path) - 3:
            return 0.0

        min_idx, min_dist = utils.search_nearest_local(self.path, (x,y), self.current_idx, lookahead=50)
        self.current_idx = min_idx
        
        Ld = self.kp*v + self.Lfc

        # Optional TODO: Pure Pursuit Control for Basic Kinematic Model
        # Search for look-ahead point with wraparound support
        for i in range(len(self.path)):
            search_idx = (self.current_idx + i) % len(self.path)
            dist = np.hypot(self.path[search_idx][0] - x, self.path[search_idx][1] - y)
            if dist >= Ld:
                self.current_idx = search_idx
                break
        else:
            # Fallback for non-loop tracks or extremely large look-ahead
            self.current_idx = len(self.path) - 1
            
        target = self.path[self.current_idx]
        
        alpha = np.arctan2(target[1] - y, target[0] - x) - np.deg2rad(yaw)
        next_w = np.rad2deg(v * 2 * np.sin(alpha) / Ld)
        # [end] Optional TODO: Pure Pursuit Control for Basic Kinematic Model
        
        return next_w
