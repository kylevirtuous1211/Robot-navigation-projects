import sys
import numpy as np 
sys.path.append("..")
import PathTracking.utils as utils
from PathTracking.controller import Controller

class VanillaLongController(Controller):
    def __init__(self):
        self.path = None
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
        x, y, yaw = info["x"], info["y"], info["yaw"]

        # Check if reached end of track
        if self.current_idx >= len(self.path) - 3:
            return 0, self.path[-1]

        # Search Nearest Target Locally
        min_idx, min_dist = utils.search_nearest_local(self.path, (x,y), self.current_idx, lookahead=50)
        self.current_idx = min_idx
        target = self.path[min_idx]
        v_ref = target[4]

        return v_ref, target