
import cv2
import numpy as np

from path_planning import *
from path_planning.rrt_star_planner import RRTStarPlanner


class RRTStarImplementation(RRTStarPlanner):
    # TODO: implement your own version of preloop, step and postloop
    def preloop(self):
        # This is for illustrative purposes only, feel free to modify
        self.visited_nodes:set[PathNode] = set()
        self.visited_nodes.add(self.start_node)

    def step(self):
        # ===== some given data/parameters =====
        self.start_node
        self.goal_node
        self.world_map # bgr
        self.occupancy_map # bool
        self.goal_threshold
        self.step_size # for steer
        self.search_radius # for reparent/rewire
        # ==========
        random_node = self.sample_random_node() # must use this to 
                                                # sample new node
        self.is_done.set() # only set this on termination
    
    def postloop(self):
        return (
            collect_path(self.goal_node), 
            set() # replace with set of visited nodes
        )