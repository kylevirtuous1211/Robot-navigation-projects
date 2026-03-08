
import cv2
import numpy as np

from path_planning import *
from path_planning.a_star_planner import AStarPlanner


class AStarImplementation(AStarPlanner):
    # TODO: implement your own version of preloop, step and postloop
    def preloop(self):
        # This is for illustrative purposes only, feel free to modify
        self.queue:set[PathNode] = set()
        self.g:dict[PathNode, float] = {}
        self.h:dict[PathNode, float] = {}
        self.visited_nodes:set[PathNode] = set()
        self.visited_nodes.add(self.start_node)
        self.queue.add(self.start_node)
        self.g[self.start_node] = 0
        self.h[self.start_node] = calculate_node_distance(
            self.start_node, 
            self.goal_node
        )

    def step(self):
        # ===== some given data/parameters =====
        self.start_node
        self.goal_node
        self.world_map # bgr
        self.occupancy_map # bool
        self.goal_threshold
        self.grid_size # for sampling neighbor nodes
        # ==========
        # to sample neighbor nodes, use 
        # self.get_neighbor_nodes(current_node)
        self.is_done.set() # only set this on termination

    def postloop(self):
        return (
            collect_path(self.goal_node), 
            set() # replace with set of visited nodes
        )