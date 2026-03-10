import heapq
from itertools import count

import cv2
import numpy as np

from path_planning import *
from path_planning.a_star_planner import AStarPlanner


class AStarImplementation(AStarPlanner):
    def preloop(self):
        self.counter = count() # To handle nodes with same f-score
        self.visited_nodes:set[PathNode] = set()
        
        # g[node] is the cost of the cheapest path from start to node
        self.g:dict[PathNode, float] = {self.start_node: 0}
        self.start_node.cost = 0
        
        # f[node] = g[node] + h(node)
        start_h = calculate_node_distance(self.start_node, self.goal_node)
        self.f:dict[PathNode, float] = {self.start_node: start_h}
        
        # The priority queue storing (priority, count, node)
        self.heap = [(start_h, next(self.counter), self.start_node)]

    def step(self):
        if not self.heap:
            self.is_done.set()
            return

        # Pop the node with the lowest f_score
        f_score, _, current_node = heapq.heappop(self.heap)

        if current_node in self.visited_nodes:
            return
            
        current_node.cost = self.g.get(current_node, float('inf'))
        self.visited_nodes.add(current_node)

        # Check if goal reached
        if calculate_node_distance(current_node, self.goal_node) <= self.goal_threshold:
            # Update goal_node's parent and cost to reconstruct the path correctly
            self.goal_node.parent = current_node
            self.goal_node.cost = self.g[current_node] + calculate_node_distance(current_node, self.goal_node)
            self.visited_nodes.add(self.goal_node)
            self.is_done.set()
            return

        for neighbor in self.get_neighbor_nodes(current_node):
            if neighbor in self.visited_nodes:
                continue
                
            tentative_g_score = self.g[current_node] + calculate_node_distance(current_node, neighbor)
            
            if tentative_g_score < self.g.get(neighbor, float('inf')):
                neighbor.parent = current_node
                self.g[neighbor] = tentative_g_score
                f_score = tentative_g_score + calculate_node_distance(neighbor, self.goal_node)
                self.f[neighbor] = f_score
                heapq.heappush(self.heap, (f_score, next(self.counter), neighbor))

    def postloop(self):
        # If goal was not reached, goal_node.parent will be None, 
        # and collect_path will return [goal_node].
        path = collect_path(self.goal_node)
        return path, self.visited_nodes