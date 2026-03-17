
import cv2
import numpy as np

from path_planning import *
from path_planning.rrt_star_planner import RRTStarPlanner


class RRTStarImplementation(RRTStarPlanner):
    def preloop(self):
        self.visited_nodes:set[PathNode] = {self.start_node}
        self.start_node.cost = 0
        
        # Expand invalid area from border (50 pixels)
        self.occupancy_map[:50, :] = 1
        self.occupancy_map[-50:, :] = 1
        self.occupancy_map[:, :50] = 1
        self.occupancy_map[:, -50:] = 1
        
        # Compute distance transform for safety (distance to nearest obstacle)
        # occupancy_map is 1 for obstacles, 0 for free space
        # distanceTransform needs a binary image where 0 is the object (obstacle)
        binary_map = (1 - self.occupancy_map).astype(np.uint8) * 255
        self.dist_map = cv2.distanceTransform(binary_map, cv2.DIST_L2, 5)
        # Normalize/Scale for cost usage
        self.max_clearance = np.max(self.dist_map)
    
    def _get_safety_cost(self, node):
        x, y = int(node.coordinates.x), int(node.coordinates.y)
        h, w = self.dist_map.shape
        if 0 <= x < w and 0 <= y < h:
            dist = self.dist_map[y, x]
            # Penalty is higher when closer to walls
            # 1.0 at wall, approaching 0 far away
            return 100.0 / (dist + 1.0)
        return 1000.0 # Should be inside map anyway

    def step(self):
        # 1. Sample
        random_node = self.sample_random_node()
        
        # 2. Nearest
        nearest_node = min(
            self.visited_nodes, 
            key=lambda n: calculate_node_distance(n, random_node)
        )
        
        # 3. Steer
        dist = calculate_node_distance(nearest_node, random_node)
        if dist > self.step_size:
            # Linear interpolation / "Steering"
            ratio = self.step_size / dist
            new_x = nearest_node.coordinates.x + (random_node.coordinates.x - nearest_node.coordinates.x) * ratio
            new_y = nearest_node.coordinates.y + (random_node.coordinates.y - nearest_node.coordinates.y) * ratio
            new_node = PathNode(coordinates=PixelCoordinates(new_x, new_y))
        else:
            new_node = random_node

        # 4. Collision Check (Pre-insertion)
        if not check_inside_map(self.occupancy_map, new_node):
            return
        if not check_collision_free(self.occupancy_map, nearest_node, new_node):
            return

        # 5. Choose Parent (Optimization)
        # Find potential parents within search_radius
        near_nodes = [
            n for n in self.visited_nodes 
            if calculate_node_distance(n, new_node) <= self.search_radius
        ]
        
        best_parent = nearest_node
        # Normal distance + safety penalty
        edge_dist = calculate_node_distance(nearest_node, new_node)
        min_cost = nearest_node.cost + edge_dist + self._get_safety_cost(new_node)
        
        for near_node in near_nodes:
            edge_dist = calculate_node_distance(near_node, new_node)
            tentative_cost = near_node.cost + edge_dist + self._get_safety_cost(new_node)
            if tentative_cost < min_cost:
                if check_collision_free(self.occupancy_map, near_node, new_node):
                    best_parent = near_node
                    min_cost = tentative_cost
        
        new_node.parent = best_parent
        new_node.cost = min_cost
        
        # 6. Add to Tree
        self.visited_nodes.add(new_node)
        
        # 7. Rewire (Optimization)
        for near_node in near_nodes:
            if near_node == best_parent:
                continue
            
            edge_dist = calculate_node_distance(new_node, near_node)
            tentative_cost = new_node.cost + edge_dist + self._get_safety_cost(near_node)
            if tentative_cost < near_node.cost:
                if check_collision_free(self.occupancy_map, new_node, near_node):
                    near_node.parent = new_node
                    near_node.cost = tentative_cost
                    
        # 8. Goal Check
        if calculate_node_distance(new_node, self.goal_node) <= self.goal_threshold:
            if check_collision_free(self.occupancy_map, new_node, self.goal_node):
                self.goal_node.parent = new_node
                self.goal_node.cost = new_node.cost + calculate_node_distance(new_node, self.goal_node)
                self.visited_nodes.add(self.goal_node)
                self.is_done.set()
    
    def postloop(self):
        path = collect_path(self.goal_node)
        return path, self.visited_nodes