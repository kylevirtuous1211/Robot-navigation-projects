import random

from .primitives import PathNode, PixelCoordinates
from .planner import Planner


class RRTStarPlanner(Planner):
    def planner_specific_init(
        self, 
        step_size:float, 
        search_radius:float
    ) -> None:
        self.step_size = step_size
        self.search_radius = search_radius

    def sample_random_node(self) -> PathNode:
        p = random.random()
        if p <= 0.3:
            return self.goal_node
        map_height, map_width = self.occupancy_map.shape[:2]
        x = int(random.uniform(0, map_width))
        y = int(random.uniform(0, map_height))
        coordinates = PixelCoordinates(x, y)
        random_node = PathNode(coordinates=coordinates)
        return random_node
        