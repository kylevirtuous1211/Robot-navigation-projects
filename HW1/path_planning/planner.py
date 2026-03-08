from abc import abstractmethod
from threading import Event
from typing import final, Iterable

import cv2
import numpy as np
from numpy.typing import NDArray

from .primitives import PathNode, PixelCoordinates
from .planner_utils import world_map_to_occupancy_map


class Planner:
    @final
    def plan(
        self, 
        start_coordinates:PixelCoordinates, 
        goal_coordinates:PixelCoordinates, 
        world_map:NDArray, 
        goal_threshold:float, 
        iteration_limit:int=1000000, 
        *args, 
        **kwargs
    ) -> tuple[list[PathNode], set[PathNode]]:
        self.iteration_limit = iteration_limit
        if not isinstance(start_coordinates, PixelCoordinates):
            start_coordinates = PixelCoordinates(*start_coordinates)
        if not isinstance(goal_coordinates, PixelCoordinates):
            goal_coordinates = PixelCoordinates(*goal_coordinates)
        self.start_node = PathNode(coordinates=start_coordinates)
        self.goal_node = PathNode(coordinates=goal_coordinates)
        if len(world_map.shape) < 3:
            world_map = cv2.cvtColor(world_map, cv2.COLOR_GRAY2BGR)
        self.world_map = world_map.copy()
        self.occupancy_map = world_map_to_occupancy_map(world_map)
        self.goal_threshold = goal_threshold
        self.is_done = Event()
        self.planner_specific_init(*args, **kwargs)

        self.preloop()
        for _ in range(self.iteration_limit):
            self.step()
            if self.is_done.is_set(): break
        else:
            pass # timeout
        path, visited_nodes = self.postloop()
        return path, visited_nodes

    @abstractmethod
    def planner_specific_init(self) -> None:
        ...

    @abstractmethod
    def preloop(self) -> None:
        ...

    @abstractmethod
    def step(self) -> None:
        ...

    @abstractmethod
    def postloop(self) -> tuple[list[PathNode], set[PathNode]]:
        ...