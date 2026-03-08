from typing import Iterable

import cv2
import numpy as np
from numpy.typing import NDArray

from .primitives import PathNode


iteration_limit = 1000000

# https://rosettacode.org/wiki/Bitmap/Bresenham%27s_line_algorithm
def bresenham(
    x0:int, 
    x1:int, 
    y0:int, 
    y1:int
) -> list[tuple[int, int]]:
    rec = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = -1 if x0 > x1 else 1
    sy = -1 if y0 > y1 else 1
    if dx > dy:
        err = dx / 2.0
        while x != x1:
            rec.append((x, y))
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2.0
        while y != y1:
            rec.append((x, y))
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    return rec

def world_map_to_occupancy_map(
    world_map:NDArray
) -> NDArray:
    world_map = world_map.copy()
    if len(world_map.shape) >= 3:
        world_map = cv2.cvtColor(world_map, cv2.COLOR_BGR2GRAY)
    world_map = 255 - cv2.dilate(255 - world_map, np.ones((2, 2)))
    return np.where(world_map > 128, 0, 1)

def check_inside_map(
    occupancy_map:NDArray, 
    path_node:PathNode
) -> bool:
    height, width = occupancy_map.shape[:2]
    coordinates = path_node.coordinates
    if coordinates.x < 0 or coordinates.x >= width:
        return False
    if coordinates.y < 0 or coordinates.y >= height:
        return False
    return True

def check_collision_free(
    occupancy_map:NDArray, 
    source_node:PathNode, 
    target_node:PathNode
) -> bool:
    if (
        not check_inside_map(occupancy_map, source_node) or
        not check_inside_map(occupancy_map, target_node)
    ):
        raise RuntimeError()
    source_coordinates = source_node.coordinates
    target_coordinates = target_node.coordinates
    line_pixels = bresenham(
        x0=source_coordinates.x, 
        x1=target_coordinates.x, 
        y0=source_coordinates.y, 
        y1=target_coordinates.y
    )
    for pixel in line_pixels:
        if occupancy_map[pixel[1], pixel[0]] == 1:
            return False
    return True

def calculate_node_distance(
    source_node:PathNode, 
    target_node:PathNode
) -> float:
    source_coordinates = source_node.coordinates
    target_coordinates = target_node.coordinates
    return np.hypot(*((target_coordinates - source_coordinates).to_tuple()))

def collect_path(
    goal_node:PathNode
) -> list[PathNode]:
    path:list[PathNode] = []
    path.append(goal_node)
    for _ in range(iteration_limit):
        head = path[0]
        if head.parent is None:
            break
        path.insert(0, head.parent)
    else:
        pass # TODO: timeout
    return path

def visualize_start_goal(
    canvas:NDArray, 
    start_node:PathNode, 
    goal_node:PathNode
) -> NDArray:
    start_node_style = {
        "radius": 10, 
        "color": (0, 0, 255), 
        "thickness": -1
    }
    goal_node_style = {
        "radius": 10, 
        "color": (0, 255, 0), 
        "thickness": -1
    }
    cv2.circle(
        canvas, 
        start_node.coordinates.to_tuple(), 
        **start_node_style
    )
    cv2.circle(
        canvas, 
        goal_node.coordinates.to_tuple(), 
        **goal_node_style
    )
    return canvas

def visualize_visited_nodes(
    canvas:NDArray, 
    visited_nodes:Iterable[PathNode]
) -> NDArray:
    node_style = {
        "radius": 5, 
        "color": (100, 100, 100), 
        "thickness": -1
    }
    edge_style = {
        "color": (100, 100, 100), 
        "thickness": 2
    }
    for node in visited_nodes:
        cv2.circle(
            canvas, 
            node.coordinates.to_tuple(), 
            **node_style
        )
        if node.parent is None: continue
        cv2.line(
            canvas, 
            node.coordinates.to_tuple(), 
            node.parent.coordinates.to_tuple(), 
            **edge_style
        )
    return canvas

def visualize_path(
    canvas:NDArray, 
    path:list[PathNode]
) -> NDArray:
    node_style = {
        "radius": 5, 
        "color": (255, 0, 0), 
        "thickness": -1
    }
    edge_style = {
        "color": (255, 0, 0), 
        "thickness": 2
    }
    for parent_node, child_node in zip(path[:-1], path[1:]):
        cv2.circle(
            canvas, 
            parent_node.coordinates.to_tuple(), 
            **node_style
        )
        cv2.line(
            canvas, 
            parent_node.coordinates.to_tuple(), 
            child_node.coordinates.to_tuple(), 
            **edge_style 
        )
    return canvas