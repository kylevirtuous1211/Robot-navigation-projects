
import cv2
import numpy as np
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from path_planning.primitives import PixelCoordinates, PathNode
from path_planning.planner_utils import visualize_path, visualize_start_goal
from your_implementation.rrt_star_implementation import RRTStarImplementation

def verify_safety_path():
    map_path = "/home/cvlab2080/Robot-navigation-projects/HW1/data/map1/map.png"
    world_map = cv2.imread(map_path)
    
    # Coordinates that usually result in a path close to walls if not careful
    start_px = PixelCoordinates(200, 150)
    goal_px = PixelCoordinates(600, 550)
    
    planner = RRTStarImplementation()
    print("Planning safety-aware path...")
    path, nodes = planner.plan(
        start_px, 
        goal_px, 
        world_map, 
        goal_threshold=30.0, 
        iteration_limit=5000,
        step_size=30.0,
        search_radius=50.0
    )
    
    canvas = world_map.copy()
    canvas = visualize_start_goal(canvas, PathNode(start_px), PathNode(goal_px))
    if len(path) > 1:
        canvas = visualize_path(canvas, path)
        print(f"Path found with {len(path)} nodes.")
    else:
        print("No path found.")
        
    cv2.imwrite("safety_aware_path.png", canvas)
    print("Saved visualization to safety_aware_path.png")

    # Also save the distance transform for debugging
    binary_map = (1 - planner.occupancy_map).astype(np.uint8) * 255
    dist_map = cv2.distanceTransform(binary_map, cv2.DIST_L2, 5)
    dist_vis = cv2.normalize(dist_map, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    cv2.imwrite("dist_map_vis.png", dist_vis)
    print("Saved distance map visualization to dist_map_vis.png")

if __name__ == "__main__":
    verify_safety_path()
