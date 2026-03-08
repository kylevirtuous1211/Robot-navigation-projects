import argparse
import random
import json
from pathlib import Path

import cv2

from path_planning import *
from your_implementation import (
    AStarImplementation, 
    RRTStarImplementation
)


random.seed(9999)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-p", 
        "--planner", 
        type=str, 
        default="a_star", 
        help="a_star/rrt_star"
    )
    parser.add_argument(
        "-m", 
        "--map", 
        type=str, 
        default="map1", 
        help="the name of the map folder"
    )
    args = parser.parse_args()

    # initialize configs
    map_name = args.map
    map_folder = Path(__file__).parent / "data" / str(map_name)
    if not map_folder.is_dir():
        raise RuntimeError()
    map_path = map_folder / "map.png"
    world_map = cv2.imread(str(map_path))
    info_path = map_folder / "info.json"
    with open(info_path, "r") as file:
        info:dict[str, list[int]] = json.load(file)
    start_coordinates = info.get("start_coordinates")
    goal_coordinates = info.get("goal_coordinates")
    
    # initialize planner
    if args.planner == "a_star":
        planner = AStarImplementation()
        planner_argument = {
            "start_coordinates":start_coordinates, 
            "goal_coordinates":goal_coordinates, 
            "world_map":world_map, 
            "goal_threshold":50, 
            "grid_size":50,  
        }
    elif args.planner == "rrt_star":
        planner = RRTStarImplementation()
        planner_argument = {
            "start_coordinates":start_coordinates, 
            "goal_coordinates":goal_coordinates, 
            "world_map":world_map, 
            "goal_threshold":50, 
            "step_size":50, 
            "search_radius":200,  
        }
    else:
        raise RuntimeError()
    
    # plan
    path, vidited_nodes = planner.plan(**planner_argument)

    # visualization
    visualization = visualize_start_goal(
        world_map.copy(), 
        PathNode(PixelCoordinates(*start_coordinates)), 
        PathNode(PixelCoordinates(*goal_coordinates))
    )
    visualization = visualize_visited_nodes(visualization, vidited_nodes)
    visualization = visualize_path(visualization, path)
    cv2.imshow("", visualization)
    cv2.waitKey(0)
    cv2.destroyAllWindows()