
from .primitives import PathNode, PixelCoordinates
from .planner_utils import check_inside_map, check_collision_free
from .planner import Planner


class AStarPlanner(Planner):
    def planner_specific_init(
        self, 
        grid_size:int
    ) -> None:
        self.grid_size = grid_size
        self.direction_vectors = [
            (-1, -1), (-1,  0), (-1,  1), ( 0, -1), 
            ( 0,  1), ( 1, -1), ( 1,  0), ( 1,  1)
        ]
        self.sampled_nodes:dict[PixelCoordinates, PathNode] = {}

    def get_neighbor_nodes(
        self, 
        current_node:PathNode
    ) -> list[PathNode]:
        neighbor_nodes:list[PathNode] = []
        current_coordinates = current_node.coordinates
        for direction_vector in self.direction_vectors:
            direction_x, direction_y = direction_vector
            neighbor_coordinates = PixelCoordinates(
                x=current_coordinates.x + self.grid_size*direction_x, 
                y=current_coordinates.y + self.grid_size*direction_y
            )
            sampled_node = self.sampled_nodes.setdefault(
                neighbor_coordinates, 
                PathNode(
                    coordinates=neighbor_coordinates, 
                )
            )
            if not check_inside_map(
                occupancy_map=self.occupancy_map, 
                path_node=sampled_node
            ):
                continue
            if not check_collision_free(
                occupancy_map=self.occupancy_map, 
                source_node=current_node, 
                target_node=sampled_node
            ):
                continue
            neighbor_nodes.append(sampled_node)
        return neighbor_nodes