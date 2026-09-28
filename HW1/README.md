# HW1 - Path Planning

A* and RRT* on three occupancy maps, implemented on top of the course's `preloop` / `step` / `postloop` planner template (`your_implementation/`).

![A* vs RRT* on map 2](assets/map2_rrt_star_output.png)

## Results

Blue is the final path, gray is every node the planner explored, and the start and goal are the colored dots.

| Map | A* (50 px grid) | RRT* (50 px step, 200 px rewire radius) |
|---|---|---|
| map1 | ![map1 A*](assets/map1_a_star_output.png) | ![map1 RRT*](assets/map1_rrt_star_output.png) |
| map2 | ![map2 A*](assets/map2_a_star_output.png) | ![map2 RRT*](assets/map2_rrt_star_output.png) |
| map3 | ![map3 A*](assets/map3_a_star_output.png) | ![map3 RRT*](assets/map3_rrt_star_output.png) |

| Map | A* path length | A* nodes explored | RRT* path length | RRT* nodes explored |
|---|---:|---:|---:|---:|
| map1 | 1652 px | 148 | **1617 px** | 152 |
| map2 | 1683 px | 176 | **1540 px** | 655 |
| map3 | 1493 px | 100 | **1443 px** | 102 |

A* is optimal on its 8-connected 50 px grid, so its paths are staircases.
RRT* connects nodes at any angle and rewires toward cheaper parents, so it finds 2-9% shorter paths.
The cost shows on the cluttered map2: RRT* explores almost 4x as many nodes and takes 2.3 s, against 0.02 s for A*.

## Method

- **A\*** (`your_implementation/a_star_implementation.py`): heap ordered by `f = g + h` with an L2 heuristic and a closed set; the path is rebuilt by backtracking parent pointers.
- **RRT\*** (`your_implementation/rrt_star_implementation.py`): sample, steer at most one step toward the sample, collision-check, choose the cheapest parent within the search radius, then rewire neighbors through the new node when that lowers their cost.

The full write-up is in [`report.md`](report.md).

## Reproduce

```bash
cd HW1
uv venv && uv pip install -r requirements.txt
for map in map1 map2 map3; do
  for planner in a_star rrt_star; do
    .venv/bin/python main.py -p $planner -m $map   # writes <map>_<planner>_output.png
  done
done
```

Assignment spec: [`HW1_path_planning.pdf`](HW1_path_planning.pdf).
