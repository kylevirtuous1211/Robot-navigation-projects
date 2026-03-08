
# Code structure

preloop: Initializes the heap with the start node and its f-score.

step: Pops the node with the lowest f-score, expands neighbors, and updates their costs and parents if a better path is found.

postloop: Returns the reconstructed path and the set of visited nodes.


# A Star
Pop the node with the lowest $f$ (The "Most Promising" node).
Lookup its $g$ (The cached "Distance traveled").
Update neighbors: If g[current] + distance < g[neighbor], you've found a better way! Update the cache and put the neighbor back in the heap.

# RRT
Classic RRT:

Greedy and fast. Once a node is added to the tree, its parent never changes.
The Problem: It often finds very jagged, "zig-zag" paths because it's just trying to fill space, not find the shortest route.
Result: Sub-optimal path.
RRT* (The "Optimizer"):

Choose Parent (Look Ahead): When adding a new node $z_{new}$, it doesn't just connect to the nearest neighbor. It looks at all nodes within search_radius and picks the one that results in the lowest total cost from the start.
Rewire (The Magic): This is the "optimization" part. After adding $z_{new}$, RRT* looks at all other nodes in the search_radius. If passing through $z_{new}$ would give an already existing node a shorter path to the start, it changes that node's parent to $z_{new}$.
Result: As you add more samples ($N \to \infty$), the path "straightens out" and converges to the optimal path.

# Reinforcement Learning
**Strategy: PPO (Proximal Policy Optimization)**
- **Why?**: It offers the best balance between implementation complexity and training stability. Its "clipping" mechanism ensures the agent doesn't over-correct its mistakes, leading to smoother learning curves in navigation tasks.
- **Environment**: A custom Gymnasium wrapper that provides the agent with its relative position to the goal and a local "Lidar" view of the occupancy map.