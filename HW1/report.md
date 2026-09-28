# Path Planning & Navigation Report

## 1. Code Structure Overview

*   **`preloop`**: Initializes internal data structures before the main search loop begins (e.g., seeding the heap with the start node).
*   **`step`**: The core iterative logic. Pops the most promising node, explores its neighbors, and updates path costs and parents if a better route is found.
*   **`postloop`**: Concludes the search by reconstructing the final path and returning it alongside the set of visited nodes.

---

## 2. A* (A-Star) Algorithm

### 2.1 `preloop`
Initializes essential data structures required for the search:
*   **`visited_nodes`**: A closed set. Due to the greedy property of A* with a consistent heuristic (like L2 distance), once a node is visited, it guarantees the shortest path to that node has been found. It does not need to be visited again.
*   **`g`**: A dictionary storing the current lowest cost from the start node to each known node.
*   **`f`**: A dictionary storing the estimated total cost ($f(n) = g(n) + h(n)$) for each node, including the heuristic distance to the goal.
*   **`heapq`**: A priority queue storing the nodes to be explored, strictly ordered by their $f$-score (with a counter to break ties).

### 2.2 `step`
Executes the following algorithm iteratively:
1.  **Pop** the node with the lowest $f$-score from the heap.
2.  **Mark** it as visited by adding it to `visited_nodes`.
3.  **Goal Check**: Check if the current node is within the distance threshold of the goal node.
4.  **Expand Neighbors**: Evaluate surrounding nodes. Update a neighbor's cost ($g$ and $f$) and set its parent to the current node if the new path is cheaper than any previously found path.
5.  **Push** updated neighbors back into the priority queue (`heapq`).

### 2.3 `postloop`
Recursively builds the path from the goal node. Because every `PathNode` records its parent during the search, the optimal path is reconstructed by simply backtracking from the goal back to the start node.

---

## 3. RRT* (Rapidly-exploring Random Tree Star)

### 3.1 `preloop`
Initializes the tree:
*   Sets the root `start_node`'s cost to 0.
*   **`visited_nodes`**: Initializes the set forming the spanning tree. In RRT*, this set is continuously referenced for both *reparenting* and *rewiring* newly discovered nodes.

### 3.2 `step`
Executes the following incremental growth algorithm:
1.  **Sample**: Randomly sample a target node in the configuration space.
2.  **Nearest / Steer**: Find the `new_node` by first identifying the `nearest_node` in the tree.
    *   If the random node is within the maximum step size, `new_node` becomes the random node itself.
    *   Otherwise, use linear interpolation (steering) to project a new node from `nearest_node` towards the random node, strictly limited by the step size.
3.  **Collision Check**: Ensure `new_node` is inside the map boundaries and the straight-line path to it is free of obstacles.
4.  **Choose Parent (Optimization 1)**: Find the optimal parent for `new_node`. Iterate through all existing tree nodes within a specific search radius to find the connection that yields the absolute minimum cost from the start.
5.  **Add to Tree**: Successfully link and insert `new_node` into `visited_nodes`.
6.  **Rewire (Optimization 2)**: Check if `new_node` serves as a better parent for its neighboring nodes in the tree. Iteratively calculate if routing adjacent nodes through `new_node` lowers their total cost, and update their parents if it does.
7.  **Goal Check**: Verify if `new_node` is sufficiently close to the goal.

### 3.3 `postloop`
Recursively traces the `.parent` pointers backward from the goal node to the start node to reconstruct the final path.

## 4. Conclusion
Thank you TA for such a nice code template! I really liked how utils has all the helper functions, and the A* and RRT* planner class also it's inherited from the base planner class, which makes the code clean and organized. 

It's my first time implementing both algorithms, A* feels like Dijkstra, only difference is the heuristic function creating the f function. RRT*'s sampling strategy is more versatile as it's not restricted to a discrete grid, also, implementing rewiring and reparenting strategy is a fun algorithm exercise! Really enjoyed this project!