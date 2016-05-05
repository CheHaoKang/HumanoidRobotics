# Exercise 16: Rapidly Exploring Random Trees (RRTs)

Consider a robot in a 2D environment represented by a grid map. Our objective is to find a path
between a start cell and a goal cell. The robot is initially located at the start cell's center and it
can move to any of the eight neighboring cells' centers (if they are not occupied). Use RRTs to find
that a possible path for the robot.

**Exercise steps:**
1. Implement the method `getRandomNode` that returns a random grid cell in the map, with the
   following conditions:
   * The cell must be free of obstacles.
   * The cell must not already belong to the tree that will be expanded in the current step.
   * The random generation should be biased towards the goal of the current tree, i.e., the
     method should return the goal node instead of a random node with a probability of 10%.
   * Hint: Use the standard C function `rand()` that returns a random integer between 0 and
     `RAND_MAX`.
2. Implement the method `distance` that computes the Euclidean distance between two given
   cells.
3. Implement the method `getClosestNodeInList` that returns the grid cell with the smallest
   distance to the current node from a list of cells.
4. Implement the method `getNeighbors` that returns a list of neighbors of the current cell that
   are not occupied and have not already been expanded in the current tree.
5. Implement the method `tryToConnect` that checks for each neighbor whether it is is already
   contained in the list of expanded nodes of the other tree. If this is the case, then we have
   found a connection between the two trees and the method should return the neighbor as the
   connection node. Otherwise, the method should return NULL.
6. Implement the method `addNearestNeighbor` that determines which neighbor is the nearest
   with respect to the random node and adds that neighbor to the tree.
7. Have a look at how the method `extendClosestNode` combines the above methods 
   (you don't have to implement anything here).
8. Implement the method `constructPath` that reconstructs the path from the start node to the
goal node once the trees have been connected.
9. Implement the method `planPath` which grows the trees from the start and goal nodes and
returns the complete path, using the RRT-connect algorithm on slide 25.

If you have Gnuplot installed on your computer, then you can run `rosrun rrt plot.gp` to show
an animation of the node expansion and the final path found by your implementation.
