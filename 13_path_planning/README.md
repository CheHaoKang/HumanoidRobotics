# Exercise 12: Path planning with A*

Implement the A* algorithm for planning a path on a 2D grid map. All the methods that need
to be implemented are in the file `13_path_planning/src/PathPlanning.cpp`, you don't have to
change any other files.

The code provides implementations for the open list and closed list. The open list is a priority
queue where you can add grid nodes together with a cost value. The method removeMin() will
then always return and remove the node with the lowest costs.

At the beginning of each method that you have to implement, you will find a list of classes and
methods that you can use in your implementation.

1. Implement the cost function for moving from the current node to the next node in `getCosts`.
Use the Euclidean distance metric.
2. Implement the straight line distance heuristic for estimating the distance to the goal node in
`StraightLineDistanceHeuristic::heuristic`.
3. Implement the Manhattan distance heuristic in `ManhattanDistanceHeuristic::heuristic`.
4. Implement the method `getNeighborNodes`. For the cell given as an argument, this method
should return a vector of all nodes in the neighborhood (up, down, left, right, and diagonally).
It should only return the cells that are reachable for the robot, i.e., cells that are within the
map boundaries and that are not occupied by an obstacle.
5. Implement the method `expandNode`. This method should expand the current node and add
new nodes to the open list. It should also
  * calculate the costs for each neighbor,
  * set the predecessor of the neighbor nodes (that will be used later to extract the path),
  * update the cost and heuristic values of the neighbors that are already in the open list.
6. Implement the method `planPath`. It should
  * add the start node to the open list,
  * process the open list in a loop by removing the node with minimum costs,
  * expand that node and put it onto the closed list,
  * check if the goal has been reached using `isCloseToGoal()`,
  * call followPath for extracting the final path once the goal has been reached.
7. Implement the method `followPath` for extracting the final path by following the predecessors
from the current node back to the start node and returning them in the correct order.

If you have Gnuplot installed on your computer, then you can get run `rosrun path_planning plot.gp`
to show an animation of the node expansion and the final path found by your implementation.


