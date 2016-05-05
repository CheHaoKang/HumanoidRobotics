# Exercise 14: Footstep planning

Humanoid robots need to plan foot steps for getting from one location to another. In this exercise,
we will extend the A* algorithm from the previous exercise for planning foot steps.

1. Implement the cost function for moving from the current footstep to the next footstep in
`getCosts()` according to the cost function defined on slide 13. Choose `k = 0.3` for the stepping
costs and for the costs `d(s')` for the distance to the nearest obstacle use the function
```
.             | (r-D)^2 / D          if D < r
.     d(s') = |
.             | 0                    otherwise
```
where the distance `D` to the nearest obstacle is provided by the method
`getDistanceToNearestObstacle(successorFootstep)`. Choose `r = 0.2` for the clipping
radius (if the robot if further away than `r`, the obstacle costs will drop to 0).

2. Implement the Euclidean distance heuristic for the foot steps in `heuristic()`.

3. Calculate the new coordinates `(x', y', theta')` that the foot moves to when the robot executes the
given footstep action `delta_x, delta_y, delta_theta` when it is currently in `(x, y, theta)` in `executeFootstep`.

4. Implement the method `getNeighborNodes()` that should return the neighbor foot steps that
are reachable from the current foot step. The possible foot step actions are given as a parameter
to the method. You can use the method executeFootstep defined above, and the method 
`bool isColliding(footstepNode)` that is already provided by the code.
If you have Gnuplot installed, you can display an animation of the footstep sequence by running
`rosrun footstep_planning plot.gp`

