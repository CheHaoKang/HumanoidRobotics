# Exercise 12: A*

For the grid map given below, assume that your robot is located at the cell labeled with `R`, and
your target is located at the cell labeled with `T`. Moreover, assume that all the cells labeled with `X`
are considered as occupied cells. Apply the A* algorithm to get the shortest path for your robot to
reach its goal; given that the side length of each cell is 10 cm.

```
| X |   | X | X |   |   |   |   |   |   |
|   |   | X | X |   |   |   |   |   |   |
|   |   | X | X |   |   |   |   |   |   |
|   |   |   |   |   |   |   |   |   |   |
| T | X | X | X |   |   | X | X |   |   |
|   |   |   | X |   |   | X | X | X |   |
|   |   |   | X | X |   |   |   |   | X |
|   |   | X | X |   | X |   |   |   |   |
| X |   |   |   |   |   |   | X |   | X |
|   | X |   |   |   |   |   | R | X | X | 
```

From its current cell, the robot can move up, down, left, and right to the next cell, but it cannot
move diagonally. Choose an admissible heuristic for planning the path.

**Exercise steps:**
1. Explain which heuristic you chose and why.
2. Draw the shortest path and show the steps of your solution.
3. Save your explanations for a) and the drawn tree to a file named as `12_astar/solution.pdf`
(Note: hand-drawn figures are acceptable as well as computer-generated ones).
4. Git add, commit and push that file to the exercise workspace.

