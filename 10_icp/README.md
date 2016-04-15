# Exercise 10: Iterative Closest Point (ICP)

Apply ICP algorithm to allign the set of 2D points `P` to those of set `Q` (as close as possible).
```
P = {(4,-3), (5,-4), (2.5,-4.5), (5,-6), (-3,-7.5), (5.5,-8), (5.8,-9)}
Q = {(1,1), (2.5,1.5), (3,0.5), (4,1), (5,-0.5), (7,3.1), (8.9,3)}
```
Where, the points are given in `(x,y)` form (ex: `(p1_x,p1_y) = (4,-3)`).

Also, compare between using the `closest point` and `point-to-line` matching methods to find the corresponding points of `Q` in `P`.

Note: The line is represented by the closest point and the closer of its two direct neighbours.

**Exercise Steps:**

Implement the missing parts in `10_icp/src/ICP.cpp` according to the following instructions:
1. Implement the function `distance` that calculates the Euclidean distance between a couple of 2D points.
2. Implement the function `closestPointOnLine` that computes the closest point that lies on a given line to a given 2D point.
3. Implement the function `min` that gets the minimum value within vector of values.
4. Implement the function `euclideanCorrespondences` that computes the corresponding points in `P` to those 
points in `Q`, using the 'closest point' matching method.
5. Implement the function `closestPointToLineCorrespondences` that computes the corresponding points in `P` to those points in `Q`, 
using the `point-to-line` matching method.
6. Implement the function `calculateAffineTransformation` that computes the affine transformation matrix needed to 
allign the previously computed corresponding points to the points of `Q`.
7. Implement the function `applyTransformation` that applies the affine transformation matrix on the points in `P`.
8. Implement the function `computeError` that computes the error between the points in `Q` and the transformed corresponding points.
9. Implement the function `iterateOnce` that performs one iteration of ICP and prints the error of that iteration.
