# Exercise 9: Signed Distance Function

In this exercise you will implement the signed distance function to estimate
a 2D map from a series of laser range measurements recorded by a robot while
travelling through an environment.

Each measurement consists of the 2D robot position and a list of 2D laser end points
indicating where the laser beams hit obstacles. The existing code already loads the 
data into the following struct:

```C++
struct Measurement {
	Eigen::Vector2d robotPose;
	std::vector<Eigen::Vector2d> laserPoints;
};
```

All input data and constants in this exercise have already been converted to grid cells
in the map coordinate system, so there is no need to convert between units or coordinate
systems.

1. Implement the Euclidean distance between two points in `calculateDistance()`.
2. Implement the truncated distance function in `truncateDistance()` according
to Fig. 1(a) on the exercise sheet.
3. Implement the weighting function in `calculateWeight()` according to Fig. 1(b) on the 
exercise sheet.
4. Update the signed distance value of a grid cell in `updateMap()` according to the rule
```
D = (W * D + w * d) / (W + w)
```
`W` is the weight of the current cell before the update, `w` is the weight of the current measurement.
`D` is the signed distance function value of the grid cell before the update and `d` is the signed
distance function value from the current measurement. 
5. Update the weight value of a grid cell in `updateWeight()` according to the rule
```
W=W+w
```
6. Implement the method `integrateLaserScan` that integrates a new laser measurement
into the map and updates the weights associated with the grid cells.
  1. Iterate over all laser points in the current measurement.
  2. Use the existing method `bresenham(A, B)` to get a list of cells that lay on the
     straight line through the points A and B. This modified Bresenham algorithm gives you the cells not
     only between the points `A` and `B`, but also beyond `B` up to `B+(B-A)`.
     The remaining two arguments `numRows`, `numCols` are the number of rows and columns
     of the map matrix.
  3. For each point returned by the Bresenham algorithm compute the signed distance function and
     weight with the methods defined above and update the grid cell and the associated weight. 

Be careful with the following implementation issues:
* Following common conventions, the `x` axis corresponds to the columns of the map matrix
  and the `y` axis corresponds to the rows of the map matrix.
* Update a map cell only if the weight of the current measurement is greater than 0, 
  otherwise Equation (1) will cause division by zero errors.
* Make sure that you cast row and column indices from `double` to an integer type when 
  accessing elements of the map and weight matrices.

If you have Gnuplot installed on your computer, then you can get an interactive
3D view of the resulting signed distance map as well as a 2D view of the walls and 
robot trajectory by executing the commands

```sh
source devel/setup.bash
rosrun signed_distance_function signed_distance_function_node
rosrun signed_distance_function plot_3d.gp
rosrun signed_distance_function plot_map.gp
```



