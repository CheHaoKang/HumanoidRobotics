# Exercise 8: Kd-trees
A kd-tree is an efficient data structure for storing 3D points (or even 
higher-dimensional tuples). In kd-trees, location queries can be executed 
in time O(log n), compared to O(n) in the case of ordinary lists.

The kd-tree is a special case of binary trees, but for k instead of two 
dimensions. There are many ways to select the root node and pivot elements. 
The pivot elements are the nodes of the tree which determine the splitting 
planes.

In this exercise, we are considering the simple, balanced kd-trees. In this
method, we select one of the available dimensions and sort the set of 
points based on the value of that selected dimension. Then, we select the 
median point as the pivot element (without replacement from the set of 
points). Based on that, the set of points is divided into two sets: 
one set contains the points of the left sub-tree which have smaller values 
compared to the pivot point (relative to the selected dimension), and the 
other set contains the remaining points. After that, this step will be 
repeated recursively for every set of points and we keep cycling through 
the different available dimensions throughout the tree levels.

Generate a kd-tree for the following set of 3D points:

    	|  5  8  2  -8  -7  5  7  3  -4  9 -1  0  0  2  4  9  10 |
    	|  0 -1 -10  5   7  7  3  2  -1  1  5  7  5  4  7 -8  -3 | 
	    | -3 -4  8   6  -6  15 4  2   3 -4  7  0  2 -1  4  5  -7 | 

Each column represents as point in the form of (x,y,z) 
(e.g., (x_1,y_1,z_1) = (5,0,-3)).

**Exercise steps:**
1. Gernerate the kd-tree for the given set of points and draw it.
2. Save your drawn tree to a file named as '08_kdtree/solution.pdf' 
(Note: hand-drawn figures are acceptable as well as computer-generated ones).
3. Git add, commit and push that file to the exercise workspace.

