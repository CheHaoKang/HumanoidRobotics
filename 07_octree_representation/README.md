# Exercise 7: Octree Representation

Assume that we have a 3D environment where the length of each side is 128 cells.
Split the map into a left and a right half and add a cube-shaped object at
the center of each of the two halves, as shown in the figure on the exercise
sheet. The two cubes have a side length of 16 cells each. 

We would like to represent this environment in a voxel grid and as an 
octree and compare the number of cells that each representation needs. 

The octree algorithm starts with the complete cube and subdivides it 
recursively into eight smaller cubes. It only splits cubes that are neither 
fully occupied nor fully empty until all cubes are either occupied or free.

**Exercise steps:**
1. Calculate the number of cells needed in a uniform 3D voxel grid 
   representation versus the number of cells in an octree representation 
   for the given environment.
2. Create a file `07_octree_representation/solution` in the exercise 
   workspace. Write your solution in the first line of the file in the
   following format:

   OctreeCells VoxelCells

   Example: If you need 8 cells for the octree and 256 cells for the voxel 
   grid, then write: 8 256

Note: No formulas are allowed (ex: 4 is OK, 2*2 is not acceptable).

3. Git add, commit and push that file as usual.

