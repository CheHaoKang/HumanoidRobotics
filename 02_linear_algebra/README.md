# Exercise 2: Linear algebra and the Eigen library

Linear algebra will be a prerequisite for many applications in the humanoid
robotics lecture. A popular library for linear algebra algorithms is *Eigen*.
Eigen does not have any requirements except for the standard C++ library
and is available in most Linux package managers. We will use version 3.

* Official homepage: http://eigen.tuxfamily.org
* Getting started tutorial: http://eigen.tuxfamily.org/dox/GettingStarted.html

Make yourself familiar with the library and then implement the missing parts
of the file `02_linear_algebra/src/LinearAlgebra.cpp` according to the
following instructions:
1. Find out how to fill vector and matrices with elements, then complete 
the functions `vectorA()`, `vectorB()`, and `matrixM()` so that they return the 
following values:
```
|       ( 2 )          ( -1 )           (  1   2   7  )
|  a := ( 1 )     b := ( -5 )      M := (  0   2   0  )
|       ( 3 )          (  2 )           (  1   0  -1  )
```
2. Find out how to get the inverse of a matrix, then complete the function `invMatrixM(M)`.
3. Find out how to get the transpose of a matrix, then complete the function `transposeMatrixM(M)`.
4. Complete the function `detOfMatrixM(M)`, which returns the determinate of matrix M.
5. Find out how to compute the *dot product* (also known as *inner product*)
and complete the function `dot(a,b)`.
6. Complete the function `isLinearIndependent` for determining whether two vectors are linearly
dependent or not.
7. Find out how to solve linear systems and solve the equation `Mx = a`
for `x` and complete the function `solveLinearSystem(M,a)`.

As always push your changes to the GIT server and check the results in the Wiki.
