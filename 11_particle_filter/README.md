# Exercise 11: Particle filter

A vacuum cleaner robot drives through a corridor with three ceiling lights. 
The robot does not know its starting position within the corridor, but it 
has a map of the corridor indicating the positions of the three light 
sources:

```
|   --0--1--2--3--4--5--6--7--8--9--10-->
|          (L)         (L)   (L) 
```

The robot is equipped with a light sensor and from the measured brightness 
it can calculate the distance to the nearest light source.

The robot provides a log file containing odometry measurements (i.e., the 
displacements of the robot) and the measured distance to the nearest light 
source for a number of time steps. Localize the robot on the map using a 
particle filter (Monte Carlo localization) according to the following steps:

1. Implement the function `gaussianProbability` that calculates the 
probability of a measurement according to a Gaussian distribution. This 
function is called `phi(d, sigma)` on slide 77. `sigma` is the standard 
deviation of the distribution.
2. Implement the function `sampleFromGaussian` that draws a sample from a 
Gaussian distribution with mean `mu` and standard deviation `sigma` 
(see slide 66). For generating random numbers, you can use the standard C 
function `rand()` that returns a random integer between 0 and `RAND_MAX`.
3. Implement the function `initParticles` that initializes the particles as
 follows:
  * The position `x` of the robot should be linearly distributed in the 
    interval [0, 10]. You can either distribute the positions equidistantly 
    or sample the positions from a linear distribution in the given interval.
  * All particles should have the same weight weight and the weights must 
    sum up to 1.
4. Implement the function `normalizeWeights` that normalizes the weights of 
   the particles so that they sum up to 1.
5. When the robot travels for the distance `ux`, the particles have to be 
   displaced accordingly. However, `ux` is subject to measurement errors 
   and noise. Hence, instead of shifting all particles by the same distance 
   `ux`, we sample the displacement from a Gaussian with mean `ux`. 
   Implement the function `integrateMotion` for sampling a new position for 
   each particle according to this motion model.
   This step is called the *prediction step* of the Monte Carlo filter.
6. Implement the method `getDistanceToNearestLight` that calculates the 
   distance from the robot's position `x` to the nearest light source 
   according to map shown above.
7. While traveling, the robot measures the distance to the nearest light 
   source using its light sensor. Integrate the measurement in the method 
   `integrateObservation` by changing the particles' weights according to 
   the observation model
```
p(z~ | x) = phi(z - z~, sigma)
```
   where `z~` is the measured distance (given as an argument to the function), 
   `z` is the distance between the particle pose and the nearest light 
   source (calculate this for each particle using 
   `getDistanceToNearestLight`), and `sigma` is the standard deviation 
   (given as an argument).
    This step is called the *correction step* of the Monte Carlo filter.
8. Implement low variance resampling in the method `resample` according to 
   slides 34-36.
   
Hint: When doing calculations, make sure to cast integer values to double 
first. The expression `1.5 + 3/2` will evaluate to `2.5` because `3/2` will 
be truncated to the integer 1, whereas `1.5 + 3.0/2.0` will evaluate to 
`3.0` as expected.

If you have Gnuplot installed on your computer, then you can get run the 
commands
```bash
source devel/setup.bash
rosrun particle_filter plot.gp
```
in your Catkin workspace directory to generate 20 plots in the folder 
`11_particle_filter/data` that show how your particle set evolves over time.

