# Exercise 17: Inverse reachability maps (IRM)

In this exercise, we will compute reachability maps and inverse reachability maps for the robot arm
with three links from exercise 15.

## Reachability map

Compute the reachability map of the endeffector with respect to the robot's base according to the
following steps:

1. Implement the method `sampleConfiguration` for choosing a random configuration of the
robot's joints `(q_0, q_1, q_2)` and return the configuration as a vector. The joint angles should be
sampled uniformly within the following bounds:
```
| joint   | min     | max     |
| ------- | ------- | ------- |
| q_0     | 0       | +pi/2   |
| q_1     | -pi     | +pi     |
| q_2     | -pi     | +pi     |
```
2. Compute the following measurement of manipulability for a given joint configuration in
`computeManipulability()`:
```
score := 1 - 1 / (4 * pi) (|4 * q_0 - pi| + |q_1| + |q_2| + |e_theta|)
```
(This manipulatibility measurement favors configurations where `q_0` is near 45 degrees and
penalizes configurations with pointed angles between links.)
3. Implement the method `computeRM` that computes the reachability map of the endeffector by
   iterating the following steps:
  * Sample a joint configuration.
  * Compute the end effector pose with the `forwardKinematics` method.
  * Check that the end effector is above the ground (i.e., `e_y > 0`). If the end effector 
    collides with the ground, then sample again. (You don't have to perform other checks 
    such as self-collisions, object collisions, etc. in this exercise.)
  * Compute the manipulability score for the configuration.
  * Add the configuration to the reachability map with the `addToRM` method.

With `rosrun irm plot-rm.gp` or in the Wiki you can inspect the resulting reachability map. The
robot's base is located at the origin and the colors represent the manipulability of the possible end
effector poses.

## Inverse reachability map
For the sake of simplicity, we assume that the robot arm is mounted on a wheeled base so that we
do not have to deal with stance feet and swing feet.

The inverse reachability map indicates suitable positions where the base of the robot has to be
located so that the gripper can reach the desired object. Using this map, the robot can first drive
to a suitable place near the table and then grasp the drink.

4. Implement the method `computeIRM`. The voxels of the reachability map are given to the
   method as an argument. For each configuration stored in the reachability map, add an entry
   to the inverse reachability map as follows:
   * Compute the end effector pose from the configuration's joint angles with the
     `forwardKinematics` method.
   * Convert the end effector pose to a homogeneous transformation matrix, find the inverse,
     and convert the inverted transformation matrix back to the pose of the robot's base
     `(b_x, b_y, b_theta)`. The result is the pose of the base expressed in the coordinate 
     system of the gripper.
   * Add the base pose together with the joint angles and the manipulability score to the
     inverse reachability map by calling the `addToIRM` method.

With `rosrun irm plot-irm.gp` or in the Wiki you can inspect the resulting inverse reachability
map. The robot's gripper is located at the origin and the colors represent the possible base poses
that are suitable for reaching the object in the coordinate frame of the gripper.
