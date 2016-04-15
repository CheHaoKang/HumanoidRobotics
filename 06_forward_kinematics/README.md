The task of computing the end-effector pose given the current joint encoder readings is called
forward kinematics. In this exercise, we will compute the forward kinematics for the left arm of a
Nao robot.

For the kinematic computations, we will use the Denavit-Hartenberg (DH) representation. Each
joint is represented by a coordinate system (`X`, `Y`, `Z`) and four parameters (`d`, `a`, `theta`, `alpha`). 
The Z axis is always the rotation axis of the joint. For an explanation of the other axes and parameters, 
see the [video by Ethan Tira-Thompson](https://www.youtube.com/watch?v=rA9tm0gTln8).

The layout of Nao's joints are given on slide 5 of the slides on whole-body self-calibration. The DH
parameters of the left arm's kinematic chain are as follows:

| joint name     | abbrev. | d [meters] | a [meters] | theta [radians] | alpha [radians] |
| -------------- | ------- | ---------- | ---------- | --------------- | --------------- |
| LShoulderPitch | LSP     | 0          | 0          | 0               |  pi/2           |
| LShoulderRoll  | LSR     | 0          | 0          | pi/2            |  pi/2           |
| LElbowYaw      | LEY     | 0.0900     | 0          | 0               | -pi/2           |
| LElbowRoll     | LER     | 0          | 0          | 0               |  pi/2           | 
| LWristYaw      | LWY     | 0.0506     | 0          | 0               |  pi/2           | 

The point where the arm is attached to the torso is represented by the homogeneous 
transformation matrix
```
|     ( 1   0   0   0     )
|     ( 0   0   1   0.098 )
|     ( 0  -1   0   0.100 )
|     ( 0   0   0   1     )
```
and the point where the hand is attached to the other end of the arm is given by 
```
|     ( 0   1   0   0.0159 )
|     ( 1   0   0   0.0580 )
|     ( 0   0  -1   0      )
|     ( 0   0   0   1      )
```

The goal of this exercise is to compute the transformation `F_E^B(q)` from the left hand frame 
`E` to the robot's base frame `B` given the encoder readings `q`. See the exercise sheet for
the derivation of the transformation matrices.

1. Implement the homogeneous rotation and translation matrices for the given axes in 
   `rotationX`, `rotationZ`, `translationX`, and `translationZ`.
2. Implement the method `getA` for computing A(q).
3. Implement the method `computeHandTransform` for computing the complete transformation
`F_E^B(q)` between the robot's torso and the hand end-effector.

If you have Gnuplot installed on your computer, then you can get an interactive 3D view
of the resulting trajectory by executing the commands
```sh
source devel/setup.bash
rosrun forward_kinematics forward_kinematics_node
rosrun forward_kinematics plot.gp
```
in your Catkin workspace folder. On the Wiki, you'll find the same plot and also a video of
the robot's motion in a simulation environment.

