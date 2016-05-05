# Exercise 15: Inverse kinematics

Consider a robot arm in a 2D environment as shown on the exercise sheet.
The arm has two links with lengths a_0, a_1, and a gripper of length h attached to the end of the arm.
The links are connected to each other and to the base with two rotational joints. The current joint
angles are q_0 and q_1 .

The robot would like to grasp an object with its gripper. The location g of the object is given.
Compute suitable joint angles q_0 , q_1 so that the gripper arrives at the object location g by applying
the inverse kinematics algorithm from slide 15 as follows:

1. Implement the method forwardKinematic that computes the endeffector pose e given the
current joint angles q. The equation for the endeffector pose in homogeneous coordinates is
```
(e_x, e_y, 1)^T = R(q_0) * T(a_0, 0) * R(q_1) * T(a_1, 0) * (h, 0, 1)^T
```
with the homogeneous rotation and translation matrices `R(theta)` and `T(x, y)`.
Do not forget to convert `e` back to Cartesian coordinates in the end, as homogenous coordinates
would disturb later when calculating the Jacobian.

2. Using pen and paper, derive the Jacobian `J` of the endeffector pose `e` with respect to the joint
angles `q` from the forward kinematics equation. Afterwards, implement the result in the method 
`jacobian`.

3. Implement the remaining methods up to `computeIK` following the algorithm from slide 15 and
the comments in the code.
Note: In `computeIK`, you have to initialize the joint angles `q`. You can choose the initialization
values arbitrarily with the following restrictions:
  * Do not initialize an angle to 0, as this would lead to a singularity. If you initalize all
    joint angles to 0, all tangents will be vertical, so `de_x/dq_i = 0`.
    In this case, the Jacobian is not invertible and the algorithm will fail.
  * For most goal locations, there exist two solutions. The algorithm will return 
    the solution that is closer to the initialization values. If the initialization 
    values are very far from the solution, it may occur that the algorithm oscillates 
    between the two solutions.
    
Now we would like to have our robot serve a drink. While the robot arm from above can reach any
given point within its range, it cannot control the orientation of the hand, so it would spill out the
drink. Hence, we have to add another link to the arm as shown on the exercise sheet.

With this configuration, the robot can move the glass along a trajectory while keeping the glass
upright. The endeffector pose now consists of the position and orientation of the hand: 
`e := (e_x, e_y, e_theta)^T`. `e_theta` is the orientation of the hand with respect to the ground, 
so `e_theta = 0` means that the hand is horizontal and the glass is upright.

4. Implement the method `InverseKinematics_3Links::forwardKinematic` for the new robot.
The position can be computed as
```
(e_x, e_y, 1)^T = R(q_0) * T(a_0, 0) * R(q_1) * T(a_1, 0) * R(q_2) * T(a_2, 0) * (h, 0, 1)^T
```
Additionally, compute e_theta from q_0, q_1, q_2 and return the complete 
vector `(e_x, e_y, e_θ)`.

5. The Jacobian is now a 3 x 3 matrix.
As deriving the Jacobian for more than two joints is a lot of work, we use a numerical estimation
of the Jacobian instead. The top left component can be esimated using the difference quotient
as shown on the exercise sheet.

In `InverseKinematics_3Links::jacobian()`, calculate the difference quotients with the help
of the `forwardKinematic()` method and fill the Jacobian matrix.

6. Implement `InverseKinematics_3Links::computeIK()` to complete the computation of the
inverse kinematics. 

If you have Gnuplot installed on your computer, you can see an animation of the robot arm serving
a drink by calling `rosrun inverse_kinematics plot.gp`

