# Exercise 3: Odometry calibration with least squares

Mobile robots typically execute motion commands only inaccurately due to
slippage on the ground, uneven terrain, or hardware issues. For example,
if one of the knee joints of a humanoid robot is weaker than the other one,
then it will walk on a circular trajectory although it intends to walk
straight ahead. In order to account for such systematic errors, odometry 
calibration can be applied to learn and correct the drift.

In the lecture a system was introduced for calibrating odometry using a 
least squares approach.

The repository contains the following files:
  * `03_odometry_calibration/data/calib.dat` contains odometry
data recorded by a real robot. The first three columns contain the odometry data
(`u*_x`, `u*_y`, `u*_theta`) measured with an external
tracking system and the last three columns contain the odometry (`u_x`, `u_y`,
`u_theta`) measured by the robot.
`theta` is measured in radians counterclockwise.
  * `03_odometry_calibration/src/OdometryCalibration.cpp` contains
the functions that you have to implement.
  * `03_odometry_calibration/include/odometry_calibration/CalibrationData.h`
contains the data structures for the odometry, calibration data, and 2D poses.


The existing code already loads the data file into the `MeasurementData`
data structure (i.e., both the ground truth odometry data observed externally, 
and the robot's observed odometry). This is done in the main function.

**Exercise steps:**


1. Implement `errorFunction(groundTruth,observation,calibrationMatrix)`, which 
calcuates the error `e_i(x)` between the ground truth odometry `u*_i` and 
the corrected observed odometry (which is corrected by the current estimate of the 
calibration matrix).
2. Calculate the Jacobian `J_i` of the error function for a given odometry measurement by 
implementing `jacobian(observation)`.
3. Calculate the calibration matrix by implementing `calibrateOdometry(measurements)`.
The function uses the loaded data passed as a parameter and should return the calibration matrix. 
In our case, the weight matrix Omega_i is the identity matrix. 
One iteration is sufficient, because the error function is linear.
4. Use the calibration matrix (computed in the previous step) 
to correct the robot's odometry observations in
`applyOdometryCorrection(uncalibratedOdometry,calibrationMatrix)`.
5. Compute the robot's trajectory based on the corrected odometry observations 
by implementing `calculateTrajectory(calibratedOdometry)`:
    * Assume that the robot starts at the position `(x,y,theta)=(0,0,0)`.
    * Transform the robot's pose using the corrected odometry 
      by implementing `odometryToAffineTransformation(odometry)` 
      (Check the lecture slides 34--35 for more information about affine transformations).
    * Chain the affine transformation to get the next pose. (See lecture slides 36--38).
    * Convert the chained affine transformation back to a robot pose 
      by implementing `affineTransformationToPose(transformation)`.
    * Store the pose in the trajectory vector.


When you push your code to the server, it will plot the obtained trajectory in blue.
Compare it to the ground truth trajectory (in red). The trajectories should match
approximately, but there will still be a small error accumulating over time.
