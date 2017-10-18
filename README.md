# CarND-Extended-Kalman-Filter-Project

#### In this project a Sensor Fusion Model is implemented in C++ to fuse LIDAR and RADAR sensor data into an Extended Kalman Filter for Localization of a moving car.

Kalman Filter is a loop of 2 Major Steps:
1. Predict - Calculated State Vector x_ and State Covariance P_
2. Update - Incorporate the new sensor data to the predicted state, and correcting the x_ and P_.

These Following Steps are Executed in order to Calculate x_ and P_:
1. Calculate dt to accomodate for F matrix.
