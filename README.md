# CarND-Extended-Kalman-Filter-Project

#### In this project a Sensor Fusion Model is implemented in C++ to fuse LIDAR and RADAR sensor data into an Extended Kalman Filter for Localization of a moving car.

Kalman Filter is a loop of 2 Major Steps:
1. Predict - Calculated State Vector x_ and State Covariance P_
2. Update - Incorporate the new sensor data to the predicted state, and correcting the x_ and P_.

### These Following Steps are Executed in order to Calculate x_ and P_ upon a sensor measurment:
1. Determine ***dt*** to further calculate the ***F*** and ***Q*** matrices.
2. Predict *State Vector **x*** to determine x: position, velocity - P: State Covariance
      * LIDAR and RADAR prediction functions are the same.

3. Determine whether the detected measurment is from a RADAR or LIDAR sensor:
    1. RADAR - **Using Extended Kalman Filter Equations**:
    
     * Convert Input Polar Coordinates to Cartesian using the ***h*** function to calculate Error ***y***.
     * Calculate Jacobian Matrix ***H*** to calculate the Kalman Gain ***K*** while maintaining linearity, including the ***R*** measurement noise covariance matrix.
     * Use calculated ***y*** and ***K*** to correct for the predicted ***x*** and ***P***.
   
   2. LIDAR - **Using Standard Kalman Filter Equations**:
    * Directly Calculate the Error ***y***
    * Calculate the Kalman Gain Matrix ***K***, including the ***R*** measurement noise covariance matrix.
    * Use calculated ***y*** and ***K*** to correct for the predicted ***x*** and ***P***. 

---

*For More Information check the [source files](./src) to review the EKF algorithm. 
