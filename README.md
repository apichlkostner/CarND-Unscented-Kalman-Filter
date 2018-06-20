# Unscented Kalman Filter Project

# Summary

This project implements sensorfusion of radar and lidar sensor using an unscented kalman filter.

The steps of this project are the following:

- Implementation of the unscented kalman filter.
- Connection of the kalman filter with the simulator using websockets.
- Running the simulator and process the sensor data sent over the websockets.
- Tracking the object with position and velocity.
- Comparing the result with the ground truth as RSME.
- Checking the results with only one of the sensors used.
- Initialization of the sensor noise parameter with values from the vendor.
- Initialization of the process noise parameter with estimations.
- Calculating the NIS value for different parameters to check if the noise estimation is plausible.
- Comparison with the extended kalman filter

# The filter

## System

The system is modeled as CTRV (constant turn rate and velocity magnitude).

## Radar sensor

Measures position and velocity of the object in polar coordinates. The measurement function is not linear.

## Lidar sensor

Measures only the position of the object. The measurement function is linear.

# Implementation

## Classes

* `UKF`
  * is called by `main()` with new measurements
  * initializes the unscented kalman filter
  * calls the kalman filter for prediction and update steps
* `Logger`
  * logging as csv table
* `Measurement`
  * covariance matrix
  * `LidarMeasurement`
    * measurement matrix for kalman filter
  * `RadarMeasurement`
    * coordinate transformation polar <-> cartesian
    * transforms sigma points to measurement space
* `Tools`
  * calculation of RMSE

## NIS analysis

Reads the log from the UKF and plots the NIS curve:

[analyse/nis_analyse.ipynb](analyse/nis_analyse.ipynb)

# Initialization

From analysing the trajectory of the car it can be found out that there is a maximum acceleration of 1.5 and a maximum yaw rate of 2.5.

So good values could be std_a = 0.7 and std_yawdd = 1.2. After checking different values the final parameters were std_a = 0.7 and std_yawdd = 1.2.

The velocity is initialized with the measured velocity of the first radar measurement since there are no better values available. If the heading of the object was available the real velocity could be calculated.

# Result

## Result with radar and lidar

![](docu/result_radar_lidar.png)

## Result with lidar only

![](docu/result_lidar.png)

## Result with radar only

![](docu/result_radar.png)

# Comparison with the Extended Kalman Filter

The final RMSE with unscented and extended kalman filter are

Filter     | RMSE x | y     | vx    | vy     |
-----------|--------|-------|-------|--------|
Unscented  | 0.064  | 0.084 | 0.238 | 0.215  |
Extended   | 0.096  | 0.084 | 0.390 | 0.423  |

With the unscented kalman filter and the CTRV system model the position in x direction and the velocity is much better tracked.

Since both system model and filter type are changed together, the performance of the UKF can't be rated against the EKF.


# NIS

Using the Normalized Innovation Squared (NIS) it can be checked if the noise parameters are plausible. For every filter step the NIS value is calculated and written to a file. Then a scatter plot of the values is drawn. Since the NIS values follow a X² distribution the corresponding value for the lower 95% is drawn additionally. Then about 5% of all plotted values should be above the line.

In the table the values calles "nis radar" and "nis lidar" are the fraction of nis values which are above the line. Ideally the value should be 5% = 0.05.

std_a   | std_yawdd | RMSE x | y     | vx     | vy     | nis radar | nis lidar
--------|-----------|--------|-------|--------|--------|-----------|-----------
0.8     | 1.0       | 0.0626 | 0.0832| 0.1650 | 0.2151 | 0.036     | 0.020     
10.0    | 1.0       | 0.0843 | 0.1044| 0.5895 | 0.8717 | 0.044     | 0.036     
10.0    | 10.0      | 0.0907 | 0.1042| 0.8346 | 0.8443 | 0.028     | 0.016     
0.1     | 0.1       | 0.1255 | 0.1353| 0.2996 | 0.3152 | 0.124     | 0.137     
0.05    | 0.05      | 0.1255 | 0.1353| 0.2996 | 0.3152 | 0.253     | 0.373     

The plot for std_a = 0.8 and std_yawdd = 1.0:

![](docu/nis_0p8_1p0.png)

The plot for std_a = 0.05 and std_yawdd = 0.05:

![](docu/nis_0p05_0p05.png)

# Dependencies
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF` Previous versions use i/o from text files.  The current state uses i/o
from the simulator.