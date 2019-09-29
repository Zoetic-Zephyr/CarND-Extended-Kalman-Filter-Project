### Overview of a Kalman Filter: Initialize, Predict, Update

To review what we learned in the extended Kalman filter lectures, let's discuss the three main steps for programming a Kalman filter:

- **initializing** Kalman filter variables
- **predicting** where our object is going to be after a time step Δ*t*
- **updating** where our object is based on sensor measurements

Then the prediction and update steps repeat themselves in a loop.

To measure how well our Kalman filter performs, we will then calculate **root mean squared error** comparing the Kalman filter results with the provided ground truth.

These three steps (initialize, predict, update) plus calculating RMSE encapsulate the entire extended Kalman filter project.



### Files in the Github src Folder

The files you need to work with are in the `src` folder of the github repository.

- `main.cpp` - communicates with the Term 2 Simulator receiving data measurements, calls a function to run the Kalman filter, calls a function to calculate RMSE
- `FusionEKF.cpp` - initializes the filter, calls the predict function, calls the update function
- `kalman_filter.cpp`- defines the predict function, the update function for lidar, and the update function for radar
- `tools.cpp`- function to calculate RMSE and the Jacobian matrix

The only files you need to modify are `FusionEKF.cpp`, `kalman_filter.cpp`, and `tools.cpp`.



### How the Files Relate to Each Other

Here is a brief overview of what happens when you run the code files:

1. `Main.cpp` reads in the data and sends a sensor measurement to `FusionEKF.cpp`
2. `FusionEKF.cpp` takes the sensor data and initializes variables and updates variables. The Kalman filter equations are not in this file. `FusionEKF.cpp` has a variable called `ekf_`, which is an instance of a `KalmanFilter` class. The `ekf_` will hold the matrix and vector values. You will also use the `ekf_` instance to call the predict and update equations.
3. The `KalmanFilter` class is defined in `kalman_filter.cpp` and `kalman_filter.h`. You will only need to modify 'kalman_filter.cpp', which contains functions for the prediction and update steps.