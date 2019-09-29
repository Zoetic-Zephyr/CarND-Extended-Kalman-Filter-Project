Let's discuss the three files that you will need to modify.



### FusionEKF.cpp

In `FusionEKF.cpp`, we have given some starter code for implementing sensor fusion. In this file, you won't need to include the actual Kalman filter equations; instead, you will be initializing variables, initializing the Kalman filters, and then calling functions that implement the prediction step or update step. You will see `TODO` comments indicating where to put your code.

You will need to:

1. initialize variables and matrices (x, F, H_laser, H_jacobian, P, etc.)
2. initialize the Kalman filter position vector with the first sensor measurements
3. modify the F and Q matrices prior to the prediction step based on the elapsed time between measurements
4. call the update step for either the lidar or radar sensor measurement. Because the update step for lidar and radar are slightly different, there are different functions for updating lidar and radar.

#### Initializing Variables in FusionEKF.cpp

```c++
  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
```

Every time `main.cpp` calls `fusionEKF.ProcessMeasurement(measurement_pack_list[k])`, the code in `FusionEKF.cpp` will run. - If this is the first measurement, the Kalman filter will try to initialize the object's location with the sensor measurement.

#### Initializing the Kalman Filter in FusionEKF.cpp

```c++
  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
```

#### Predict and Update Steps in FusionEKF.cpp

Once the Kalman filter gets initialized, the next iterations of the for loop will call the `ProcessMeasurement()` function to do the predict and update steps.

```c++
  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO: Use the sensor type to perform the update step.
   * TODO: Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates

  } else {
    // TODO: Laser updates

  }
```

In `FusionEKF.cpp`, you will see references to a variable called `ekf_`. The `ekf_` variable is an instance of the `KalmanFilter` class. You will use `ekf_` to store your Kalman filter variables (x, P, F, H, R, Q) and call the predict and update functions. Let's talk more about the KalmanFilter class.



### KalmanFilter Class

`kalman_filter.h` defines the `KalmanFilter` class containing the x vector as well as the P, F, Q, H and R matrices. The KalmanFilter class also contains functions for the prediction step as well as the Kalman filter update step (lidar) and extended Kalman filter update step (radar).

You will need to add your code to `kalman_filter.cpp` to implement the prediction and update equations. You do not need to modify 'kalman_filter.h'.

Because lidar uses linear equations, the update step will use the basic Kalman filter equations. On the other hand, radar uses non-linear equations, so the update step involves linearizing the equations with the Jacobian matrix. The `Update` function will use the standard Kalman filter equations. The `UpdateEKF` will use the extended Kalman filter equations:

```c++
void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
}
```



### Tools.cpp

This file is relatively straight forward. You will implement functions to calculate root mean squared error and the Jacobian matrix:

```c++
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO: Calculate a Jacobian here.
   */
}
```

HINT: You implemented these already in the coding quizzes.



### Compiling and Running Your Code

Take a look at the github repo [README file](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project/blob/master/README.md) for instructions on how to compile and run your code.