Here we will discuss the `main.cpp` file. Although you will not need to modify this file, the project is easier to implement once you understand what the file is doing. As a suggestion, open the github repository for the project and look at the code files simultaneously with this lecture slide.



### Main.cpp

You do not need to modify the `main.cpp`, but let's discuss what the file does.

The **Term 2 simulator** is a client, and the c++ program software is a web server.

We already discussed how `main.cpp` reads in the sensor data. Recall that `main.cpp` reads in the sensor data line by line from the client and stores the data into a measurement object that it passes to the Kalman filter for processing. Also a ground truth list and an estimation list are used for tracking RMSE.

`main.cpp` is made up of several functions within `main()`, these all handle the uWebsocketIO communication between the simulator and it's self.

Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.

```c++
INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]
```

All the main code loops in `h.onMessage()`, to have access to intial variables that we created at the beginning of `main()`, we pass pointers as arguments into the header of `h.onMessage()`.

For example:

```c++
h.onMessage([&fusionEKF,&tools,&estimations,&ground_truth]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
               uWS::OpCode opCode)
```

The rest of the arguments in `h.onMessage` are used to set up the server.



```c++
 // Create a Fusion EKF instance
  FusionEKF fusionEKF;

  // used to compute the RMSE later
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  //Call the EKF-based fusion
  fusionEKF.ProcessMeasurement(meas_package); 
```



The code is:

- creating an instance of the `FusionEKF` class
- Receiving the measurement data calling the `ProcessMeasurement()` function. `ProcessMeasurement()` is responsible for the initialization of the Kalman filter as well as calling the prediction and update steps of the Kalman filter. You will be implementing the `ProcessMeasurement()` function in `FusionEKF.cpp`:

Finally,

The rest of `main.cpp` will output the following results to the simulator:

- estimation position
- calculated RMSE

`main.cpp` will call a function to calculate root mean squared error:

```c++
// compute the accuracy (RMSE)
Tools tools;
cout << "Accuracy - RMSE:" << endl << tools.CalculateRMSE(estimations, ground_truth) << endl;
```

You will implement an RMSE function in the `tools.cpp` file.

