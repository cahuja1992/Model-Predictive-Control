# CarND-Controls-Model Predictive Controller

## Background and Purpose
1. MPC is an optimization approach used for actuation of autonomoud vehicles. 
2. It uses a mathematical dynamics process model to predict and optimize the future behavior of systems [1]. 
3. This implementation was built in C++ and tries to calculate an optimal trajectory for the car simulator to follow around the track. 

### Model Description 
Kinetic model has been implemented in the following approach, which is a simplification of a dynamic vehicle model. It uses model state and output from each time-step to calculate the next, as well as plot a trajectory. 

Model inputs : 
1. Location of Vehicle (x,y)
2. Orientation (psi) 
3. velocity (v)
4. cross track error (cte)
5. error of psi, as well as acceleration (a) and steering angle (delta). 
6. Acceleration and steering angle are used as actuators to propel and guide the car. 

### Timestep Length and Elapsed Duration Rationale
N = 10 
dt = 0.1
Where, number of time steps (N) and duration of a time-step (dt), and together they define the duration of time where the model can predict the trajectory. 

<pre>
    //one second into the future
    size_t N = 10;
    double dt = .1;
</pre>

These values seemed to work very well, so I never experimented with them. 

### Polynomial Fitting and MPC Preprocessing Discussion 
At each step, a bit of per-processing was performed on the car's location data (waypoints) before a polynomial was calculated. First the values of x,y,and psi were normalized, in order to simplify the math in the polynomial calculation. Next the x and y vectors had to be cast into VectorXd collection, so they could be used in the provided polynomial method (polyfit()). 

<pre>
     //convert vector double to VectorXd for polyfit function
      Eigen::Map<Eigen::VectorXd> waypoints_x_eig(ptrx, 6);
      Eigen::Map<Eigen::VectorXd> waypoints_y_eig(ptry, 6);
</pre>
 
 Polyfit() returns the coefficients of a third order polynomial. 
 
 <pre>
     auto coeffs = polyfit(waypoints_x_eig, waypoints_y_eig, 3);
 </pre> 
 
 
### MPC Latency
The purpose of latency in the model is to simulate real world driving conditions where the car does respond to commands instantly, as well as the latency between getting sensor data and processing it. This value was left at the recommended 100 ms. 

### Other Findings - Parameter Tuning - Conclusion.  
By tweaking the cost functions and increasing the velocity parameter around 200 the car was able to go around 90MPH.