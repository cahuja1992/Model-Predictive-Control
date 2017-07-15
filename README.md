# CarND-Controls-Model Predictive Controller

### Model Description and Background
1. MPC is an optimization approach used for actuation of autonomoud vehicles. 
2. It uses a mathematical dynamics process model to predict and optimize the future behavior of systems [1]. 
3. This implementation was built in C++ and tries to calculate an optimal trajectory for the car simulator to follow around the track. 

Following equations are used to update the mdoel
<br>
<img src="https://github.com/cahuja1992/Model-Predictive-Control/blob/master/images/update.png"/>
<br>

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

<br/>
Where, number of time steps (N) and duration of a time-step (dt), and together they define the duration of time where the model can predict the trajectory. 
1. N was determined empirically as a good balance between effective planning and route stability. Large value of N cause wobbling of the car.
2. dt is tunned with reference to the latency of the system

### Polynomial Fitting and MPC Preprocessing Discussion 
The car waypoints are given in local reference frame and were converted to global reference frame for which the above equations are used and in each step also includes the following steps also:-
- Values of x,y,and psi were normalized, in order to simplify the math in the polynomial calculation. 
- x and y vectors had to be converted into VectorXd collection, so they could be used in the provided polynomial method (polyfit()), using the following method:-
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
The purpose of latency in the model is to simulate real world driving conditions where the car does respond to commands instantly, as well as the latency between getting sensor data and processing it.
A delay of 100ms is introduced before the actuations are sent back to the simulator as improper delays can introduce oscillations and bad trajectories and such delays make the control problem a so-called sampled NMPC problem.

1. Trospective position of the car is estimated based on its current speed and heading direction: By propagating the position of the car forward until the expected time when actuations are expected to have an effect. The NMPC trajectory is then determined by solving the control problem starting from that position.

2. Control problem is solved from the current position and time onwards: Latency is taken into account by constraining the controls to the values of the previous iteration for the duration of the latency. Thus the optimal trajectory is computed starting from the time after the latency period. This has the advantage that the dynamics during the latency period is still calculated according to the vehicle model.

### Other Findings - Parameter Tuning - Conclusion.  
By tweaking the cost functions and increasing the velocity parameter around 200 the car was able to go around 90MPH.
