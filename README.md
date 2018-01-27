# **Project: Model Predictive Control**
[![Udacity - Self-Driving Car Engineer NanoDegree Program](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)


## Overview
This repository contains all the codes and reflection that required for completing project. The goal of this project is to implement model predictive control to drive the car around the track.

Build instruction and dependency of this project is identical to [upstream repository](https://github.com/udacity/CarND-MPC-Project) which is the starter code repository of udacity.

## Implementation

I followed criteria of rubrics.

### The model
In this project, I used kinematic model which is mentioned in the lecture. The model is relatively simple compared to dynamic model which includes slip, tire model and so on.

The state has vehicle's position on the map (x, y), orientation (psi), velocity (psi), cte (cross track error) and psi error (epsi). Actuators are steering angle (delta) and acceleration (a).

With following update equations, we can predict future state of vehicle.
```
x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
v_[t+1] = v[t] + a[t] * dt
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```
where `dt` is time difference between future and current state of vehicle and `Lf` is the distance between the front of the vehicle and its center of gravity.

### Time-step length and elapsed duration (N & dt)
I started with (20, 0.05) which is used in quizzes in the lecture without latency. After adding latency, they needed to change since it keeps overshooting. After number of tunning N & dt. I sticked with `N = 10` and `dt = 0.1`. when I decreased `dt` below 0.1, the vehicle oscillated more. If I increase `dt` above 0.1, The green line (prediction line) get completely weired when car meets stiff corners.

### Polynomial fitting and MPC preprocessing
I used third-order polynomial for fitting waypoints. Before the fitting polynomials, coordinate transformation is needed since simulator's points are global position. The positions given by simulator are transformed to vehicle coordinate system. I wrote the function `transformCoordiate()` which transforms coordination from global to vehicle's view. It returns 2 by number of points matrix, first row for x positions and second row for y positions of vehicle. Line 131 to 136 of `main.cpp` shows fitting polynomial and transforming coordinate system. 

Since sign of angle of simulator is opposite value of our model, I multiply -1 to `psi` and `delta`. Also, right before sending `steering_angle` back to simulator, I changed the sign of the steer value.
### Model predictive control with latency
Actuator latency is used for predicting initial state of the vehicle with update equations. After transforming coordinate system, I updated state of the vehicle with latency. Codes are found in line 154 to 164 of `main.cpp`.
```C++
const double Lf = 2.67;
px = px + v * cos(psi) * latency;
py = py + v * sin(psi) * latency;
psi = psi + v * delta / Lf * latency;
v = v + a * latency;

double cte = polyeval(coeffs, px) - py;
double epsi = psi - atan(coeffs[1] + 2 * coeffs[2] * px + 3 * coeffs[3] * px * px);

Eigen::VectorXd state(6);
state << px, py, psi, v, cte, epsi;
```
I measured actual latency which includes computational time for MPC and 100 ms latency of actuator with `std::chrono::steady_clock::now()`. Since MPC processing time varies with system specification, I used measured latency for predicting initial state of the vehicle if previous measured latency larger than 100 ms. 
After updating initial state of vehicle state with above codes, the MPC procedure is applied.

### Cost Function parameters
For smooth steering, I tuned the cost function that affects steering. In line 74 of `MPC.cpp`, Multiplying that part by 5000 influenced the solver into keeping sequential steering values closer together.
Also, increasing weight for the orientation error as line 62 of `MPC.cpp` helped to keep the car pointing in the direction of the road.
I could get more stable trajectories.

## Result
I achieved about 68 MPH which is much higher than what I achieved with PID controller. Here is [youtube link](https://youtu.be/GGtviEpnJlM) of recording two rounds of given track.

## References
 - Udacity discussion forum - [How to incorporate latency into the model](https://discussions.udacity.com/t/how-to-incorporate-latency-into-the-model/257391)
 - [Slack channel of this project](https://carnd.slack.com/messages/C54DV4BK6)
