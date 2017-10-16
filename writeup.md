## MPC Project

---

### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---

### The Model

#### Student describes their model in detail. This includes the state, actuators and update equations.

I used a Kinematic Model that used a state with x and y coordinates for the location, an angle psi for the vehicle angle, and v for the velocity. For the actuators, I used a steering angle (delta) and throttle (a for acceleration).
When updating I used the following equations which came into play when defining constraints and accounting for latency (where Lf is the length from the front of the vehicle to its center of gravity):

x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt

y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt

psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt

v_[t+1] = v[t] + a[t] * dt

cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt

epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt


### Timestep Length and Elapsed Duration (N & dt)

#### Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

For N I wanted to get a few points out into the future so that I could predict where I should be heading, but I didn't want too choose too many points so that points that were too far away would weight too much on the actuations I would choose to follow the path. I started with 20, then tried 15, and ultimately settled on 10.
In choosing a value dt, I wanted dt to be short enough so that the model could be reponsive, but not so short that it would be too computationally intense. I originally tried 0.05, but setting on 0.1.

The values of 10 for N and 0.1 for dt gave me a smooth enough result that I left them with those values.


### Polynomial Fitting and MPC Preprocessing

#### If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.

Prior to solving using MPC, I accounted for latency (as described in the next section) and I also transformed the coordinates from map to car coordinates using the following equations for each waypoint:

x = ptsx[i] - px

y = ptsy[i] - py

car_x = x * cos(psi) - y * sin(psi)

car_y = x * sin(psi) + y * cos(psi)


### Model Predictive Control with Latency

#### Student provides details on how they deal with latency.

To account for a latency of 100ms (0.1s) I set a variable, latency=0.1 and used the following equations from the model substituting the value for latency in for dt to predict the state 0.1s into the future:

px = px + v * cos(psi) * latency

py = py + v * sin(psi) * latency

psi = psi - v * delta / Lf * latency
          
v = v + a * latency

