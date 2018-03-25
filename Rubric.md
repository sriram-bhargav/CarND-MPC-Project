## The Model
We use kinematic model as vehicle model. Take a look below for state model used to track current state and predict future state for the vehicle in simulation:
```
1. Position in 'x' and y'
2. Heading angle (psi)
3. Velocity (v)
4. Cross-track error (CTE)
5. Heading angle error (epsi).
```

Actuator values are the acceleration (a) - which is controlled by throttle, and the steering angle (steering_angle/delta).

Update equations for the kinematic model:
<img src="./update_equations.png" width="425"/>

## Timestep Length and Elapsed Duration (N & dt)
N = 10 and dt = 0.1 are the final chosen values for this project after some trial and error (for reference velocity = 70mph).

Previous values tried (for reference velocity = 70mph):
N = 10 and dt = 0.05 - creates zigzag behavior because of small dt
N = 20 and dt = 0.05 - creates zigzag behavior because of small dt
N = 20 and dt = 0.1 - larger N seems unnecessary

## Polynomial Fitting and Preprocessing
The waypoints are converted into vehicle coordinate system before polynomial fitting:
```
// Convert track points to car coordinates.
Eigen::VectorXd waypoints_x(ptsx.size());
Eigen::VectorXd waypoints_y(ptsy.size());

for (int i = 0; i < ptsx.size(); ++i) {
  double dx = ptsx[i] - px;
  double dy = ptsy[i] - py;
  waypoints_x[i] = dx*cos(-psi) - dy*sin(-psi);
  waypoints_y[i] = dx*sin(-psi) + dy*cos(-psi);
}

// Fit waypoints with 3rd polynomial.
auto coeffs = polyfit(waypoints_x, waypoints_y, 3);
```

## Model Predictive Control with Latency
For this project, we assumed that there is 100ms delay in actuator reaction. This is how I dealt with latency:

```
// 100 ms time delay for actuations.
double latency = 0.1;
double Lf = 2.67;

// Calculate future state.
double x_next = v*latency;
double y_next = 0;
double psi_next = v*(-1)*steer_value/Lf*latency;
double v_next = v + throttle_value*latency;
double cte_next = cte + v*sin(epsi)*latency;
double epsi_next = epsi + psi_next;

// Finally update state accordindly.
Eigen::VectorXd state(6);
state << x_next, y_next, psi_next, v_next, cte_next, epsi_next;
```

## Simulation
https://youtu.be/55SfWysRHko (Vehicle driving smoothly at ~ 65mph on the track with our vehicle model).
