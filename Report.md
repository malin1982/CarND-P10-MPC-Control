#Model Predictive Control Project
##Description
In this project you'll implement Model Predictive Control to drive the car around the track. This time however you're not given the cross track error, you'll have to calculate that yourself! Additionally, there's a 100 millisecond latency between actuations commands on top of the connection latency.
##The Vehicle Model
A simple kinematic model is used for this project. The model takes into account speed, orientation, but ignores dynamic forces such as gravity, air resistance, drag mass and geometry of the vehicle. The state of the model is: position X, Y coordinates, orientation angle Psi, and velocity V.

state: [x, y, psi, v]

The actuators are the steering wheel delta and the throttle a (control both acceleration and deceleration).

actuators: [delta, a]

The state equations describe how the next state is calculated, from the current state over a period of dt.

x[t+1]    = x[t] + v[t] * cos(psi[t]) * dt
y[t+1]    = y[t] + v[t] * sin(psi[t]) * dt
psi[t+1]  = psi[t] + v[t] / Lf * delta[t] * dt
v[t+1]    = v[t] + a[t] * dt

Lf is the distance between the vehicle's center of gravity and its front wheel, which represent the vehicle's maneuverability.

MPC tracks two errors: Cross Track Error (CTE) and Orientation Error (Epsi).

cte[t+1]   = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
epsi[t+1]  = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt

##MPC Implementation
Sample code from the MPC Quizzes was used to implement MPC. Cost functions were manually re-factored.

The cost function includes:
1.referencing cross track error, orientation error and velocity error.
2.control inputs, penalize the magnitude of the inputs as well as the change-rate.

for(int t = 0; t < N; t++) {
  fg[0] += 8.0 * CppAD::pow(vars[cte_start + t] - ref_cte, 2);   // cross track error
  fg[0] += 0.5 * CppAD::pow(vars[epsi_start + t] - ref_epsi, 2); // orientation error
  fg[0] += 0.5 * CppAD::pow(vars[v_start + t] - ref_v, 2);       // velocity error
}
// cost based on actuators (steering angle & acceleration)
for(int t = 0; t < N-1; t++) {
  fg[0] += 30000.0 * CppAD::pow(vars[delta_start + t], 2); // steering angle error
  fg[0] += 10.0 * CppAD::pow(vars[a_start + t], 2);        // acceleration/throttle error
}

// minimize the value gap between sequential actuations
for (int t = 0; t < N - 2; t++) {
  fg[0] += 2500.0 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
  fg[0] += 0.1 * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
}

Tuning the weight of difference cost components ensures better vehicle handling.

##Time Step and Duration
MPC creates a predicted trajectory path T seconds in front the vehicle's current position. T is defined as number of timesteps N times timeperiod delta dt.
Larger T means longer trajectory path and smoother changes over time. Smaller T means shorter trajectory path, faster response and better accuracy. When using a referencing speed of 40 mph, I found N equals 10 and dt equals 0.05 gives the best performance. However increasing the speed will cause the vehicle to oscillate on the track. In the end I was able to bump up the referencing speed to 65 mph, setting N to 20 and dt to 0.05.
