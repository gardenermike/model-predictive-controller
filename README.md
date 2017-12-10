# Model Predictive Controller
Udacity Self-Driving Car Engineer Nanodegree Program

![Simulator](./images/loop-small.gif "Controller in action")

---

This project uses a [Model Predictive Controller](https://en.wikipedia.org/wiki/Model_predictive_control) to control the steering and throttle of a moving vehicle in a simulator.
Model predictive control uses a predicted trajectory over a future timeframe to optimize immediate action. At each time interval, the current state of the system (in this case, the position, velocity, and orientation of the vehicle in relation to a series of waypoints on a track) is used to plot the best trajectory to follow several time steps into the future. The controls (here, steering and throttle) are actuated to follow that trajectory. The trajectory is then discarded, as it has served its purpose. At the next time step, the same process will be followed again. The process of control, then, involves continuously predicting the best trajectory at the moment, optimizing based on the current state.
Model predictive control can deal with latency and can be combined with a sophisticated model, including [deep neural networks](http://deepmpc.cs.cornell.edu/), to address much more sophisticated problems than a [PID controller](https://github.com/gardenermike/pid-controller).

The simulator for this project is part of the Udacity Self-Driving Car Nanodegree program. All of the support code around the controller and much of the structure in the controller code also came from Udacity.

The simulator can be downloaded from the [Udacity github repository](https://github.com/udacity/self-driving-car-sim/releases).

There are number of dependencies. Please see the [Udacity github repository](https://github.com/udacity/CarND-MPC-Project) for build instructions.

## Files
[MPC.cpp](https://github.com/gardenermike/model-predictive-controller/blob/master/src/MPC.cpp) implements the controller. It contains two classes, FG_eval, which builds the parameters for the solver which optimizes the trajectory, and MPC, which extracts the state information and is the public entry point to the controller.

[main.cpp](https://github.com/gardenermike/model-predictive-controller/blob/master/src/main.cpp) explicitly adds latency to the connection with the simulator,  and models that latency to work around it. It passes state to the controller with each message from the web sockets connection with the simulator.


## Discussion

### Model
The state that is modeled by the controller in this project consists of six elements:
* The X position of the vehicle. To simplify the math, I [transform the map coordinates to vehicle coordinates](https://github.com/gardenermike/model-predictive-controller/blob/master/src/main.cpp#L106), which puts the vehicle at the origin of the coordinate system. In this translated coordinate system, X is always 0.
* The Y position of the vehicle. Like the X position, the Y coordinate is always 0 from the perspective of the vehicle.
* The orientation of the vehicle, measured as an angle in radians, and signified by the Greek letter "Ψ", or "psi" in English. Like the X and Y, from the vehicle perspective, its angle is always 0.
* The velocity of the vehicle.
* The cross-track error ("cte" in the code). This is a simple linear distance away from the center of the track.
* The error in orientation bearing. In the code this is represented as the error in psi, "epsi".

The output of the controller has two elements:
* The steering angle, thought of as the delta psi. In the code this is generally called "delta".
* The acceleration, or throttle/brake, called "a" in the code.

The trajectory into the future is modeled with [six](https://github.com/gardenermike/model-predictive-controller/blob/master/src/MPC.cpp#L8) points, each one-tenth of a second apart. This is a relatively small. I tried as few as four and as many as ten. With too few points, the trajectory did not take the future sufficiently into account, and corners might be handled poorly. With too long of a time horizon, the trajectory would extend beyond the furthest waypoints, where the polynomial fitting the waypoints might diverged sharply from the real world. The tenth of a second I chose between points was chosen with similar criteria: I wanted points close enough for a smooth fit, and not so far as to extend beyond the waypoints at any velocity.

The [cost values](https://github.com/gardenermike/model-predictive-controller/blob/master/src/MPC.cpp#L51) used to determine the trajectory were hand-tuned to ensure a smooth drive. The cost consists of several components:
* The cross-track error
* The error in orientation (weighted much more heavily than the cross-track error)
* The distance from the reference velocity
* A penalty for high steering angle (heavily weighted)
* A penalty for high acceleration
* A penalty for sudden changes in steering. I weighted this part of the cost most heavily, to ensure smooth steering.
* A penalty for sudden changes in acceleration.

Each of the cost elements was squared to ensure no negative values and to penalize outliers.

The solver was instructed to drive all costs as close as possible to zero.

To forecast the six points into the future, I used [equations model the change between each time step](https://github.com/gardenermike/model-predictive-controller/blob/master/src/MPC.cpp#L106).
The X and Y values simply follow the sides of a triangle extrapolated outward by the velocity times the change in time (a tenth of a second as outlined above).
The forecast orientation uses the turning radius of the vehicle and the steering angle, together with the velocity to predict a change in vehicle angle.
The change in velocity simply adds the expected change given the current acceleration.
The cross-track error uses the linear distance between the Y values of the predicted point and the waypoint polynomial, subtracting the expected change in Y.
The expected error in angle/orientation is the most complicated calculation. It assumes motion along the tangent line of the current waypoint polynomial.

### Latency
To better approximate real-world conditions, the event handler listening to the simulator [explicitly sleeps for 100 milliseconds](https://github.com/gardenermike/model-predictive-controller/blob/master/src/main.cpp#L179).
When this latency is ignored, the model experiences increasing errors at higher velocities, leading to out-of-control oscillation.
I use a [simple solution to address latency](https://github.com/gardenermike/model-predictive-controller/blob/master/src/main.cpp#L97).
I assume a constant velocity, as acceleration within a 0.1-second interval can be assumed to be negligible.
Using the constant velocity assumption, I project out a triangle along the current orientation, using the velocity and delta time determine the length. I just use the side lengths of the triangle to find my new X and Y.
I also project the change in bearing based on the current steering angle, using the turning radius of the vehicle figure out the delta.

### Improvements
The MPC controller was able to far outperform previous work I've done in the simulator, including a machine-learned [behavioral cloning](https://github.com/gardenermike/behavioral-cloning) controller and a [PID](https://github.com/gardenermike/pid-controller) controller.
However, at increasing velocity, problems start showing up. At around 90 mph, the driving becomes dangerous.
One problem is imperfect modeling of latency. No adjustment is made to account for changes in latency caused by variation in processing time. I also use a simple model to project the future position based on the latency. A more sophisticated model could improve performance.
I could also penalize velocity during turning, which is risky.
My emphasis on smooth turns also leads to the vehicle cutting corners. At higher speeds, this becomes increasingly dangerous.

### Tweaks
The easiest things to change are the reference velocity and number of time steps in the forecast. If you'd like to play with the code, that is good place to start. Adjusting the weights given to the various cost components also is a fine-tuning step that could be taken further.

### Video
A recording of the loop is in this repo in [gif](./images/loop.gif) or [mov](./images/loop.mov) format.
