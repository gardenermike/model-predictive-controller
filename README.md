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


