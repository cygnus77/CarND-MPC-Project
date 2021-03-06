## Model Predictive Controller

### System overview

Udacity self-driving car simulator fires telemetry calls to the MPC program with the following inputs:
 - Current position, orientation and velocity of the vehicle within a map.
 - Current actuator state: steering wheel angle and throttle value
 - Waypoints (x,y) coordinates in map. The waypoints are a trajectory for the car to follow. In a real-world implmentation this could be a 3rd degree polynomial generated by the path planning module.

The MPC program must return a new steering angle and throttle value with the assumption that there is a 100ms latency for actuator inputs to take effect.
The goal is to maximize speed while keeping vehicle safe and close to the intended trajectory.

### The Model

The state of the vehicle is represented by coordinates: x, y, orientation: ψ and velocity: v.

Kinematic model is used to predict vehicle motion using the following formulae:

- x_(t+1) = x + v * cos⁡(ψ) * dt
- y_(t+1) = y + v * sin⁡(ψ) * dt
- ψ_(t+1) = ψ + v/L_f * δ * dt
- v_(t+1) = v + a * dt

Where
- δ: steering angle
- a: accelaration (throttle)
- dt: time step length in seconds
- L_f: distance from front of vehicle to center-of-gravity (constant provided for the vehicle)

### Timestep Length and Elapsed Duration (N & dt)

MPC predicts the vehicle trajectory for `N` timesteps, each `dt` seconds apart. MPC optimizer selectes actuator values that minimizes the error between predicted trajectory and desired trajectory over the N timesteps.

Since at each iteration, we only select the first set of actuator values, computing a very large trajectory (N > 20) is wasteful. However too short a trajectory might miss planning for a sharp curve that the car is just about to encounter.

Likewise, a short value of dt will effetively reduce the trajectory length, whereas a very large dt value will prevent the car from reacting to changes quickly.

At faster speeds, dt value has to be smaller, as the trajectory will grow longer because of more ground being covered in a shorter time span.

I set a target speed of more than 80mph and after some experimentation, I selected N = 10 and dt = 0.05.


### Polynomial Fitting and MPC Preprocessing

Since the simulator provides waypoints instead of a polynomial (as a path-planning module would), we would have to compute one. The first step is to convert coordinates and angles into a uniform coordinate space that is relative to the vehicle's current position and angle, called `Vehicle Coordinate Space`.

Waypoints are converted to vehicle space by tranlating them to vehicle origin and rotating them to vehicle's orientation.
Then we fit a 3rd degree polynomial to the waypoints and obtain a set of coefficients that describe it. This polynomial describes the path the car should aim to be on.

Next, we predict the vehicle's location on the map after 100ms using kinematic model equations and values of current throttle, orientation and steering angle. This location is converted to vehicle space and used as the starting point for planning the next action.

#### CTE (`cte`)
Based on the polynomial, we can calculate the cross-track error of the current position. To do that, we evaluate the polynomial at the car's current x coordinate, obtaining the expexceted y coordinate value. The difference between expected y and actual y is a measure of the CTE.

- cte_t = f(x_t) − y_t

#### Orientation error (`eψ`)

Error in vehicle orientation is calculated by comparing the slope of the polynomial (computed using the derivative of the polynomial) at x and the current orientation:

- eψ_t= ψ_t − ψdes_t
- eψ_(t+1) = eψ + v_t/L_f * δ_t * dt

The CTE and orientation error complete the state of the vehicle used in solving process:

State of vehicle: `[x, y, ψ, v, cte, eψ]`

### Cost function and Solving

A `Cost` function is defined to compute the cost of a given state `[x, y, ψ, v, cte, eψ]`. There are several factors that are taken into account while computing the cost:

- Summation of squares of cte, eψ, and v−speedlimit. This makes cost proportional to how far the car strays from the desired path.
- Penalize large actuator values (δ and a)
- Penalize large changes to actuators (δ_(t+1) − δ_t  & a_(t+1) − a_t)

I had to scale up (by a factor of 1000) the cost of large steering actuator values to ensure stability at higher speeds.

Further, I set the target speed of the car based on the curvature of the projected trajectory. Sharper the curve, slower the car would drive; straighter the trajectory, faster the car would attempt to go.

A `Solve` function uses the IPOPT (Interior Point OPTimizer) package to find the set of actuator values that minimizes the cost while keeping within defined constraints.

The solve function computes the cost for the N steps in the trajectory at each timestep dt, by invoking the cost function multiple times.

Finally, the best actuator values are converted back to ranges accepted by the simulator and returned as telemetry output.


### Video of this MPC controller is posted here 

[You tube video](https://youtu.be/PKcjEsma77M)

#### Note:

* Hits 105mph while breaking on sharp curves.
* If breaking is taken out, max speed hits 110mph, but unsafe.


---


## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Build Instructions
1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

