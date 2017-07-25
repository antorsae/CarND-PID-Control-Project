# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---
## Implementation

This project consists of implementing a PID controller to control the steering angle of a car in a simulator. The PID implementation is a vanilla PID implementation as per the udacity lessons. The parameters where chosen using the Twiddle algorithm, altough I also tried to choose them using function minimization (which didn't work).

## PID Parameters

The PID controller controls a system using a feedback loop to get the cross-track error (CTE). There are three parameters which govern how the controller attempts to minimize the CTE:

* P: Proportional -> The `P` parameter counteracts the error in the control loop with a signal inversely proportional to the instantaneous CTE. By itself, the `P` has a very obvious effect in our case, a high `P` makes the car steer abruptly to the center of the track but (because of the inertia) it overshoots and creates oscillations. Not a very comfortable car.
* D: Derivative -> The `D` parameter counteracts the error in the control loop with a signal inversely proportional to the rate of change of the CTE. In our case, since we are implementing a digital and sequential PID controller, it we calculate the rate of change of the CTE by substracting `cte` from the current observation and the previous one. The main purpose of the `D` component is to model first-order dynamics in the car, and hence correct the overshooting of having the `P` parameter alone.
* I: Integral -> The `I` parameter models potential bias in the overall loop (from control to sensors) with a signal inversely proportional to the accrued CTE. If we neglect this term the car still completes the lap but I believe it has a tendency to err to the right side. 

## Video

I have been unable to record videos using my Macbook pro with Quicktime b/c it slows down the simulator/PID controller combo and doesn't work.

## Hyperparameter optimization

This was by far the most difficult part of the project. 

My first goal was to decouple the PID controller from the optimization strategy. I made the code multi-threaded so one thread kept running the `uWS` callbacks and another thread runs the optimization algorithm.

I first tried a generic function minimization library `dlib` to minimize the accrued squared CTE accross approximately a single lap. This didn't work, various reasons were involved but since I'm working against the clock to finish this project before the final I opted to implement the `twiddle` algorithm.

I first chose a starting point for the twiddle algorithm for the `P I D` parameters so that the car would complete a lap at a very slow speed (0.1). Then I run run twiddle accross aproximately a full lap for each twiddle iteration and manually increasing the throttle up to 0.3. I also ignore the first 5% of CTE samples (to allow for the effects of the previous iteration to fade).

The final parameters were:

```
 P: 0.211576
 I: 0.0005
 D: 2.57
```

I would have loved to implement a steering PID controller, but did not have time! 

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
