# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

Project to drive a car (steering angle and throttle) in a simulator by using PID controllers based on the distance between the car and the center of the lane.

<img src="./preview.gif"  width="480">

---

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

## Constants to change

If you want to make the car to drive faster, set `FAST_AND_FURIOUS_MODE_ENABLED=true` in [main.cpp](https://github.com/matiaslgh/CarND-PID-Control-Project/blob/3e612ed6254f1214752f0c618bdb0161225cb684/src/main.cpp#L18). It's disabled by default because it doesn't drive as safe as it does when it's disabled.

If you want to enable twiddle in order to optimize the PID parameters, set `TWIDDLE_ENABLED=true` in [main.cpp](https://github.com/matiaslgh/CarND-PID-Control-Project/blob/3e612ed6254f1214752f0c618bdb0161225cb684/src/main.cpp#L16). Depending on the initial values, you might want to change the [MIN_TOLERANCE](https://github.com/matiaslgh/CarND-PID-Control-Project/blob/3e612ed6254f1214752f0c618bdb0161225cb684/src/main.cpp#L15), the [AMOUNT_OF_ITERATIONS](https://github.com/matiaslgh/CarND-PID-Control-Project/blob/3e612ed6254f1214752f0c618bdb0161225cb684/src/main.cpp#L17) (to collect errors to calculate average and then optimize params), and the initial values for [best_error](https://github.com/matiaslgh/CarND-PID-Control-Project/blob/3e612ed6254f1214752f0c618bdb0161225cb684/src/params_optimizer.cpp#L11) and [param_deltas_](https://github.com/matiaslgh/CarND-PID-Control-Project/blob/3e612ed6254f1214752f0c618bdb0161225cb684/src/params_optimizer.cpp#L17)

## Details of implementation

Given a simulator that provides the CTE (Cross Track Error / Distance from the center of the lane) several times per second, I had to calculate the best steering angle in order to reduce that error in the next iteration. As a result, the car will be most of the time in the center of the lane.

To achieve this objective I created a PID class that is initialized with three parameters (PID controller = proportional–integral–derivative controller. So, I need a coefficient for every element of the PID controller), and then it calculates the error based on the current CTE, the previous CTE and the cumulative of all the previous CTEs.

The hard part is how to set the initial coefficients. My strategy was to set a random value for `kp` (proportional coefficient) and the rest (`ki` and `kd`) with zero. Then I kept tweeking the value + running the code until the car achieved an acceptable result. Then, I left the best value I got for `kp` and I started tweeking with the same approach `kd`. Again, when I got an acceptable result, I left the best param and started tweeking `ki`. Surprisingly I got the best result with `ki=0`.

Once I got an acceptable result with the 3 coefficients already set up, it was time to fine tuning. For that, I created the class `ParamsOptimizer` which basically implements [the twiddle algorithm](https://martin-thoma.com/twiddle/). In it, I had to add some extra logic to decide which param to update, depending on the stage of the algorithm + the help of the flag `was_last_param_decreased_`. This algorithm basically takes a param, increases it a bit (delta), if the global result improves it updates the next parameter, if it gets a worse result, tries the same change but in the other direction. If that improves the result, it moves to the next parameter, if not, it leaves the value as is and moves to the next parameter. Depending on the stage it increases or decreases the delta to add/substract to a specific param. This stops when a minimun tolerance is reached.

First I tried to run twiddle in every iteration, but then I noticed that the error wasn't going to change that fast depending on the position of the car, so, I decided to accumulate the absolute value of the CTE of every iteration by `AMOUNT_OF_ITERATIONS` iterations, and after that run the optimization with the average of the errors. This worked much better, but still the changes could be too big causing the car flying out of the track. It's for that reason that I changed the iniitial values of the deltas to be smaller. That decision made sense because I had a pretty good error already, but I needed to fine tune them and this approach was great for that.

All this was always with a fixed throttle at `0.3`, using `AMOUNT_OF_ITERATIONS=10000` which was more or less an entire lap on the map.

At the end I had a pretty decent result, but at one specific curve the car was too close to the edge of the lane. It's for that reason that I added an extra validation for errors greater than `MAX_CTE` to boost the steering angle a bit more solving the problem.

With that was enough to pass the requirements of the project, but I went a step further and I tried to use a PID to control the throttle as well. At the end I got a hybrid that you can check out in the [calculateThrottle](https://github.com/matiaslgh/CarND-PID-Control-Project/blob/3e612ed6254f1214752f0c618bdb0161225cb684/src/main.cpp#L42) function. It works great! but it needs some fine tuning still. That's why I left it disabled by default and can only be activated setting the constant `FAST_AND_FURIOUS_MODE_ENABLED` to `true`

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
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

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

