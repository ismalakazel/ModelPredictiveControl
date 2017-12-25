# CarND Model Predictive Control
*Self-driving Car Nanodegree at Udacity.*

**Goals:**

- Learn about Model Predictive Control
- Implement Model Predictive Control in C++
- Learn C++

**Tools:**
- [CarND-MPC-Project](https://github.com/udacity/CarND-MPC-Project)
- [Xcode](https://developer.apple.com/support/xcode/)


## The Model

The model is constructed, according to the lessons in the Model Predictive Control class, as follows:

```
// x position
xt + 1 = xt + vt * cos(ψt) * dt

// y position
yt + 1 = yt + vt * sin(ψt) * dt

// Orientation
ψt + 1 = ψt + vt/Lf * 𝛿 * dt

// Velocity
vt + 1 = vt + at * dt

// Steering angle
ctet + 1 = f(xt) - yt + vt * sin(eψt) * dt

// Throttle
eψt + 1 = ψt  = ψdest + vt/Lf * 𝛿t * dt
```

In this model one variable was provided by the study material - Lf - which is distance between front of the vehicle and its center of gravity.

The class `Vehicle.h` models the attributes and implements the necessary functions to have the state of the vehicle sent to the `MPC.h` class. `Vehicle` state is representated by [x, y, orientation, speed]. `Vehicle` actuators are represented by [steering angle, throttle]. No other state or state influencers are accounted in the implementation (But that would be nice to study and implement).

## Timestep Length and Elapsed Duration (N & dt)

In this implementation the the prediction horizon is set to 1 - **N** * **dt** - where **N** = 10 and **dt** = 0.1. These parameters where chosen by trial and error in combination with the penalties for orientation and velocity. Other parameters where chosen, also in combination with the penalties, but they caused inconsistencies in car speed and/or made the vehicle overshoot when making curves.

## Polynomial Fitting and MPC Preprocessing

In `Vehicle.h` the `convertCoordinates` function was built to convert global coordinates to vehicle coordinates. This step was necessary before get the coefficientes from a 3rd order polynomial function of x and y vectors. The coordinates are converted as follows:

```
for (int n = 0; n < xp.size(); n++) {
    double x = xp[n] - this->x;
    double y = yp[n] - this->y;
    x_values[n] = x * cos(-orientation) - y * sin(-orientation);
    y_values[n] = x * sin(-orientation) + y * cos(-orientation);
}
```
After fitting the coordinates the state of the `Vehicle` and its actuators ared updated in the `move` function (Maybe should have been called sense as in the examples provided in the localization class).

## Model Predictive Control with Latency

This implementation uses a delay time of 0.1 to compensate for the 100ms latency in the simulator. A bigger number causes the vehicle to zig-zag, while a smaller number doesn't have any meaningfull effect in terms of sensing the vehicle state.

## Preview

[Preview](https://www.youtube.com/watch?v=iZRxPHhc5eg)
