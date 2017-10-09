# mpc_control
A simple implementation of MPC control 

## The model
A kinematic bicycle model of the vehicle is used to model the car. This model has the following states:
[ x location, y location, heading angle, velocity] 

Two actuators are used to actuate the model - steering angle and a throttle value. Both the steering angle and throttle are limited between min and max values. 

## Optimizing the actuation
The basic idea is to get the current vehicle state and predict the state of the vehicle forward in time based on the current actuation and the vehicle model. An optimizer is then used to optimize a constructed cost based on constraints to find the best possible actuations to keep the cost low. The best guess is used and then the entire process repeats again.

## Choosing N and dt
N is the number of points ahead in time that the vehicle state is predicted for. Together with dt, the model's state is predicted for a time of Nxdt forward. Therefore, it makes sense to not choose too large a value of N since 
* the optimizer has to do more work for larger values of N 
* predicting too far ahead in time will eventually produce noise since the kinematic model is not 100% accurate

The choice of dt is determined by :
* Not choosing a very large value for the same reason as above
* Choosing a value > latency in the actuator system to ensure correct actuation

A value of N = 10 and dt = 0.15 is chosen keeping the above in mind

## Pre-processing waypoints
The track waypoints are pre-processed to change them from global coordinates to the car's coordinates. This helps with the polynomial fitting. This is done simply by displacing the global origin to the car's location and then rotating the global coordinate system by the heading angle of the car such that the car is always at x,y,psi = 0, 0, 0. 

## Handling latency
Latency is handled by moving the current measurement of the car forward by the amount of latency. Thus the states being sent for optimization are already delayed by the amount of latency. 

