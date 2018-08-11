# Reflection of the Project
---
[//]: # (Image References)
[image1]: ./media/global_kinematic_model.JPEG "Kinematic Model"
[image2]: ./media/states.JPEG "State Vector"
[image3]: ./media/controller_model.JPEG "Controller Model"
[image4]: ./media/constraint_equations.JPEG "Constraints"




## Project Introduction
---
The main aim of the project is to develop a MPC controller to control steer and throttle of a vehicle around a track in a simulator. The vehicle model used in this case is called Global Kinematic Model as shown below. 

![alt text][image1]

There are four main parts of this project:
1. Controller Model
2. Tuning of N (timestep length) & dt (timestep duration)
3. Polynomial fitting and MPC preprocessing
4. MPC with latency

## Controller Model
---
There are three main important aspects of controller modelling:
1. States and Actuator variables
2. Constraints 
3. Cost Function

### States and Actuator Variables
Initial state vector consists of x-position, y-position, orientation and velocity of the vehicle. The cross talk error and the error in the orientation are also part of the final state. 

![alt text][image2]

So the overall model for the controller looks something as shown below.

![alt text][image3]

The actuator variables here are the steering angle (delta) and throttle (a). Here throttle value is assumed to be between -1 and 1, where positive values are considered to be for acceleration and negative fro braking. 

### Constraints
These equations are the one which model should follow while solving outputs, they are based on vehicle model. These equations are shown below, along with that there are constraints on actuators too based on physical constraints, like steering angle is between -25 to 25 degrees and throttle between -1 and 1 as shown below.

![alt text][image4]

### Cost Function
These is the important part of solver, because the optimizer tries to find the optimal values for the actuators by minimizing the cost function. The cost function consists of all the error terms which the optimizer reduces, hence it consists of postion erros, orientation error, actuator error and many more terms can be added to further improve the performance. Here, 6 types of squared sum error terms are considered:

1. Cross Track Error (CTE) 
2. Vehicle Orientation
3. Actuator Command (Steering and Throttle)
4. Actuator Delta Command (to reduce rate of change of actuator commands)
5. Speed Error
6. Minimizing Speed wrt Steering and vice versa

All these error terms have weights associated with it to tune it for smoother performance of vehicle.

## Time-step Length and Duration
---
These parameters play huge role on computing cost function and predicting the actuator inputs to the car ahead of time. Higher value of N, more number of estimates while higher value of dt gets the estimates closer in time. Also with too large N, the controller fails to calculate real time estimates for a given time horizon, whereas with too large dt, it adds more inaccuracy to the model as it assumes control inputs to be constant over the dt time period. There were several combinations of N & dt that were tried and effects of each are mentioned below:

```sh
N       dt        Effect
20      0.1       Vehicle turned quickly and predicted too ahead of time giving prediction trajectory too many swirls
10      0.1       Vehicle oscillating but very unstable at high speeds
10      0.05      Vehicle oscillations were lowered but on turns vehicle is late to predict
8       0.05      Oscillations were lowered even  more but it affected turning of vehicle even further giving a late braking effect.
```
With N = 8 and dt = 0.05, the vehicle is able to complete the track without going off track at reference speed of 40, but at higher speeds, its not able to handle turns very well. It needs further tuning! :(

## Polynomial fitting and Pre-processing
---
The reference waypoints of the vehicle received from the simulator are in map co-ordinates and they are tranformed to vehicle co-ordinate systems to avoid complex calculation of CTE and psi at each point. The waypoints are translated to vehicle co-ordinate system using vehicle's orientation (psi) and then it's fitted using functions polyfit and polyeval into a 3rd degree polynomial to further calculate CTE and EPSI (error in vehicle orientation)

## MPC with Latency
---
There were two types of latency observed while running simulation, one was manual latency of 100ms introduced and the other was when Ubuntu Virtual Machine was communicating with the simulator running on windows. For first one, the states were updated based on the Global Kinematic Model to predict the states after the latency (implemented in main.cpp), whereas for the second the actuation was delayed by one time-step (implemented in MPC.cpp).
