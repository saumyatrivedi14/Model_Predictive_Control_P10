# Reflection of the Project
---
[//]: # (Image References)
[image1]: ./media/global_kinematic_model.png "Kinematic Model"

## Project Introduction
---
The main aim of the project is to develop a MPC controller to control steer and throttle of a vehicle around a track in a simulator. The vehicle model used in this case is called Global Kinematic Model as shown below. 

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
3. Cost Equation

### States and Actuator Variables
Initial state vector consists of x-position, y-position, orientation and velocity of the vehicle. The cross talk error and the error in the orientation are also part of the final state. So the overall model for the controller looks something as shown below.
