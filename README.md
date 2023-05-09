# GVSETS_RL_QuarterCar
Deep reinforcement learning for simultaneous path planning and stabilization of offroad vehicles

![alt text](https://github.com/ameyarsalvi/GVSETS_RL_QuarterCar/blob/main/gvsets_rl_1.png)

### IMPORTANT FILES
1. QuarterCarRLDDPG_vel_osc_more_layers.m
This is the main file which calls all the sub files. Running this file directly starts the training process with existing hyperparameters.

2. QuarterCarENV.m
This is the first file called by the main file. This file defines the Quarter Car environment in context of deep reinforcement learning. This involves the definitions for observations and action spaces, reset and reward functions.

3. QuarterCar.m
This file computes the one step simulation for quarter car using ODE45.

## Training : 
Run the QuarterCarRLDDPG_vel_osc_more_layers.m as is. This starts the training process. Once the training is finished, the trained agent is saved as : VelocityOscAgentFreq2_neg2vel.mat

## Evaluation : 
The training file contains the evaluation section itself. Once evaluation is complete, data is stored in sim_states_T_C2.mat and actionData_T_C2.mat, which can then be ploted using paper_plots.m file for eps format images.

