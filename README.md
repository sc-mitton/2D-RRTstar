# 2D-RRTstar

# Overview

A simulation of obstical avoidance with a two link robot which uses a sampling based RRT* algorithm.

### Code

1. A start and goal position is chosen
2. The objects are mapped from cartesian space into joint space (the combination of joint positions for the robot)
3. Randomly explore joint space with the RRT and once the tree comes within a certain threshold of the goal, the algorithm stops and an optimal path through the tree towards the goal is found. 

### Running

The driver for the program is RRTStar.m which runs the simulation in Matlab. The progam uses [Robot Toolbox for Matlab](https://petercorke.com/toolboxes/robotics-toolbox/).


![alt text](https://github.com/sc-mitton/rrtStar/blob/assets/image1.png) 

![alt text](https://github.com/sc-mitton/rrtStar/blob/assets/image2.png) 

![alt text](https://github.com/sc-mitton/rrtStar/blob/assets/image3.png) 

![alt text](https://github.com/sc-mitton/rrtStar/blob/assets/image4.png) 

![alt text](https://github.com/sc-mitton/rrtStar/blob/assets/image5.png) 

![alt text](https://github.com/sc-mitton/rrtStar/blob/assets/image6.png) 

![alt text](https://github.com/sc-mitton/rrtStar/blob/assets/image7.png) 