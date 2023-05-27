# Autonomous underwater robot with intelligent navigation system.

Navigation through an unknown environment requires the development of advanced navigation algorithms that can continuously and accurately detect new obstacles and adjust the ROV’s trajectory in real-time to avoid a collision. 

In this project, the robot to traverse a two-dimensional maze where the position of the obstacles is initially unknown. Implemented in ROS noetic environment, for this, an A* search algorithm is used to navigate and maneuver around obstacles from a definite start point to a set destination point. Also makes use of a grid map to define the parameters of the maze in order to generate a path whenever an obstacle is detected along the path.

 <p align="left">
  <img src="Autonomous Underwater Robotics Image/Fig 14 The testing environment for the experiment.png" width="350" title="The testing environment for the experiment">
</p> 

## Grid map
The water tank is considered as a 6x3 grid where each cell is 50cm by 50 cm (defined by the size of the BlueROV and the size of the tank)
 <p align="left">
  <img src="Autonomous Underwater Robotics Image/Fig 8 Manhattan distance.png" width="250" title="Manhattan distance">
</p> 

## Motion Control
Robot motion control: Four controllers to control the longitudinal, lateral, yaw and vertical axes of the BlueROV.
PID control (Proportional Integral Derivative control):
+ Depth Controller Z - Axis : Constant depth 
+ Longitudinal Controller Y and X Axes : Motion Planner
+ Latitudinal Controller Y and X Axes :  Motion Planner
+ Yaw Controller :  Motion Planner

Anti Windup Strategy:
Stop accumulating error for the integral term when saturation is reached

<p align="left">
  <img src="Autonomous Underwater Robotics Image/Fig 12 Generic PID control block diagram.png" width="450" title="Generic PID control block diagram">
</p>

## Implemented Algorithm 
<p align="center">
  <img src="Autonomous Underwater Robotics Image/Fig 7 The overall setup of the ROS nodes.png" width="850" title="The overall setup of the ROS nodes"> 
</p>

# Localization
BlueROV is localized using the AprilTags that are on the floor of the tank. The BlueROV’s bottom camera is used to detect the AprilTags. The localization scheme developed by the institute is used to find the pose of the ROV w.r.t the tank coordinate frame. The orientation is converted from quaternions to euler angles.

<p align="left">
  <img src="Autonomous Underwater Robotics Image/Fig 10 The obstacles being detected by the BlueROV’s frontal camera.png" width="550" title="The obstacles being detected by the BlueROV’s frontal and down camera">
</p>

## Obstacle Detection along the path
Continuous Obstacles (here Apriltags) are detected using the front camera sensor at each node. Upon detection, the path planner is initiated again to update the map and create a new path.

Limitation: Only obstacles directly in front of the ROV can be detected. 
 
<p align="left">
  <img src="Autonomous Underwater Robotics Image/Fig 22 Obstacle being detected in the real-life experiment.png" width="550" title="Obstacle being detected in the real-life experiment">
</p>

## Motion Planning Algorithm.
Localization is achieved w.r.t the tank coordinate frame. Hence, the direction in which the ROV moves determines which controller (either longitudinal or lateral) will read which position (x or y).  
<p align="center">
  <img src="Autonomous Underwater Robotics Image/Fig 11 Expected outcome of motion planner.png" width="650" title="Expected outcome of motion planner">
</p>
Motion Planner: Linear motion to one of 4 adjacent cells. The camera must face the direction of travel.

## Path planning Algorithm
Path Planner using A* search algorithm
<p align="center">
  <img src="Autonomous Underwater Robotics Image/Fig 13 The setup for the experiment and expected outcome.png" width="250" title="Expected outcome of path planner">
</p> 

## Setup of the Project
Two obstacles are to be placed in the tank as seen in the Figure. (1.3, 0.5)m and (2.6, 1)m.
The ROV is first moved to the cell (6,1) using the function goToStart().
The target cell is cell (1,1).
The ROS nodes are launched.
Rosbags activated to record the pose of the ROV and the controller setpoints. 

<p align="left">
  <img src="Autonomous Underwater Robotics Image/Fig 1 Top view of experimental setup.png" width="250" title="Top view of experimental setup">
</p>
