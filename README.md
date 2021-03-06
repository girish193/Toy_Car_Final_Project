# Toy_Car_Final_Project
This repository uses path planning algorithms such as A*, RRT, and RRT* for a user-defined toy car differential drive.

## Step 1 (Implementation of Path Planning Algorithms for Toy Car)

### Description
The robot here under consideration is an user defined Toy Car model. Various path planning algorithms such as A*, RRT and RRT* are implemented. A* algorithm gives an optimal path whereas in case of RRT and RRT*, it only gives a suboptimal path as nodes are randomly chosen and connected to the nearest node based on its Euclidean distance (heuristic measure here). All the nodes corresponding with different points (x,y) on the map are explored until a goal is found.

### Run Code
First run either of a_star.py / rrt.py / rrt_star.py file. It will generate an output either a_star_controls.txt / rrt_controls.txt / rrt_star_controls.txt file in the parent of code directory. Use the script of waypointCreation.py to generate inbetween waypoints in the path_planning. This will generate a .csv file in the parent of code directory.

The world considered here is same as considered in the Differential_Drive repo (project 3_c). To keep the code simple and not to clutter the terminal, user inputs regarding start orientation and goal orientation is set to (-4, -4, 0) and (4, 4, 0) w.r.t. gazebo world coordinates. 

##### RRT Path
![RRT Path](images/RRT.png)

##### RRT* Path
![RRT* Path](images/RRTstar.png)

### Dependencies
a) python -version 3

b) numpy

c) sys

d) matplotlib

e) time

f) math

g) queue

h) heapq

i) random

j) collections

### Parameters
a) threshold_distance = 0.1 (threshold distance between each node in m)

b) goal_threshold_radius = 0.1 (threshold radius for goal node in m)

c) clearance = 0.4 (effective clearance i.e., radius of robot & obstacle clearance)

d) dr = 0.1 (differential steer length needed in m)

e) r = 1 (total steering for each action in m)

### Class Description
#### 1) Node
Inside this class, we use an init special built-in function to initialize different instances of this class. We initialize the node state (x, y, theta) and its coressponding parent node.  

### Function Descriptions 
(Only the main ones are mentioned here)
#### 1) isValidWorkspace
In this function the given value is checked against obstacles and true is returned if it lies outside obstacle space and also takes into consideration the clearance needed for the toy car. Obstacles which are given are two circles, 1 square, and two rectangles.

#### 2) isSafe
This function checks whether next action is near an obstacle or ill defined.

#### 3) steer
This function steers the toy car.

#### 4) nearest
This function finds the nearest neighboring node for a given node.

#### 5) generatePath
This is the function which calls other functions and generates the required RRT/ RRT* path. We have set a the maximum number of iterations to try to generate these paths as 30000.  

##### NOTE :
Using Pygame, animation is generated. Firstly, animation for node exploration is generated and followed by best solution path trajectory's animation. Keep caution while using Pygame as the origin of Pygame is at top-left corner in the workspace while the Gazebo worls coordinates are at the center.  

## Step 2 (Gazebo Simulation)

### Description
We implement the results of the above Path Planning Algorithms on our Toy Car by simulating it on gazebo using ROS.

### Dependencies
a) python -version 2.7

b) python -version 3

c) ROS

d) Gazebo

### Run 

Follow all the steps mentioned in step 1. It generates the required files as output. Firstly, build your catkin workspace with both the 'main_assem' and 'simple_navigation_goals'. 

Open terminal and type the following, 

###### roslaunch main_assem main.launch

We should see the Toy Car spawned in our world as shown below. Then in another terminal tab, type the following, 

##### Toy Car in Gazebo
![Toy Car in Gazebo](images/Toy_car_spawned_in_world.png)

Then in another terminal tab, type the following,

###### roslaunch simple_navigation_goals movebase_seq.launch

Ideally, we would observe the turtlebot to move in our new world with obstacles with the path generated from the above planning algorithms (step 1). Here instead of using the velocities to publish on the /cmd_vel topic, we use action client-server move base method (see inside the scripts directory of simple_navigation_goals package). We set the poses to reach on the parameter server (see inside the launch directory/ movebase_seq.launch). Later in the move_base_seq.py file, we get these parameters one at a time and move our toy car to that pose. This way of publishing onto our toy car is much more suited to our purpose. 

However, occasionally we have to wait for a long time for the action server to start (No idea how to fix this yet! Probably the clock on my machine is not properly synchronized with ROS). In future with proper diagnostics, this error will be fixed.
