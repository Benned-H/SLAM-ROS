# SLAM-ROS
A repository for code developed by and for UR Robotics implementing various algorithms from the Probabilistic Robotics textbook. Algorithms so far implemented:

* Velocity-based motion model for forward simulation of a differential drive robot.
* A* search for planning paths on a discrete grid.
* Pure pursuit for path-following control.
* Occupancy grid mapping using laserscan sensor data.




## Demos

Planner - Run `roslaunch planner planner_demo.launch` to see the planner operate on a continuous stream of queries from (0,0) to points on a 3m-radius circle.

## Known TO-DO:

* Revisit GUI RViz files and ensure they're what we want to be using.
* Various TO-DO comments throughout code.
* Check simulator and mapper launch files as well.


