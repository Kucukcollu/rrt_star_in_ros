# motion_planner

OMPL(Open Motion Planning Library) based 2D RRT* planner ROS Node

![](https://github.com/Kucukcollu/ford_otosan_solution/blob/master/figures/rviz_screenshot.png)<br></br>

* Green Markers ==> Obstacles in the map (100 x 100 m) 
* Blue marker   ==> Start point
* Red Marker    ==> Goal Point

## Dependencies

```bash
Install ROS C++ API

$ sudo apt-get install ros-melodic-ompl

Install binary library

$ sudo apt-get install libompl-dev
```

## Setup

```bash
Create a workspace

$ cd ~/
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws
$ catkin_init

move the motion_planner package to the src folder

$ mv ${package_dir} ~/catkin_ws/src/

build

$ catkin build
```

- **motion_planner** package has a service which is **PlanningService.srv**
The planner accepts start and goal points via this service.
Service structure is like following.

## ROS custom service: PlanningService.srv
```bash
float64 start_point_x
float64 start_point_y
float64 goal_point_x
float64 goal_point_y
---
bool is_path_valid
time calculation_time
```

## Run

```bash
launch the nodes

$ roslaunch motion_planner start.launch

call the service with

$ rosservice call /planning_service ...

example usage

$ rosservice call /planning_service "start_point_x: -10.0
start_point_y: -5.0
goal_point_x: 10.0
goal_point_y: 20.0"
is_path_valid: True
calculation_time:
  secs: 0
  nsecs:         0

```

## Sources
- http://ompl.kavrakilab.org/planners.html
- https://ompl.kavrakilab.org/OptimalPlanning_8cpp_source.html
