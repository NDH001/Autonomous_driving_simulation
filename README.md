## Sample Simulation Results
### DSDA

![Screenshot (125)](https://user-images.githubusercontent.com/65244703/159867533-f79ccce6-0375-44d3-9419-0460f2b88297.png)

#### CSDA

![Screenshot (126)](https://user-images.githubusercontent.com/65244703/159867613-2bf24bfc-e9a7-4bbb-b82e-12db5bb57947.png)

## Simulator Installation and Setup
Disclaimer: The skeleton code of the environment setup is adopted from NUS soc, written by Prof David Hsu.
Original github page: https://github.com/AdaCompNUS/CS4278-5478-MotionPlanning

ROS Kinetic is required for this setup. The *FoodTutrle* simulator for this lab is adapted from the [ROS TurtleBot Stage](http://wiki.ros.org/turtlebot_stage) package.
Install TurtleBot Stage:
```
sudo apt install ros-kinetic-turtlebot-stage
sudo apt install ros-kinetic-joint-state-publisher-gui
```

Clone this repository and set up the environment:
```
git clone https://github.com/NDH001/Autonomous_driving_simulation.git
cd Autonomous_driving_simulation
catkin_make
source devel/setup.bash
```
If you use the ROS docker, you may need to make a [docker commit](https://docs.docker.com/engine/reference/commandline/commit/) to save changes. 

Check your setup:
```
roslaunch planner turtlebot_in_stage.launch
```
You should see RViz and ROS Stage. 

## Code Execution
Launch the simulator and execute the planner: 
```
roscd planner/src
sh run.sh [your map name] start_x start_y start_theta
python your_planner.py --goal 'goal_x,goal_y' --com use_com_1_map_or_not
```

For example, load `map2.png` and set the robot start pose as (x=1, y=1, θ=0): 
```
roscd planner/src
sh run.sh map2.png 1 1 0
```
Set the robot goal as (x=5, y=5) and run the planner:

For discrete space, we use DSDA.py which is the implementation of standard A* search.
```
python DSDA.py --goal '5,5' --com 0
```
For continuous space, we use CSDA.py which is the implementation of hybrid A* search.
```
python CSDA.py --goal '5,5' --com 0
```
The flag `--com`  indicates whether the COM1 map is used, as it requires a special set of environment parameters. 



