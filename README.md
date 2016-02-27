swarm_simulator
================

This is the simulator for the Swarm Robotics project at IIT Kharagpur.
The simulator is based on Gazebo, interfaced using ROS (Robot Operating System).

Prerequisites :
- Install Gazebo with ROS. Follow [Gazebo Install](http://gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros).

To get started :

- Clone the repo as :
```
$ git clone https://github.com/Swarm-IITKgp/swarm_simulator.git swarm_simulator
```

- Make sure $GAZEBO_MODEL_PATH is set in ~/.bashrc or set it to the location it is. If not, then add the following line to .bashrc:
```
export GAZEBO_MODEL_PATH=<path to gazebo models folder>
```

- Add the following lines to your ~/.bashrc file at the end:
```
source <path to catkin_ws>/devel/setup.bash
```

- Close the current window and open a new terminal.

- To generate the launch file :
```sh
 $ roscd swarm_simulator
 $ bash scripts/generator.sh [number of agents]
```

- To launch the simulator :
```sh
 $ roslaunch swarm_simulator swarm.launch
```

- To check the published obstacle list : <br/>
```sh
 $ rostopic echo /obstacleList
```
