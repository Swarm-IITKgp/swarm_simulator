swarm_simulator
======================

This is the simulator for the Swarm Robtics project at IIT Kharagpur.
The simulator is based on Gazebo, interfaced using ROS (Robot Operating System).

Prerequisites :
-> Install Gazebo with ROS. Follow http://gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros.

To get started :
-> Clone the repo as :
  git clone https://github.com/Swarm-IITKgp/swarm_simulator.git swarm_simulator
  
-> Make sure $GAZEBO_MODEL_PATH is set in ~/.bashrc or set it to the location it is.

-> To launch the world file :
 $ roslaunch swarm_simulator swarm.launch
  
-> To run the code in cpp file :
 $ rosrun swarm_simulator swarm_simulator_node

To check the published obstacle list : <br />
 $ rostopic echo /obstacleList
  
  
