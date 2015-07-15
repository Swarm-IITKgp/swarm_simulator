#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/String.h"
#include "gazebo_msgs/ModelStates.h"
#include <string>
#include <vector>
#include "swarm_simulator/obstacleList.h"
#include "swarm_simulator/obstacleData.h"

#define print(a) std::cout << a << std::endl;

using namespace std;

std::vector<std::string> s;
std::vector<geometry_msgs::Pose> poses;

void ModelStateCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
	s = msg->name;
	poses = msg->pose;
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "swarm_simulator_node");

	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<swarm_simulator::obstacleList>("/obstacleList", 10);
	ros::Subscriber sub = n.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states",10,ModelStateCallback);
                  
	ros::Rate loop_rate(5);


	while (ros::ok())
	{
		swarm_simulator::obstacleList msg;
		
		for(unsigned i=0;i<s.size();i++)
		{
			swarm_simulator::obstacleData toAdd;
			toAdd.x = poses[i].position.x;
			toAdd.y = poses[i].position.y;

			if(s[i].find("cylinder")!=std::string::npos)
			{
				toAdd.shape=0;
		    	toAdd.radius = 2;
				msg.obstacles.push_back(toAdd);
			}
			else if(s[i].find("box")!=std::string::npos)
			{
				toAdd.shape=1;
		    	toAdd.radius = 2;
				msg.obstacles.push_back(toAdd);
			}
			else if(s[i].find("sphere")!=std::string::npos)
			{
				toAdd.shape=2;
				toAdd.radius = 2;
				msg.obstacles.push_back(toAdd);
			}
			else if(s[i].find("swarmbot")!=std::string::npos)
			{
				toAdd.shape = s[i][s[i].size()-1]-'0';
				toAdd.radius = 1.5;
				msg.obstacles.push_back(toAdd);
			}			
		}
		
		pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
} 
