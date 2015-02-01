#include "ros/ros.h"
#include <math.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "gazebo_msgs/ModelState.h"
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/GetWorldProperties.h>
#include <stdio.h>
#include <string.h>

#include "swarm_simulator/obstacleList.h"
#include "swarm_simulator/obstacleData.h"

#define print(a) std::cout << a << std::endl;

using namespace std;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "CreateController");

  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<swarm_simulator::obstacleList>("/obstacleList", 1);
  ros::ServiceClient client = n.serviceClient<gazebo_msgs::GetWorldProperties>("/gazebo/get_world_properties"); 
  gazebo_msgs::GetWorldProperties prop;
  std::vector<std::string> s;
  if(client.call(prop))
    s=prop.response.model_names;

  ros::ServiceClient serv_client = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state"); 
  gazebo_msgs::GetModelState getmodelstate;
  gazebo_msgs::ModelState modelstate;
                  
  ros::Rate loop_rate(5);
  int **img;
  img=(int **)malloc(sizeof(int *)*2400);
  for(int i=0;i<2400;i++){
    img[i] =(int *)malloc(sizeof(int)*2400);
    for(int j=0;j<2400;j++)
      img[i][j]=0;
  }

  int count = 0;
  int s_l=s.size();
  int i=0;

  while (ros::ok())
  {
    i=0;
    swarm_simulator::obstacleList msg;
    while(i < s_l){
      getmodelstate.request.model_name=s[i];

      if(serv_client.call(getmodelstate)){

        swarm_simulator::obstacleData toAdd;
        toAdd.x=getmodelstate.response.pose.position.x;
        toAdd.y=getmodelstate.response.pose.position.y;
        toAdd.radius = 2.0; 

        if(s[i].find("cylinder")!=std::string::npos){
          toAdd.shape=0;
          msg.obstacles.push_back(toAdd);
        }
        else if(s[i].find("box")!=std::string::npos){
          toAdd.shape=1;
          msg.obstacles.push_back(toAdd);
        }
        else if(s[i].find("sphere")!=std::string::npos){
          toAdd.shape=2;
          msg.obstacles.push_back(toAdd);
        }
      }
      i++;
    }   
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
} 