#include "ros/ros.h"
#include <math.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "gazebo_msgs/ModelState.h"
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/GetWorldProperties.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <cstdlib>
#include <iostream>

#include "swarm_simulator/obstacleList.h"
#include "swarm_simulator/obstacleData.h"
#include "swarm_simulator/polarCoordinates.h"

#define NUM_BOTS 4
#define print(a) std::cout << a << std::endl;

using namespace std;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "CreateController");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<swarm_simulator::obstacleList>("/obstacleList", 1);
  ros::Publisher polarCoordPublisher[NUM_BOTS];
  ros::Publisher pose_publisher[NUM_BOTS];
  for (int i = 0; i < NUM_BOTS; ++i)
  {
    stringstream ss;
    ss << i;
    string int_str = ss.str();
    string str = "/swarmbot";
    str += int_str;
    string str1 = str + "/closebot";
    polarCoordPublisher[i] = n.advertise<swarm_simulator::polarCoordinates>(str1,1);
    string str2 = str + "/pose";
    pose_publisher[i] = n.advertise<geometry_msgs::Pose>(str2, 1);
  }
  ros::ServiceClient client = n.serviceClient<gazebo_msgs::GetWorldProperties>("/gazebo/get_world_properties"); 
  gazebo_msgs::GetWorldProperties prop;
  std::vector<std::string> s;
  geometry_msgs::Pose bot_pos[NUM_BOTS]; 
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
    swarm_simulator::polarCoordinates polarCoordinateMessage[NUM_BOTS];
    while(i < s_l){

      //cout << s[i];

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
        else if(s[i].find("swarmbot")!=std::string::npos)
        {
          string num = s[i];
          num.erase(0, string("swarmbot").length());
          std::istringstream i(num);
          int bot_num;
          i >> bot_num;
          geometry_msgs::Pose temp_pose;
          temp_pose = getmodelstate.response.pose;
          pose_publisher[bot_num].publish(temp_pose);
          bot_pos[bot_num] = temp_pose;
        }
      }
      i++;
    }
    for(int j=0;j < NUM_BOTS;j++)
    {
      float min = 10000;
      float min_theta = 0;
      int min_id = -1;
      for(int k=0;k < NUM_BOTS;k++)
      {
        if(j==k)
          continue;
        float delta_x = bot_pos[k].position.x - bot_pos[j].position.x;
        float delta_y = bot_pos[k].position.y - bot_pos[j].position.y;
        float r = sqrt(pow(delta_x, 2) + pow(delta_y, 2));
        float theta = atan(delta_y / delta_x);
        if(delta_x < 0)
        {
          if(theta > 0)
            theta -= M_PI;
          else
            theta += M_PI;
        }

        if(r<min)
        {
          min = r;
          min_theta = theta;
          min_id = k;
        }
      }
      //In 2D, the quaternions are <0, 0, sin(theta/2), cos(theta/2)>
      float z = bot_pos[j].orientation.z;
      float w = bot_pos[j].orientation.w;
      float self_theta = acos(w);
      if(z < 0) self_theta = -self_theta;
      self_theta *= 2;
      self_theta = fmod(self_theta, float(2*M_PI));

      min_theta -= self_theta;
      min_theta = fmod(min_theta, float(2*M_PI));
      if(min_theta > M_PI) min_theta -= 2*M_PI;

      polarCoordinateMessage[j].r = min;
      polarCoordinateMessage[j].theta = min_theta;
      polarCoordinateMessage[j].id = min_id;
    }
    pub.publish(msg);
    for(int i=0; i < NUM_BOTS ; i++)
      polarCoordPublisher[i].publish(polarCoordinateMessage[i]);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
} 