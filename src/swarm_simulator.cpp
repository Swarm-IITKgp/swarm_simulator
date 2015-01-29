#include "ros/ros.h"
#include <math.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "gazebo_msgs/ModelState.h"
#include <gazebo_msgs/GetModelState.h>

#include <gazebo_msgs/GetWorldProperties.h>
#include <stdio.h>
#include <string.h>

#define print(a) std::cout << a << std::endl;

typedef struct param{
  double x;
  double y;
  int shape;
}param;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "CreateController");

  ros::NodeHandle n;

  ros::Publisher vel_pub_0 = n.advertise<geometry_msgs::Twist>("/swarmbot0/cmd_vel", 1);
  ros::Publisher vel_pub_1 = n.advertise<geometry_msgs::Twist>("/swarmbot1/cmd_vel", 1);
  ros::Publisher vel_pub_2 = n.advertise<geometry_msgs::Twist>("/swarmbot2/cmd_vel", 1);
  ros::Publisher vel_pub_3 = n.advertise<geometry_msgs::Twist>("/swarmbot3/cmd_vel", 1);
  ros::ServiceClient client = n.serviceClient<gazebo_msgs::GetWorldProperties>("/gazebo/get_world_properties"); 
  gazebo_msgs::GetWorldProperties prop;
  std::vector<std::string> s;
  if(client.call(prop))
    s=prop.response.model_names;
    //  ROS_INFO("List :%d", (prop.response.model_names.size()));
  //std::cout << "here:";

  ros::ServiceClient clien = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state"); 
  gazebo_msgs::GetModelState getmodelstate;
  gazebo_msgs::ModelState modelstate;
                  
  ros::Rate loop_rate(5);
  int **img;
 // std::cout << "here";
  img=(int **)malloc(sizeof(int *)*2400);
  for(int i=0;i<2400;i++){
    img[i] =(int *)malloc(sizeof(int)*2400);
    for(int j=0;j<2400;j++)
      img[i][j]=0;
  }

 // std::string s[]={"unit_cylinder_1", "unit_cylinder_2"};

  int count = 0;
  int s_l=s.size();
  int i=0;

  while (ros::ok())
  {
   i=0;
   std::vector<param> obs;
    while(i < s_l){
    std::cout << s[i] << std::endl;
    getmodelstate.request.model_name=s[i];
  
    if(clien.call(getmodelstate)){
    //  std::cout << getmodelstate.response.pose << std::endl;
      param toAdd;
      toAdd.x=getmodelstate.response.pose.position.x;
      toAdd.y=getmodelstate.response.pose.position.y;
      if(s[i].find("cylinder")==-1){
        toAdd.shape=0;
        obs.push_back(toAdd);
      }
      else if(s[i].find("box")==-1){
        toAdd.shape=1;
        obs.push_back(toAdd);
      }
      else if(s[i].find("sphere")==-1){
        toAdd.shape=2;
        obs.push_back(toAdd);
      }
    }
    i++;

    geometry_msgs::Twist cmd_vel;

    cmd_vel.linear.x = 0.5;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = 0.4*sin(count);


    vel_pub_0.publish(cmd_vel);
    vel_pub_1.publish(cmd_vel);
    vel_pub_2.publish(cmd_vel);
    vel_pub_3.publish(cmd_vel);

    //ROS_INFO("%d",i);
    
    ros::spinOnce();

    loop_rate.sleep();


    ++count;
    }
    
  }
  return 0;
} 