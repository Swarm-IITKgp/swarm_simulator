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
  double radius;
  int shape;
}param;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "CreateController");

  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<gazebo_msgs::GetWorldProperties>("/gazebo/get_world_properties"); 
  gazebo_msgs::GetWorldProperties prop;
  std::vector<std::string> s;
  if(client.call(prop))
    s=prop.response.model_names;
    //  ROS_INFO("List :%d", (prop.response.model_names.size()));

  ros::ServiceClient serv_client = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state"); 
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
    
      if(serv_client.call(getmodelstate)){
      //  std::cout << getmodelstate.response.pose << std::endl;
        param toAdd;
        toAdd.x=getmodelstate.response.pose.position.x;
        toAdd.y=getmodelstate.response.pose.position.y;
        toAdd.radius = 2.0;
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
    }    
  }
  return 0;
} 