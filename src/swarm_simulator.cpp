#include "ros/ros.h"
#include <math.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "gazebo_msgs/ModelState.h"
#include <gazebo_msgs/GetModelState.h>
#include <stdio.h>
#include <string.h>


typedef struct param{
  double x;
  double y;
  int shape;
};

int main(int argc, char **argv)
{

  ros::init(argc, argv, "CreateController");

  ros::NodeHandle n;

  ros::Publisher vel_pub_0 = n.advertise<geometry_msgs::Twist>("/swarm_0/cmd_vel", 1);
  ros::Publisher vel_pub_1 = n.advertise<geometry_msgs::Twist>("/swarm_1/cmd_vel", 1);
  ros::Publisher vel_pub_2 = n.advertise<geometry_msgs::Twist>("/swarm_2/cmd_vel", 1);
  ros::Publisher vel_pub_3 = n.advertise<geometry_msgs::Twist>("/swarm_3/cmd_vel", 1);
  ros::Publisher vel_pub_4 = n.advertise<geometry_msgs::Twist>("/swarm_4/cmd_vel", 1);
  ros::Publisher vel_pub_5 = n.advertise<geometry_msgs::Twist>("/swarm_5/cmd_vel", 1);
  ros::ServiceClient client = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state"); 
  gazebo_msgs::GetModelState getmodelstate;
  gazebo_msgs::ModelState modelstate;
  std::string s[] ={"grey_wall", "grey_wall_0" ,"grey_wall_1", "grey_wall_2", "grey_wall_3", "grey_wall_4", "grey_wall_5", "grey_wall_6", "grey_wall_7", "grey_wall_8", "grey_wall_9", 
                  "grey_wall_10", "nist_maze_wall_120" , "nist_maze_wall_120_1" , "nist_maze_wall_120_2" , "nist_maze_wall_120_3" , "nist_maze_wall_120_4" , "nist_maze_wall_120_5"
                  , "nist_maze_wall_120_6", "nist_maze_wall_120_7" , "nist_maze_wall_120_8" , "nist_maze_wall_120_9" , "nist_maze_wall_120_10" , "nist_maze_wall_120_11"
                  , "nist_maze_wall_120_12" , "unit_sphere_1", "unit_sphere_2", "unit_box_1", "unit_box_2", "unit_cylinder_1", "unit_cylinder_2" , "unit_cylinder_3", "swarm_0", "swarm_5",
                  "swarm_4", "swarm_3", "swarm_1", "swarm_2" };
                  
  ros::Rate loop_rate(5);
  int **img;
  img=(int **)malloc(sizeof(int *)*2400);
  for(int i=0;i<2400;i++){
    img[i] =(int *)malloc(sizeof(int)*2400);
    for(int j=0;j<2400;j++)
      img[i][j]=0;
  }

  int count = 0;
  int s_l=0;
  int i=0;

  while(!s[s_l].empty())
    s_l++;

  while (ros::ok())
  {
   i=0;
   std::vector<param> obs;
  while(i++ < s_l){
    std::cout << i << std::endl;
    getmodelstate.request.model_name=s[i];
  
    if(client.call(getmodelstate)){
      std::cout << getmodelstate.response.pose << std::endl;
      param toAdd;
      toAdd.x=getmodelstate.response.pose.position.x;
      toAdd.y=getmodelstate.response.pose.position.y;
      obs.push_back(toAdd);
      //now color for respective shape  
    }
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
    vel_pub_4.publish(cmd_vel);
    vel_pub_5.publish(cmd_vel);

    //ROS_INFO("%d",i);
    
    ros::spinOnce();

    loop_rate.sleep();


    ++count;
    }
    
  }
  //ros::spin();

  return 0;
}
