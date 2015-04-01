#include "ros/ros.h"
#include <math.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
 
#define TOL 0.5
 
geometry_msgs::Pose current;
geometry_msgs::Pose initial;
bool flag = true;
 
void odomCB(const nav_msgs::Odometry& msg)
{
  if(flag == true)
  {
    flag = false;
    initial = msg.pose.pose;
  }
  else
  {
    current = msg.pose.pose;
  }
  //std::cout << msg.pose.pose.position.x << " " << msg.pose.pose.position.y << " " << msg.pose.pose.position.z << std::endl;
}
 
/**
 * This tutorial demonstrates simple sending of velocity commands to the IRobot Create in Gazebo.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "SwarmSimu");
 
  ros::NodeHandle n;
 
  ros::Publisher vel_pub_0 = n.advertise<geometry_msgs::Twist>("/swarmbot0/cmd_vel", 1);
  ros::Subscriber odom_data = n.subscribe("/swarmbot0/odom",10,odomCB);
  // ros::Publisher vel_pub_1 = n.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1);
  ros::Rate loop_rate(5);
 
  int count = 0;
 
  geometry_msgs::Twist cmd_vel;
  cmd_vel.angular.z = -0.4;
 
  bool flipped = false;
 
  while (ros::ok())a
  {
    nav_msgs::Odometry odom;
    cmd_vel.linear.x = 1;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    // cmd_vel.angular.z = 0.4*sin(count);
 
    if((current.position.x < initial.position.x + TOL && current.position.x > initial.position.x - TOL)&&(current.position.y < initial.position.y + TOL && current.position.y > initial.position.y - TOL))
      {
        if(flipped==false)
        {
          cmd_vel.angular.z = -cmd_vel.angular.z;
          flipped = true;
          initial = current;
        }
      }
    else
      flipped = false;
    vel_pub_0.publish(cmd_vel);
    ros::spinOnce();
 
    loop_rate.sleep();
 
    ++count;
  }
  return 0;
}

/*
#include "ros/ros.h"
#include <math.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"


 // This tutorial demonstrates simple sending of velocity commands to the IRobot Create in Gazebo.
 
 void callback(const std_msgs::String::ConstPtr& msg)
 {
    ROS_INFO("I heard: [%s]", msg->data.c_str()); 
 }
int main(int argc, char **argv)
{

  ros::init(argc, argv, "SwarmSimu");

  ros::NodeHandle n;

  ros::Publisher vel_pub_1 = n.advertise<geometry_msgs::Twist>("/swarmbot1/cmd_vel", 1);
 // ros::Publisher vel_pub_2 = n.advertise<geometry_msgs::Twist>("/swarmbot0/cmd_vel", 1);
  // ros::Publisher vel_pub_1 = n.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1);
 // ros::Subscriber sub = n.subscribe("/swarmbot1/odom/",2);
  ros::Subscriber sub = n.subscribe("/swarmbot0/odom/pose/pose/orientation/x", 1,callback );
 // float theta = sub.pose.pose.orientation;
   //float theta = tf::getYaw(sub->pose.pose.orientation) ;
  ros::Rate loop_rate(5);

  int count = 0;
  int counter = 0 ;

  while (ros::ok())
  {
    geometry_msgs::Twist cmd_vel;
    vel_pub_1.publish(cmd_vel);
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = 0;

  int state = 1;
  switch(state)
  {
   case 1 : cmd_vel.linear.x = 3 ;
            if(counter==20) {state = 2 ; counter = 0 ;} 
            else counter++ ;
            break ;
   case 2 : cmd_vel.linear.y = -3 ;
            if(counter==20) {state = 3 ; counter = 0 ; } 
            else counter++ ;
            break ;
   case 3 : cmd_vel.linear.x = -3 ;
            if(counter==20) {state = 4 ; counter = 0 ;} 
            else counter++ ;
            break ;
   case 4 : cmd_vel.linear.y = 3 ;
            if(counter==20) {state = 5 ; counter = 0 ; } 
            else counter++ ;
            break ;
   case 5 : cmd_vel.linear.x = -3 ;
            if(counter==20) {state = 6 ; counter = 0 ; } 
            else counter++ ;
            break ;
   case 6 : cmd_vel.linear.y = -3 ;
            if(counter==20) {state = 7 ; counter = 0 ; } 
            else counter++ ;
            break ;
  case 7 :  cmd_vel.linear.x = 3 ;
            if(counter==20) {state = 8 ; counter = 0 ; } 
            else counter++ ;
            break ;
  case 8 :  cmd_vel.linear.y= 3 ;
            if(counter==20) {state = 1 ; counter = 0 ; } 
            else counter++ ;
            break ;
  }
  
  if(counter==0)
    if(state<4)cmd_vel.angular.z = 15;
    else cmd_vel.angular.z = -1*15;
  vel_pub_1.publish(cmd_vel);
    ros::spinOnce();

    loop_rate.sleep();

    ++count;
  }
  return 0;
}

*/
