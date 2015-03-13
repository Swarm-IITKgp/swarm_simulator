#include "ros/ros.h"
#include <math.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "gazebo_msgs/ModelState.h"
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/GetWorldProperties.h>
#define des_x 0
#define des_y -15
#define des_o 0
#define cor_x (getmodelstate.response.pose.position.x - des_x)
#define cor_y (getmodelstate.response.pose.position.y - des_y)
#define or_x getmodelstate.response.pose.orientation.x
#define or_y getmodelstate.response.pose.orientation.y
#define or_z getmodelstate.response.pose.orientation.z
#define or_w getmodelstate.response.pose.orientation.w 
#define cor_o normalizeangle((2*acos(or_w)-des_o))
#define min_bot_speed 0.5
#define max_bot_speed 10
#define bot_point_thres 0.1
#define lambda 2
#define beta 0.7
#define k1 .75
#define k2 4
#define k3 20
#define pi 3.14159

/**
 * This tutorial demonstrates to move bot along a smooth trajactory between 2 points in Gazebo.
 * Initial point (-12,-12,0) --> Final Position (-0,-15,0)
 */

/* Class to convert xyz coordinates to pyd and give v and w */
double normalizeangle(double theta)
{
return (theta-2*pi*floor((theta+pi)/(2*pi)));
}
class polar_Cordinate
{

float p;
float y;
float d;
public:
polar_Cordinate(){}
void set_values(double a,double b, double c)
{
	p=sqrt(a*a+b*b);
	y=normalizeangle(atan2(b,a)-c+pi);
	d=y+c;
}
double get_v()
{
	return (k1*p*cos(y));
}
double get_w()
{
     if(y==0)
	return k2*y+k1*cos(y)*(y+k3*d);
     else
	return k2*y+k1*sin(y)*cos(y)*(y+k3*d)/y;
		
}
double get_p() {return p;}
double get_y() {return y;}
double get_d() {return d;}
};


int main(int argc, char **argv)
{

  ros::init(argc, argv, "SwarmSimu");

  ros::NodeHandle n;

  ros::Publisher vel_pub_0 = n.advertise<geometry_msgs::Twist>("/swarmbot0/cmd_vel", 1);
  // ros::Publisher vel_pub_1 = n.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1);
  ros::Rate loop_rate(50);

ros::ServiceClient serv_client = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state"); 
  gazebo_msgs::GetModelState getmodelstate;
  gazebo_msgs::ModelState modelstate;

  std::string s="swarmbot0";
  polar_Cordinate pol_c;
  int count = 0;

  while (ros::ok()&&((pol_c.get_p()>0.5||cor_o >0.1)||count<50)){
    geometry_msgs::Twist cmd_vel;
    getmodelstate.request.model_name=s;
    serv_client.call(getmodelstate);
    pol_c.set_values(cor_x,cor_y,cor_o);
    double k= 20*(pol_c.get_w()/pol_c.get_v());
   // cmd_vel.linear.x = max_bot_speed/(1+beta*pow(fabs(k),lambda));
//if(cmd_vel.linear.x<min_bot_speed)cmd_vel.linear.x=min_bot_speed;
    cmd_vel.linear.x=pol_c.get_v();
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = pol_c.get_w();
if(count%10==0)
    ROS_INFO("%lf,%lf,%lf,%lf ,%lf,%lf,%lf,%lf ",cor_x,cor_y,cor_o,pol_c.get_p(),pol_c.get_y(),pol_c.get_d(),pol_c.get_v(),pol_c.get_w());
    vel_pub_0.publish(cmd_vel);
count++;
    ros::spinOnce();
 loop_rate.sleep();
}


   geometry_msgs::Twist cmd_vel;

    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = 0;
vel_pub_0.publish(cmd_vel);

  return 0;
}
