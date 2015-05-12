#include "ros/ros.h"
#include <math.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include <utility>
#include <fstream>
#include <vector>
#include <ros/console.h>

#define circular_arc
#define TOL 0.5
#define pb push_back
#define mk std::make_pair
geometry_msgs::Pose current;
geometry_msgs::Pose initial;
bool flag = true;
 
#define arenalen 2200
#define simulen 480
#define PI 3.14
#define MAX_BOT_LINEAR_VEL_CHANGE 18 // or 3 for real case 
#define MAX_BOT_SPEED 5// 80.0 for real case
//#define intialAng PI/4   // set the value 
#define THRESH 5
#define angle_thresh 0.2*PI/4

using namespace std ;

double v , w ;

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
 inline float normalizeAngle(float angle)
{
  if (angle > PI)
    return angle - 2 * PI;
  else if (angle <= -PI)
    return angle + 2 * PI;
  return angle;
}
void PolarBased(std::pair<int,int> s, std::pair<int,int> e,double etheta, double stheta,  float &vl, float &vr, double prevSpeed)
{
    
    /*static const double d          = 12.2*(arenalen/simulen);  //currentDistance between wheels in cm
    static const double ticksToCmS = 1.08; //still only approximate...
    static const double fieldXConvert = 29.75;     // 
    static const double fieldYConvert = 27.33333;
    static const double xUncertainty = 0;//0.5; // Uncertainty is in %age of max value. eg. 1% where fabs(x) <= 1000 means fabs(error) <= 10
    static const double yUncertainty = 0;//0.5;
    static const double thetaUncertainty = 0;//3;
    const int timeLCMs = 16;
    const double timeLC = timeLCMs*0.001;
    const int NUMTICKS = 300;
    static long int ticks = 0;
    */
    // NOTE: its preferable to call x(), y(), and theta() of each object exactly once since they may return different
    // values on each call.


    //******************************************
    std::pair<int,int> initial= std::make_pair(s.first-e.first, s.second-e.second);
    //double etheta = 0//e.theta();
    double theta = normalizeAngle(etheta - stheta);//s.theta
   // rotate initial by -e.theta degrees;
    #ifdef arpit
    double newx = initial.first * cos(-etheta) - initial.second * sin(-etheta);
    double newy = initial.first * sin(-etheta) + initial.second * cos(-etheta);
    initial = std::make_pair(newx, newy);
    #endif
    double rho = sqrt(initial.first*initial.first + initial.second*initial.second);
    double gamma = normalizeAngle(atan2(initial.second, initial.first) - theta + PI);
    double delta = normalizeAngle(gamma + theta);
    double k1 = 0.5 , k2 =4 , k3 = 20;    // tuning parameters ..................
    v = k1*rho*cos(gamma);
    
    if(gamma == 0) 
        w = k2*gamma+k1*cos(gamma)*(gamma+k3*delta);
    else 
        w = k2*gamma+k1*sin(gamma)*cos(gamma)/gamma*(gamma + k3*delta);

    if(abs(v)>MAX_BOT_SPEED)
      v = (v/abs(v))*MAX_BOT_SPEED ;

    if(abs(v/w)>=5)
    {
       v = (v/abs(v))*1 ;
       w = (w/abs(w))*1 ;  
    }
    else
      if(abs(v/w)<0.2)
      {
        v= (v/abs(v))*MAX_BOT_SPEED ;
        w = (w/abs(w))*1 ;
      }
    /*
    vl = v - d*w/2;
    vr = v + d*w/2;
    double timeMs = 0.07*rho + 12 * sqrt(rho); // empirical
    double speed = timeMs/timeLCMs<(prevSpeed/MAX_BOT_LINEAR_VEL_CHANGE)?prevSpeed-MAX_BOT_LINEAR_VEL_CHANGE:prevSpeed+MAX_BOT_LINEAR_VEL_CHANGE;
    if(speed > MAX_BOT_SPEED)
        speed = MAX_BOT_SPEED;
    else if (speed < 0)
        speed = 0;
    double max = abs(vl)>abs(vr)?abs(vl):abs(vr);
    if(max > 0) {
        vl = (vl)*speed/max;
        vr = (vr)*speed/max;
  int currentDist = sqrt((e.first-s.first)^2 + (e.second-s.second)^2) ;
  ticks++;
  }
  */
}
int main(int argc, char **argv)
{
 
  vector< pair<int,int> > points;
  ros::init(argc, argv, "SwarmSimu");
 
  ros::NodeHandle n;
 
  ros::Publisher vel_pub_0 = n.advertise<geometry_msgs::Twist>("/swarmbot0/cmd_vel", 1);
  ros::Subscriber odom_data = n.subscribe("/swarmbot0/odom",10,odomCB);
  // ros::Publisher vel_pub_1 = n.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1);
  ros::Rate loop_rate(5);
  ROS_DEBUG("Hello %s", "World");
  int count = 0;
 
  geometry_msgs::Twist cmd_vel;
  cmd_vel.angular.z = 0;//0.4;
 
  bool flipped = false;
  float vl = 0 , vr  = 0 ;
  int posX,posY,posZ , finalX , finalY;   // it was initially double
  std::pair<int,int> currentPos , finalPos ;
  double d          = 12.2*(arenalen/simulen);
  float v_x , v_y , v_t ;
  int flag1 = 0 ;
     points.push_back(mk(initial.position.x + 20 , initial.position.y - 10   )) ;
     points.push_back(mk(initial.position.x + 30 , initial.position.y - 30 ));
  //   points.push_back(mk(initial.position.x - 20 , initial.position.y + 10 )) ;
  while (ros::ok())
  {
   
    nav_msgs::Odometry odom;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = 0 ;
    // cmd_vel.angular.z = 0.4*sin(count);
 
  std::ofstream myfile;
  myfile.open ("example.txt");

    currentPos.first = current.position.x ; currentPos.second = current.position.y ;
    finalPos.first =  10 ; finalPos.second = -28;
    float initialAng = 2*acos(current.orientation.w) ;
    int currentDist = (int)(sqrt(pow((finalPos.second - currentPos.second),2)+pow((finalPos.first-currentPos.first),2))) ;
    float currentSlope = atan((finalPos.second - currentPos.second)/(finalPos.first - currentPos.first)) ;
   // initialAng = normalizeAngle(initialAng) ;
    
    #ifdef polarbased
    PolarBased(std::make_pair(currentPos.first,currentPos.second ),std::make_pair(finalPos.first,finalPos.second),initialAng,0,vl,vr,(double)((vl+vr)*0.5));
    
    if(currentDist>THRESH){  
     ROS_INFO("v :: %lf , w :: %lf ",v,w);
     cmd_vel.linear.x = v ;//(vl + vr)*0.5 ;
     cmd_vel.angular.z = w ; //(vl - vr)/d ;
    }
    #endif
    
    #ifdef circular_arc

    // if(currentDist < 0.5*sin(0.5*PI/8))
   //   currentDist = 0.5*sin(0.5*PI/8) ;

    float radius = currentDist/(2*sin(0.5*PI/8)) ;
    v_x = 3;
    v_y = 0 ; 
    if(radius < 0.1)
      radius = 0.1 ;
    v_t = v_x/(radius); 
    if(currentDist>THRESH) {
     cmd_vel.linear.x = v_x  ;
     cmd_vel.angular.z = v_t ;
      if(finalPos.second > currentPos.second )
        cmd_vel.angular.z = -1*cmd_vel.angular.z ;
     // if(currentPos.first < finalPos.second )
     //   cmd_vel.linear.x = -1*cmd_vel.linear.x ;
      
    }
    else
    {
      flag1++ ;
      if(flag1>3)
          flag1 = 3 ;
    }
   printf("v_x :: %lf , w :: %lf \n",cmd_vel.linear.x,cmd_vel.angular.z) ;
  //  ROS_INFO("v_x :: %lf , w :: %lf \n",cmd_vel.linear.x,cmd_vel.angular.z)
    #endif

    #ifdef testing
    v_x = 1.5 ;
    if(currentDist>THRESH) 
       v_x = 1.5 ;
    else
       v_x = 0 ;
    v_t = -1*v_x*currentSlope/(currentDist)  ;
    cmd_vel.linear.x = v_x  ;
    cmd_vel.angular.z = v_t ;
    #endif 
   

    /*  if((initialAng < currentSlope - angle_thresh ) || (initialAng > currentSlope + angle_thresh ))
      cmd_vel.angular.z = 0.6 ;
  */

/*
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
    */
    vel_pub_0.publish(cmd_vel);
    ros::spinOnce();
    loop_rate.sleep();
 
    ++count;
  }
  return 0;
}