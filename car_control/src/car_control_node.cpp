#define DEBUG 0
#define DEBUG_ROS_INFO 0
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Range.h"
#include <sensor_msgs/Joy.h>

#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

#define MAX_L_STEER -40
#define MAX_R_STEER 40
#define STEER_NEUTRAL_ANGLE 0

#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180.*M_PI)

#define Object_Avoid_Range_1 0.5
#define Object_Avoid_Range_2 0.7
#define LIDAR_Obastacle   0.6
#define LIDAR_Wall_Detection 0.30

#define delay  1

std_msgs::Int16 yaw;
std_msgs::Int16 steerangle;
std_msgs::Int16 carspeed;

int  Base_Speed = 4;

int steering_angle_controller = STEER_NEUTRAL_ANGLE;
int motor_speed_controller = 0;
int steering_angle_main = STEER_NEUTRAL_ANGLE;
int motor_speed_main = 0;

int mode = 0;
int aeb_flag = 0;

void JoystickCallback(const sensor_msgs::Joy& msg)
{
  if(msg.buttons[1]==1) mode = 0;
  else if(msg.buttons[0]==1) mode = 1;
  else if(msg.buttons[3]==1) mode = 2;
  motor_speed_controller = msg.axes[3]*Base_Speed;
  steering_angle_controller = msg.axes[0]*MAX_R_STEER;
  if(msg.buttons[6]==1) aeb_flag =1;
}

void CarSteerControlCallback(const std_msgs::Int16& angle)
{
  steering_angle_main = (int)(angle.data) ;

  if(steering_angle_main >= MAX_R_STEER)  steering_angle_main = MAX_R_STEER;
  if(steering_angle_main <= MAX_L_STEER)  steering_angle_main = MAX_L_STEER;
}

void CarSpeedControlCallback(const std_msgs::Int16& speed)
{
  motor_speed_main = (int)(speed.data);

  if(motor_speed_main>=250)   motor_speed_main = 250;
  if(motor_speed_main<=-250)  motor_speed_main = -250;
}

void ArduinoIMUCallback(const std_msgs::Int16& msg)
{
	yaw.data = msg.data;
  //ROS_INFO("%d",  msg.data);
}


int main(int argc, char **argv)
{
  char buf[2];
  ros::init(argc, argv, "car_control");

  ros::NodeHandle n;
  std::string cmd_vel_topic = "cmd_vel";
  /*other*/
  ros::param::get("~cmd_vel_topic", cmd_vel_topic);

  ros::Publisher pub_yaw = n.advertise<std_msgs::Int16>("yaw", 1);
  ros::Publisher pub_speed = n.advertise<std_msgs::Int16>("arduino_speed", 1);
  ros::Publisher pub_angle = n.advertise<std_msgs::Int16>("arduino_angle", 1);

  ros::Subscriber sub1 = n.subscribe("joy", 1, &JoystickCallback);
  ros::Subscriber sub2 = n.subscribe("/Car_Control_cmd/SteerAngle_Int16",10, &CarSteerControlCallback);
  ros::Subscriber sub3 = n.subscribe("/Car_Control_cmd/Speed_Int16",10, &CarSpeedControlCallback);
  ros::Subscriber sub4 = n.subscribe("IMU_yaw",10, &ArduinoIMUCallback); //test

  ros::Rate loop_rate(10);  // 10
  
  
  while (ros::ok())
  {
	  steerangle.data = steering_angle_main;
	  carspeed.data = motor_speed_main;
    
    if(mode==0){
      steerangle.data = steering_angle_controller;
	    carspeed.data = motor_speed_controller;
    }
    else if(mode==1){
      steerangle.data = steering_angle_main;
	    carspeed.data = motor_speed_controller;
    }
    else if(mode==2){
      steerangle.data = steering_angle_main;
	    carspeed.data = motor_speed_main;
    }
    if(aeb_flag==1){
      steerangle.data = 0;
      carspeed.data = 0;
      mode=0;
      aeb_flag=0;
    }
    ROS_INFO("angle = %d",steerangle.data);
    ROS_INFO("speed = %d",carspeed.data);
    //ROS_INFO("mode = %d",mode);

    pub_angle.publish(steerangle);
    pub_speed.publish(carspeed);
    pub_yaw.publish(yaw);

	  loop_rate.sleep();
	  ros::spinOnce();
  }
  
  steerangle.data = 0;
  carspeed.data = 0;
  
  pub_angle.publish(steerangle);
  pub_speed.publish(carspeed);
  
  return 0;
}
