#include "ros/ros.h"
#include "std_msgs/Int16.h"

#include <stdio.h>
#include <stdlib.h>

int Base_Speed = 3;

int steerangle_vision = 0;
int speed_vision = Base_Speed;
int darknet_flag = 0;
int obj_detect = 0;
int lidar_offset = 0;
int mode_flag = 1;
int flag = 0;
int steerangle_wp = 0, speed_wp =0;
int stopline_flag = 0;

int wp = 0;


void SteerAngle_vision_Callback(const std_msgs::Int16& msg)
{
  steerangle_vision = msg.data;
}

void darknetCallback(const std_msgs::Int16& msg)
{
	darknet_flag  = msg.data;
  ROS_INFO("%d", darknet_flag);
}

void objdetectCallback(const std_msgs::Int16& msg)
{
	obj_detect = msg.data;
}
void lidaroffsetCallback(const std_msgs::Int16& msg)
{
  lidar_offset = msg.data;
}

void SteerAngle_WP_Callback(const std_msgs::Int16& msg)
{
  steerangle_wp = -msg.data;
  //ROS_INFO("wp_angle: %d", steerangle_wp);
}

void Speed_WP_Callback(const std_msgs::Int16& msg)
{
  speed_wp = msg.data;
}

void WP_Callback(const std_msgs::Int16& msg)
{
  wp = msg.data;
}

void stopline_Callback(const std_msgs::Int16& msg)
{
  stopline_flag = msg.data;
  
}


int main(int argc, char **argv)
{
  ROS_INFO("main node start!!\n");
  ros::init(argc, argv, "main_node");
  ros::NodeHandle nh;

  ros::Publisher pub_steerangle = nh.advertise<std_msgs::Int16>("/Car_Control_cmd/SteerAngle_Int16", 1);
  ros::Publisher pub_speed = nh.advertise<std_msgs::Int16>("/Car_Control_cmd/Speed_Int16",1);

  ros::Subscriber sub_steeranlge = nh.subscribe("/SteerAngle_Int16", 1, &SteerAngle_vision_Callback);
  ros::Subscriber sub_darknet = nh.subscribe("/darknet_result",1,&darknetCallback);
  ros::Subscriber sub_lidar = nh.subscribe("/obj_detect",1,&objdetectCallback);
  ros::Subscriber sub_offset = nh.subscribe("/lidar_offset",1,&lidaroffsetCallback);
  ros::Subscriber sub_steeranlge_wp = nh.subscribe("Car_Control_cmd/SteerAngle_wp_Int16",1, &SteerAngle_WP_Callback);
  ros::Subscriber sub_speed_wp = nh.subscribe("Car_Control_cmd/Speed_wp_Int16",1, &Speed_WP_Callback);
  ros::Subscriber sub_wp = nh.subscribe("/wp_go",1, &WP_Callback);
  ros::Subscriber sub_stopline = nh.subscribe("/stop_line_flag",1, &stopline_Callback);

  std_msgs::Int16 steerangle;
  std_msgs::Int16 speed;

  ros::Duration wait_settle(0.8);

  ros::Rate loop_rate(10);



  while (ros::ok())
  {
    ros::spinOnce();
  //red =1, green=2, cpz_on=3, cpz_off=4, stop=5, construction=6
  if(wp < 2){
	  if((darknet_flag == 1) && flag==0 && stopline_flag == 1){
        speed.data = 0;
        pub_speed.publish(speed);
        flag=1;
      }
      else if((darknet_flag == 1) && flag==1){
        speed_vision = 0;
      }
      else if(darknet_flag == 2 || darknet_flag == 4){
        speed_vision = Base_Speed;
        flag = 0;
      }
      else if(darknet_flag == 3){
        speed_vision = 2;
      }
      else if(darknet_flag == 5){
        speed_vision = 0;
      }
      else if(darknet_flag == 6){
        speed.data = Base_Speed;
        steerangle_vision = -0;
        pub_steerangle.publish(steerangle);
        flag=1;
        wait_settle.sleep();
      }
      steerangle.data = steerangle_vision;
      speed.data = speed_vision;
    }
    else{
	  steerangle.data = steerangle_wp;
    speed.data = speed_wp;
	}

    if(mode_flag == 0){
      steerangle_vision = lidar_offset;
    }

	//publish data
    pub_steerangle.publish(steerangle);
    pub_speed.publish(speed);

    loop_rate.sleep();
  }
  return 0;
}
