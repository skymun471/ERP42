#define DEBUG 0
#define DEBUG_ROS_INFO 0
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Range.h"

/*
? ? : I2C? ???? ???? ???? ????.
*/
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

#include <sstream>

//i2c address
#define ADDRESS 0x05

//I2C bus
static const char *deviceName = "/dev/i2c-0";

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

int  Base_Speed = 0;

int steering_angle = STEER_NEUTRAL_ANGLE;
int motor_speed = Base_Speed;

int steering_angle_old =  STEER_NEUTRAL_ANGLE;
int motor_speed_old = Base_Speed;

unsigned char protocol_data[7] = {'#',0,0,0,0,0,'*'}; // start byte '#' - end bytte '*'

int file_I2C;


int open_I2C(void)
{
   int file;

    if ((file = open( deviceName, O_RDWR ) ) < 0)
    {
        fprintf(stderr, "I2C: Failed to access %s\n", deviceName);
        exit(1);
    }
    printf("I2C: Connected\n");


    printf("I2C: acquiring buss to 0x%x\n", ADDRESS);
    if (ioctl(file, I2C_SLAVE, ADDRESS) < 0)
    {
        fprintf(stderr, "I2C: Failed to acquire bus access/talk to slave 0x%x\n", ADDRESS);
        exit(1);
    }

    return file;
}

/*
?? : ?????? ???.
*/

void close_I2C(int fd)
{
   close(fd);
}

void CarControlCallback(const geometry_msgs::Twist& msg)
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());

   steering_angle = (int)(msg.angular.z+STEER_NEUTRAL_ANGLE) ;


   if(steering_angle >= MAX_R_STEER)  steering_angle = MAX_R_STEER;
   if(steering_angle <= MAX_L_STEER)  steering_angle = MAX_L_STEER;

   Base_Speed  = (int)msg.linear.x;
   motor_speed = Base_Speed;
   if(motor_speed>=250)   motor_speed = 250;
   if(motor_speed<=-250)  motor_speed = -250;

}

void CarSteerControlCallback(const std_msgs::Int16& angle)
{
  steering_angle = (int)(angle.data) ;

  if(steering_angle >= MAX_R_STEER)  steering_angle = MAX_R_STEER;
  if(steering_angle <= MAX_L_STEER)  steering_angle = MAX_L_STEER;
}

void CarSpeedControlCallback(const std_msgs::Int16& speed)
{
  motor_speed = (int)(speed.data);

  Base_Speed = motor_speed;
  if(motor_speed>=250)   motor_speed = 250;
  if(motor_speed<=-250)  motor_speed = -250;
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

  ros::Subscriber sub1 = n.subscribe("/cmd_vel", 10, &CarControlCallback);
  ros::Subscriber sub2 = n.subscribe("/Car_Control_cmd/SteerAngle_Int16",10, &CarSteerControlCallback);
  ros::Subscriber sub3 = n.subscribe("/Car_Control_cmd/Speed_Int16",10, &CarSpeedControlCallback);

  ros::Rate loop_rate(20);  // 10
  file_I2C = open_I2C();
  if(file_I2C < 0)
  {
	  ROS_ERROR_STREAM("Unable to open I2C");
	  return -1;
  }
  else
  {
	  ROS_INFO_STREAM("I2C is Connected");
  }
  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */

  while (ros::ok())
  {

	  //if(motor_speed > 0) motor_speed = Base_Speed-0.1*abs(steering_angle);

    std_msgs::String msg;
		std_msgs::Int16 steerangle;
		std_msgs::Int16 carspeed;
		std::stringstream ss;
		std::string data;
		data = std::to_string(steering_angle);

		steerangle.data = steering_angle;
		carspeed.data = motor_speed;
		ss<<data;
		msg.data = ss.str();

		data = std::to_string(motor_speed);
		msg.data = data;

		protocol_data[0] = '#';
		protocol_data[1] = 'C';
		protocol_data[2] = (steering_angle&0xff00)>>8 ;
		protocol_data[3] = (steering_angle&0x00ff);
		protocol_data[4] = (motor_speed&0xff00)>>8 ;
		protocol_data[5] = (motor_speed&0x00ff);
		protocol_data[6] = '*';
		write(file_I2C, protocol_data, 7);
		read(file_I2C,buf,8);
		ROS_INFO("SPEED: %d\tANGLE: %d\n", motor_speed,steering_angle);

        protocol_data[0] = '#';
		protocol_data[1] = 'S';
		protocol_data[2] = (steering_angle & 0xff00)>>8 ;
		protocol_data[3] = (steering_angle & 0x00ff);
		protocol_data[4] = (motor_speed & 0xff00)>>8 ;
		protocol_data[5] = (motor_speed & 0x00ff);
		protocol_data[6] = '*';
		write(file_I2C, protocol_data, 7);
		read(file_I2C,buf,8);
    //decoding
    yaw.data = -1.0*((buf[1]*256+buf[2])-180);
    pub_yaw.publish(yaw);
	ROS_INFO("%d", yaw.data);
		loop_rate.sleep();
		ros::spinOnce();
  }


  motor_speed = 0;
  //steering_angle = 0; ban!
  protocol_data[0] = '#';
  protocol_data[1] = 'C';
  protocol_data[2] = (steering_angle&0xff00)>>8;
  protocol_data[3] = (steering_angle&0x00ff);
  protocol_data[4] = (motor_speed&0xff00)>>8;
  protocol_data[5] = (motor_speed&0x00ff);
  protocol_data[6] = '*';
  write(file_I2C, protocol_data, 7);
  close_I2C(file_I2C);
  sleep(2000);
  printf("i2c disconnected\n");
  return 0;
}
