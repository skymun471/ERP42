#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose2D.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <math.h>

#define MAX_L_STEER -2000
#define MAX_R_STEER 2000
#define STEER_NEUTRAL_ANGLE 0

#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180.*M_PI)

#define WayPoint_Tor 1.0


double cvr_value = 0.0;
double roll,pitch,yaw;
int steer_angle = STEER_NEUTRAL_ANGLE;

sensor_msgs::NavSatFix covariance_value;
geometry_msgs::Pose2D goal_pose;
geometry_msgs::Pose2D my_pose;

int vision_steering_angle = 0;
int waypoint_steering_angle = 0;
int car_speed = 30;

struct Point
{
	float x;
	float y;
	float z;
};

struct WayPoints
{
	double x;
	double y;
};

void poseCallback(const geometry_msgs::PoseStamped& msg)
{
	my_pose.x = (double)msg.pose.position.x;
	my_pose.y = (double)msg.pose.position.y;
}

void Position_Callback(const sensor_msgs::NavSatFix& m)
{
	covariance_value = m;
	cvr_value = covariance_value.position_covariance[0];
}

void goalCallback(const geometry_msgs::Pose2D& msg)
{
	goal_pose.x = (double)msg.x;
	goal_pose.y = (double)msg.y;
}

void YAWCallback(const geometry_msgs::QuaternionStamped& q){
	// my_pose.theta = (360-msg.data) % 360; // e2box IMU version
  my_pose.theta = q.quaternion.w;

  //printf("yaw : %lf \n", msg.data);
  
}

int main(int argc, char **argv)
{
  char buf[2];
  ros::init(argc, argv, "race_waypoints_navigation");
  ros::NodeHandle n;

  ros::Subscriber sub1 = n.subscribe("/utm",10, &poseCallback);
  ros::Subscriber sub2 = n.subscribe("/ublox_gps/fix", 1, &Position_Callback);
  ros::Subscriber sub3 = n.subscribe("/filter/quaternion",5, &YAWCallback);
  ros::Subscriber sub4 = n.subscribe("/goal_pose",1, &goalCallback);

  ros::Publisher car_control_pub1 = n.advertise<std_msgs::Int16>("Car_Control_cmd/SteerAngle_wp_Int16", 10);
  ros::Publisher car_control_pub2 = n.advertise<std_msgs::Int16>("Car_Control_cmd/", 10);
  ros::Publisher car_do_waypoint = n.advertise<std_msgs::Int16>("/wp_go", 10);
  
  ros::Rate loop_rate(5);

  std_msgs::Int16 s_angle;
  std_msgs::Int16 c_speed;
  std_msgs::Int16 wp_go;
  wp_go.data = 0;

  double pos_error_x = 10.0;
  double pos_error_y = 10.0;

  int error, error_old, error_d, error_sum, theta_star;
  double Kp,Ki,Kd;

  error = error_old = error_d = error_sum = 0.0;
  Kp = 1.0;
  Ki = 0.0;
  Kd = 0.00;

  while (ros::ok())
  {
    double tf_base_map_x,tf_base_map_y;
		
    if(my_pose.x == 0.0 && my_pose.y == 0.0){
      pos_error_x = 100;
		  pos_error_y = 100;
    }
    else{
      pos_error_x = abs(my_pose.x - goal_pose.x);
		  pos_error_y = abs(my_pose.y - goal_pose.y);
    }
    
    if(sqrt(pow(pos_error_x,2) + pow(pos_error_y,2)) <= WayPoint_Tor)
		{
			wp_go.data++;
		}
    
    tf_base_map_x = goal_pose.x - my_pose.x;
	  tf_base_map_y = goal_pose.y - my_pose.y;
    
    theta_star = (int)RAD2DEG(atan2f(tf_base_map_y,tf_base_map_x));
    printf("theta_star before: %d\n",theta_star);
    if(theta_star < 0) theta_star+=360;
    printf("theta_star after: %d\n",theta_star);

	  error = theta_star - my_pose.theta;

    error_sum += error;
    error_d = error - error_old;
    error_old = error;

    steer_angle = Kp*error + Ki*error_sum + Kd*error_d;
    if(steer_angle < -180) steer_angle += 360;
    else if(steer_angle > 180) steer_angle -=360;

    s_angle.data =  steer_angle;
		c_speed.data = car_speed;
		
    printf("my pose : %lf %lf \n", my_pose.x,my_pose.y);
    printf("theta : %lf \n",my_pose.theta);
		printf("WayPoint-%d\n",wp_go.data);
		printf("Covariance : %6.3lf \n", cvr_value);
    printf("angle : %d \n", s_angle.data);
    

		car_control_pub1.publish(s_angle);
		car_control_pub2.publish(c_speed);
		car_do_waypoint.publish(wp_go);

		loop_rate.sleep();
		ros::spinOnce();
  }
  return 0;
}