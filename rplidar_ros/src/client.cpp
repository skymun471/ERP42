#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int16.h"
#include "math.h"

#define debug 1
#define RAD2DEG(x) ((x)*180./M_PI) // rad -> deg

std_msgs::Int16 obj_detect;
std_msgs::Int16 offset;

int steerangle_tem = 0;
int detect_angle_P = 0;
int detect_angle_M =0;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    double F_RANGE = 0.7, J_SIZE = 0.5, SIDE_RANGE = 1.2;// 정면거리 , 좌우 폭, 좌우 detection 거리
    float lidar_angle = RAD2DEG(atan2((J_SIZE/2),F_RANGE)); // 역 tan를 사용하여 각도 계산
    float L_count=1,L_degree=73,L_range=1.0,R_count=1,R_degree=-73,R_range=1.0,C_count=0,C_degree=0,C_range=0; //좌우에 가상으로 장애물 하나씩
    int count = scan->scan_time / scan->time_increment;
	detect_angle_P = -lidar_angle+steerangle_tem;
	detect_angle_M = lidar_angle-steerangle_tem;
	if(detect_angle_P > 75) detect_angle_P = 75;
	if(detect_angle_M < -75) detect_angle_M = -75;
	
    for(int i = 0; i < count; i++) {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i); // rad -> deg하여 각도 구하기

        if(scan->ranges[i] <= SIDE_RANGE){
            if(degree > 10 && degree <= 75.0 ){
                L_count++;
                L_degree += degree;
                L_range += scan->ranges[i];
            }
            if(degree < -10 && degree >= -75.0){
                R_count++;
                R_degree += degree;
                R_range += scan->ranges[i];
            }
        }
            if(degree > detect_angle_P && degree < detect_angle_M && scan->ranges[i] <= F_RANGE){
                C_count++;
                C_degree += degree;
                C_range += scan->ranges[i];
            }


    }
    if(debug){
        if(L_count > 0) // detect left side obj
        {
            //ROS_INFO("L_OBSTACLE");
            //ROS_INFO(": [%.2f][%.2fm]", L_degree/(int)L_count , L_range/(int)L_count);
        }
        if(R_count > 0) // detect right side obj
        {
            //ROS_INFO("R_OBSTACLE");
            //ROS_INFO(": [%.2f][%.2fm]", R_degree/(int)R_count , R_range/(int)R_count);
        }
    }
    if(C_count != 0) //detect center obj
    {
        ROS_INFO("Center_OBSTACLE");
        ROS_INFO(": [%.2f][%.2fm]", C_degree/(int)C_count , C_range/(int)C_count);
        obj_detect.data = 1;
    }
    else
    {
		obj_detect.data = 0;
	}

    offset.data = (L_range/(int)L_count-R_range/(int)R_count)*200;
}

void SteerAngleCallback(const std_msgs::Int16& msg)
{
  steerangle_tem = msg.data;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "rplidar_node_client");
    ros::NodeHandle nh;

    ros::Publisher pub_result = nh.advertise<std_msgs::Int16>("/obj_detect",1);
    ros::Publisher pub_offset = nh.advertise<std_msgs::Int16>("/lidar_offset",1);

    ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);
    ros::Subscriber sub_steeranlge = nh.subscribe("Car_Control_cmd/SteerAngle_Int16", 1, &SteerAngleCallback);

    obj_detect.data = 0;
    steerangle_tem = 0;
    while(ros::ok()){
		ros::spinOnce();
		pub_result.publish(obj_detect);
        pub_offset.publish(offset);
	}

    return 0;
}
