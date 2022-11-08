#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"
#include <math.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>

ros::Publisher pub_avoid;
ros::Publisher pub_stay_right;
ros::Publisher pub_stay_left;
std_msgs::Int16 avoid; // 1 = left , 2 = right
std_msgs::Int16 stay_right; // 1 = stay 0 = gps
std_msgs::Int16 stay_left; // 1 = stay 0 = gps

pcl::PointCloud<pcl::PointXYZI> cloud_copy;

double x_right = 0;
double x_left = 0;
double y_right = 0;
double y_left = 0;
double num_right = 0;
double num_left = 0;
int mode = 0;
double right_angle = 0;
double left_angle = 0;
int avoid_right_point = 0;
int avoid_left_point = 0;

void Point_cal (const sensor_msgs::PointCloud2 input)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(input, *input_cloud);
    cloud_copy = *input_cloud;
}

void Obstacle_avoidance(const std_msgs::Int16& msg)
{
    mode = msg.data;
    if(mode == 1 && avoid.data == 0){
        double size = cloud_copy.size();
        for(double i=0;i<size;i++){
            if(i==0){
            x_left = cloud_copy.points[i].x;
            x_right = cloud_copy.points[i].x;
            }
    
            if(x_left <= cloud_copy.points[i].x){
            x_left = cloud_copy.points[i].x;
            num_left = i;
            }

            if(x_right >= cloud_copy.points[i].x){
            x_right = cloud_copy.points[i].x;
            num_right = i;
            }
        }  

        y_right = cloud_copy.points[num_right].y;
        y_left = cloud_copy.points[num_left].y;

        right_angle = ((atan2(y_right, x_right))*(180/M_PI)) + 90.0;
        left_angle = ((atan2(y_left, x_left))*(180/M_PI)) + 90.0;
        if(fabs(left_angle) < fabs(right_angle)){
            avoid.data = 1; // go left
        }
        if(fabs(left_angle) > fabs(right_angle)){
            avoid.data = 2; // go right
        }
        
    }
    if(mode == 3 || mode == 0 || mode == 2){
        right_angle = 0;
        left_angle = 0;
    }
    std::cout << "mode : " << mode << std::endl << "avoid : " << avoid << std::endl;
    pub_avoid.publish(avoid);
}

void Rigth_avoid(const std_msgs::Int32& msg)
{
    avoid_right_point = msg.data;
    if(mode == 0 && avoid_right_point != 0){
        if(avoid.data == 2){
            stay_right.data = 1;
            avoid.data = 2;
        }
    }
    if(mode == 0 && avoid_right_point == 0){
        avoid.data = 0;
        stay_right.data = 0;
    }
    pub_avoid.publish(avoid);
    pub_stay_right.publish(stay_right);
}

void Left_avoid(const std_msgs::Int32& msg)
{
    avoid_left_point = msg.data;
    if(mode == 0 && avoid_left_point != 0){
        if(avoid.data == 1){
            stay_left.data = 1;
            avoid.data = 1;
        }
    }
    if(mode == 0 && avoid_left_point == 0){
        avoid.data = 0;
        stay_left.data = 0;
    }
    pub_avoid.publish(avoid);
    pub_stay_left.publish(stay_left);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_avoidance");
    ros::NodeHandle node;
    ros::Subscriber sub_mode = node.subscribe ("/ob_mode", 1, Obstacle_avoidance);
    ros::Subscriber sub_right = node.subscribe ("/avoid_to_right", 1, Rigth_avoid);
    ros::Subscriber sub_left = node.subscribe ("/avoid_to_left", 1, Left_avoid);
    ros::Subscriber sub_av = node.subscribe ("/out_put", 1, Point_cal);
    pub_avoid = node.advertise<std_msgs::Int16>("/obstacle_avoidance",1);
    pub_stay_right = node.advertise<std_msgs::Int16>("/stay_right",1);
    pub_stay_left = node.advertise<std_msgs::Int16>("/stay_left",1);
    ros::spin();
    return 0;
}