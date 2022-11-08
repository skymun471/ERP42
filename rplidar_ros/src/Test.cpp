#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#define RAD2DEG(x) ((x)*180./M_PI) // rad -> deg

/*void scanToPoint(const sensor_msgs::LaserScan::ConstPtr& scan, float radian, float dist, int i, float& meter, float& n_deg) //거리재는 함수
{
    float dy = dist * cos(radian); //dist = scan->range[i] 스캔 후 계산되는 거리 와 cos(radian)을 곱하여 거리 계산 단위:meter
    float deg = RAD2DEG(scan->angle_min + scan->angle_increment * i);
    meter = dy;
    n_deg = deg;
}*/


void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int count = scan->scan_time / scan->time_increment;
    //ROS_INFO("I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count);
    //ROS_INFO("angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max)); 
    for(int i = 0; i < count; i++) {
        int no_collision; //장애물 유무 판단
        float radian; // rad값
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i); // rad -> deg하여 각도 구하기
        //scan->range[i] = 거리값
        
        
        //ROS_INFO(": [%f, %f]", degree, scan->ranges[i]);
        float n_deg1,n_deg2;
        float meter1,meter2; //거리값 들어가는 변수
        //scanToPoint(radian, scan->ranges[i], meter);

        if(scan->ranges[i] <= 0.30){
            if(degree >= 0.0 && degree <=31.0 ){
                no_collision = 1;
                n_deg1 = degree;
                meter1 = scan->ranges[i];
                //scanToPoint(const sensor_msgs::LaserScan::ConstPtr& scan,radian , scan->ranges[i], i, meter, n_deg);
            }
            if(degree >= -30.0 && degree <=-1.0 ){
                no_collision = 1;
                n_deg2 = degree;
                meter2 = scan->ranges[i];
                //scanToPoint(const sensor_msgs::LaserScan::ConstPtr& scan, radian ,scan->ranges[i], i, meter, n_deg);
            }
        }
        if(no_collision == 1)
        {
            ROS_INFO("Turn!!");
            ROS_INFO(": [%f, %fm]", n_deg1, meter1);
            ROS_INFO(": [%f, %fm]", n_deg2, meter2);
        }
        else
        {
            ROS_INFO("Go!!");
            ROS_INFO(": [%f][%fm]", degree, scan->ranges[i]);
        }

        
        // 각도 -30도부터 30도사이에서 거리 0.30m 이하에 장애물 인식시 no_collision = 1로 변경
        // 장애물 값에 따라 직진또는 회전이 뜸 
        
    }
    

}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "rplidar_node_client");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);

    ros::spin();

    return 0;
}

