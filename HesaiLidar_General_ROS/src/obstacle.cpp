#include <ros/ros.h>
#include <iostream>
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"
#include "hesai_lidar/erpStatusMsg.h"
#include <algorithm>

int obstacle = 0;
int right_range = 0;
int right_angle = 0;
int left_range = 0;
int left_angle = 0;
int mode_right_range = 0;
int mode_right_angle = 0;
int mode_left_range = 0;
int mode_left_angle = 0;
int count_right_range = 1;
int count_right_angle = 1;
int count_left_range = 1;
int count_left_angle = 1;
int mode_angle = 0;
int mode_range = 0;
int temp_right_range[100000];
int temp_right_angle[100000];
int temp_left_range[100000];
int temp_left_angle[100000];
std_msgs::Int16 mode;  // 0 no 1 st 2 dy 3 mid
int temp_point[100000];
int mode_point = 0;
int count_point = 0;
int sum_point = 0;
int avg_point = 0;
int speed = 0;

ros::Publisher ob_mode_pub;

void status_Callback(const hesai_lidar::erpStatusMsg& msg){
    speed = msg.speed;
}

void right_angle_cal(const std_msgs::Int32& msg)
{
    right_angle = msg.data;
    if(right_angle != 999999999){
        if(mode_right_angle == 0){
            mode_right_angle = 3;
        }
        if(speed == 0){
            temp_right_angle[count_right_angle]=(int)right_angle;
            count_right_angle++;
        }
    }
    if(mode_right_angle == 3){
        if(count_right_angle >= 17){
            if(right_angle <= temp_right_angle[16] + 1 && right_angle >= temp_right_angle[16] - 1){
                if(mode_right_angle == 3){
                    mode_right_angle = 1;
                }
            }
            else{
                if(mode_right_angle == 3){
                    mode_right_angle = 2;
                }
            }
        }
    }
    if(right_angle == 999999999){
        mode_right_angle = 0;
        count_right_angle = 1;
        std::fill(temp_right_angle, temp_right_angle + 100000, 0);
    }
}

void right_range_cal(const std_msgs::Int32& msg)
{
    right_range = msg.data;
    if(right_range != 9999900){
        if(mode_right_range == 0){
            mode_right_range = 3;
        }
        if(speed == 0){
            temp_right_range[count_right_range]=(int)right_range;
            count_right_range++;
        }
    }
    if(mode_right_range == 3){
        if(count_right_range >= 17){
            if(right_range <= temp_right_range[13] + 3 && right_range >= temp_right_range[13] - 3){
                if(mode_right_range == 3){
                    mode_right_range = 1;
                }
            }
            else{
                if(mode_right_range == 3){
                    mode_right_range = 2;
                }
            }
        }
    }
    if(right_range == 9999900){
        mode_right_range = 0;
        count_right_range = 1;
        std::fill(temp_right_range, temp_right_range + 100000, 0);
    }
}

void left_angle_cal(const std_msgs::Int32& msg)
{
    left_angle = msg.data;
    if(left_angle != 999999999){
        if(mode_left_angle == 0){
            mode_left_angle = 3;
        }
        if(speed == 0){
            temp_left_angle[count_left_angle]=(int)left_angle;
            count_left_angle++;
        }
    }
    if(mode_left_angle == 3){
        if(count_left_angle >= 17){
            if(left_angle <= temp_left_angle[8] + 1 && left_angle >= temp_left_angle[8] - 1){
                if(mode_left_angle == 3){
                    mode_left_angle = 1;
                }
            }
            else{
                if(mode_left_angle == 3){
                    mode_left_angle = 2;
                }
            }
        }
    }
    if(left_angle == 999999999){
        mode_left_angle = 0;
        count_left_angle = 1;
        std::fill(temp_left_angle, temp_left_angle + 100000, 0);
    }
}

void left_range_cal(const std_msgs::Int32& msg)
{
    left_range = msg.data;
    if(left_range != 9999900){
        if(mode_left_range == 0){
            mode_left_range = 3;
        }
        if(speed == 0){
            temp_left_range[count_left_range]=(int)left_range;
            count_left_range++;
        }
    }
    if(mode_left_range == 3){
        if(count_left_range >= 17){
            if(left_range <= temp_left_range[1] + 3 && left_range >= temp_left_range[1] - 3){
                if(mode_left_range == 3){
                    mode_left_range = 1;
                }
            }
            else{
                if(mode_left_range == 3){
                    mode_left_range = 2;
                }
            }
        }
    }
    if(left_range == 9999900){
        mode_left_range = 0;
        count_left_range = 1;
        std::fill(temp_left_range, temp_left_range + 100000, 0);
    }
}

void obstacle_Callback(const std_msgs::Int32& msg)
{

	obstacle = msg.data;
    if(obstacle >= 10){
        if(mode_point == 0){
            mode_point = 3;
        }
        if(speed == 0){
            temp_point[count_point]=(int)obstacle;
            sum_point += temp_point[count_point];
            count_point++;
            if(count_point == 15){
                avg_point = sum_point/15;
            }
        }
    }
    if(obstacle < 10){
        count_point = 0;
        obstacle = 0;
        std::fill(temp_point, temp_point + 100000, 0);
    }
    if(mode_point == 3){
        if(count_point >= 16){
            if(obstacle <= avg_point + 10 && obstacle >= avg_point - 10){
                if(mode_point==3){
                    mode_point = 1;
                }
            }
            else{
                if(mode_point==3){
                    mode_point = 2;
                }
            }
        }
    }
    if(obstacle == 0 && count_point == 0){
        mode_point = 0;
        sum_point = 0;
        avg_point = 0;
    }
}

void mode_Callback(const std_msgs::Int16& msg)
{
    int ox = msg.data;

    if(mode_right_angle == mode_left_angle){
        mode_angle = mode_left_angle;
    }
    else{
        mode_angle = 2;
    }
    if(mode_right_range == mode_left_range){
        mode_range = mode_left_angle;
    }
    else{
        mode_range = 2;
    }

    if(mode_point == mode_range && mode_range == mode_angle)
    {
        mode.data = mode_point;
    }

    if(mode_angle == 1 && mode_range == 1){
        if(mode_point == 1 || mode_point == 2){
            mode.data = 1;
        }
    }

    if(mode_point == 1 && mode_angle == 1){
        if(mode_range == 2){
            mode.data = 2;
        }
    }

    if(mode_point == 1 && mode_angle == 2){
        if(mode_range == 1){
            mode.data = 2;
        }
        if(mode_range == 2){
            mode.data = 2;
        }
    }
    if(mode_point == 2 && mode_angle == 1){
        if(mode_range == 2){
            mode.data = 2;
        }
    }
    if(mode_point == 2 && mode_angle == 2){
        if(mode_range == 1){
            mode.data = 2;
        }
    }
    std::cout << "range : " << mode_range << std::endl << "angle : " << mode_angle << std::endl << "obstacle : " << mode_point << std::endl;
    ob_mode_pub.publish(mode);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_node");
    ros::NodeHandle node;
    ros::Subscriber sub_obstacle = node.subscribe("/obstacle",1,&obstacle_Callback);
    ros::Subscriber sub_ox = node.subscribe("/point_ox",1,&mode_Callback);
    ros::Subscriber sub_status = node.subscribe("/erp42_status",3,&status_Callback);
    ros::Subscriber sub_right_angle = node.subscribe("/right_angle",1,&right_angle_cal);
    ros::Subscriber sub_right_range = node.subscribe("/right_range",1,&right_range_cal);
    ros::Subscriber sub_left_angle = node.subscribe("/left_angle",1,&left_angle_cal);
    ros::Subscriber sub_left_range = node.subscribe("/left_range",1,&left_range_cal);
    ob_mode_pub = node.advertise<std_msgs::Int16>("/ob_mode",1);
    ros::spin();
    return 0;

}