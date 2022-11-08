//opencv_line_detect.cpp
//-I/usr/local/include/opencv4/opencv -I/usr/local/include/opencv4

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>

using namespace cv;
using namespace std;

#define IMG_Width     640
#define IMG_Height    480

#define USE_DEBUG  0   // 1 Debug  사용
#define USE_CAMERA 1   // 1 CAMERA 사용  0 CAMERA 미사용

#define ROI_CENTER_Y  340
#define ROI_WIDTH     30

#define NO_LINE 20
#define DEG2RAD(x) (M_PI/180.0)*x
#define RAD2DEG(x) (180.0/M_PI)*x
#define x_diff 172


Mat mat_image_org_color;  // Image 저장하기 위한 변수
Mat mat_image_org_gray;
Mat mat_image_roi;
Mat mat_image_canny_edge;
Mat mat_image_canny_edge_roi;

Scalar GREEN(0,255,0);
Scalar RED(0,0,255);
Scalar BLUE(255,0,0);
Scalar YELLOW(0,255,255);


int img_width = 640;
int img_height = 480;

int mode_flag = 0;

Mat img;

Mat Region_of_Interest_crop(Mat image)
{
  int width = image.cols;
  int height = image.rows;
  int up = 75;
  int down = 10;
  Point points[4];
  points[0] = Point(up,ROI_CENTER_Y);
  points[1] = Point(width-up,ROI_CENTER_Y);
  points[2] = Point(width-down,height-5);
  points[3] = Point(down,height-5);

  Mat img_mask = Mat::zeros(image.rows, image.cols, CV_8UC1);
  const Point* ppt[1] = { points };
  int npt[] = { 4 };
  fillPoly(img_mask, ppt, npt, 1, Scalar(255, 255, 255), LINE_8);

  Mat img_roi_crop;
  bitwise_and(image, img_mask, img_roi_crop);

   return img_roi_crop;
}

Mat Canny_Edge_Detection(Mat img)
{
   Mat mat_blur_img, mat_canny_img;
   blur(img, mat_blur_img, Size(3,3));
   Canny(mat_blur_img,mat_canny_img, 190,210,3); //canny edge

   return mat_canny_img;
}

void imageCallback(const sensor_msgs::Image::ConstPtr &msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  img = cv_ptr->image;
  //ROS_INFO("image callback running!");
}


int main(int argc, char **argv)
{

   int i;
   int steer_angle_new, steer_angle_old;

   steer_angle_new = steer_angle_old =0;

   float gradient[NO_LINE]  = {0,};
   float intersect[NO_LINE] = {0,};
   float intersect_base[NO_LINE]  = {0,};
   float c_x_sum=0;

   int capture_width = 640 ;
   int capture_height = 480 ;
   int display_width = 640 ;
   int display_height = 480 ;
   int framerate = 60;
   int flip_method = 2 ;

   int img_width  = 640;
   int img_height = 480;
   if(USE_CAMERA == 0) img_height = 480;




   if(USE_CAMERA == 0) printf("Image size[%3d,%3d]\n", capture_width,capture_height);


   ros::init(argc, argv, "m_race_vision");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
   ros::NodeHandle nh;
   image_transport::ImageTransport it(nh);
   image_transport::Subscriber sub = it.subscribe("/usb_cam1/image_raw", 1, imageCallback);

   ros::Publisher car_control_pub_cmd = nh.advertise<std_msgs::Int16>("/SteerAngle_Int16", 10);
   ros::Publisher car_control_pub_flag = nh.advertise<std_msgs::Int16>("/stop_line_flag", 10);  //***********


   std_msgs::Int16 cmd_steering_msg;
   std_msgs::Int16 stop_line_flag;    //***********
   cmd_steering_msg.data  = 0;
   stop_line_flag.data = 0;   //***********


   ros::Rate loop_rate(10);
   int count = 0;
   int line_count = 0;
   float  c[NO_LINE] = {0.0, };
   float  d[NO_LINE] = {0.0, };
   float  line_center_x = 0.0;
   float  L_line = 0.0;
   float  R_line = 0.0;
   int M_line = 0;
   int    L_count = 0, R_count = 0;
   float centerx_buffer[] = {0.0, 0.0, 0.0, 0.0};
   float  steering_conversion = 1;
   int inter_sect_x[NO_LINE] = {0, };
   int inter_sect_group[NO_LINE] = {0, };
   /*
   Point points[4];


   points[0] = Point(0,ROI_CENTER_Y-ROI_WIDTH);
   points[1] =  Point(0,ROI_CENTER_Y+ROI_WIDTH);
   points[2] =  Point(img_width,ROI_CENTER_Y+ROI_WIDTH);
   points[3] =  Point(img_width,ROI_CENTER_Y-ROI_WIDTH);
   */

	while (ros::ok())
	{
		/**
		* This is a message object. You stuff it with data, and then publish it.
		*/
		ros::spinOnce();
		if (img.empty()) {
          //ROS_WARN("image: no data!");
          continue;
        }

		cvtColor(img, mat_image_org_gray, cv::COLOR_RGB2GRAY);
		mat_image_canny_edge = Canny_Edge_Detection(mat_image_org_gray);
		mat_image_roi = Region_of_Interest_crop(mat_image_canny_edge);    // ROI 영역을 추출함

		//imshow("test", mat_image_roi);
		vector<Vec4i> linesP;

		HoughLinesP(mat_image_roi, linesP, 1, CV_PI/180,35,25,100); //num of point, min length, min point length

		//printf("Line Number : %3d\n", (int)linesP.size());

		//line_count = line_center_x = 0.0;
		line_center_x = 0.0;
		L_count=0; R_count=0; M_line = 0;
		L_line=0.0; R_line=0.0;
		for(int i=0; i<linesP.size();i++)
		{
			float intersect = 0.0;

			if(i>=NO_LINE) break;
			Vec4i L= linesP[i];

			if(abs(L[0]-L[2])>200 && abs(L[1]-L[3]) < 10){
        M_line++;
        line(img,Point(L[0],L[1]),Point(L[2],L[3]), Scalar(0,255,0), 3, LINE_AA);
			}
      //if (abs(L[0]-L[2])>200 || abs(L[1]-L[3]) < 110)	continue;

			if(abs(L[1]-L[3]) < 50) continue;
			//else printf("%d\n", abs(L[1]-L[3]));

			if(sqrt(pow(abs(L[0]-L[2]),2)+pow(abs(L[1]-L[3]),2)) < 90){
				//printf("%.2f\n", sqrt(pow(abs(L[0]-L[2]),2)+pow(abs(L[1]-L[3]),2)));
				continue; //limit length
			}

			if((L[1]-L[3]) > 0 && L[0] < 580){
				L_line += (L[0]+L[2])/2;
				L_count++;
				line(img,Point(L[0],L[1]),Point(L[2],L[3]), Scalar(0,0,255), 3, LINE_AA);
			}
			else if((L[1]-L[3]) < 0 && L[2] > 60){
				R_line+= (L[2]+L[0])/2;
				R_count++;
				line(img,Point(L[0],L[1]),Point(L[2],L[3]), Scalar(255,0,0), 3, LINE_AA);
			}
			//line(img,Point(L[0],L[1]+ROI_CENTER_Y-ROI_WIDTH),Point(L[2],L[3]+ROI_CENTER_Y-ROI_WIDTH), Scalar(0,0,255), 3, LINE_AA);

			if(USE_DEBUG==1)
			{
				printf("L[%d] :[%3d, %3d] , [%3d , %3d] \n",i,  L[0],L[1], L[2],L[3]);

			}
		}
		// if(L_line != 0.0){
		// 	R_line = 580;
		// 	R_count = 1;
		// }

		if(R_line == 0.0){
			R_line =  620;
			R_count = 1;
		}
		if(L_line == 0.0) {
			L_line = 20;
			L_count = 1;
		}

		centerx_buffer[0] = centerx_buffer[1];
		centerx_buffer[1] = centerx_buffer[2];
		centerx_buffer[2] = centerx_buffer[3];
		centerx_buffer[3] = (L_line/L_count + R_line/R_count)/2;

    line_center_x = (centerx_buffer[0]+centerx_buffer[1]+centerx_buffer[2]+centerx_buffer[3]) / 4 - img_width/2;

    steer_angle_new = (int)( line_center_x*steering_conversion);  //스티어링 조정값 계수 수정 필요
        //printf("c_x_sum = %f  %d\n",line_center_x , steer_angle_new);
	    //printf("\n");
    cmd_steering_msg.data  = steer_angle_new;      //
    // ROS_INFO("%d", steer_angle_new);
    car_control_pub_cmd.publish(cmd_steering_msg);


    if(M_line > 0){
      stop_line_flag.data = 1;
    }
    car_control_pub_flag.publish(stop_line_flag);
    //ROS_INFO("%d", stop_line_flag);
	  line(img,Point(0,ROI_CENTER_Y),Point(img_width,ROI_CENTER_Y), Scalar(0,255,0), 1, LINE_AA);
	  line(img,Point((int)line_center_x+img_width/2,ROI_CENTER_Y+ROI_WIDTH),Point((int)line_center_x+img_width/2,ROI_CENTER_Y-ROI_WIDTH), Scalar(255,255,0), 3, LINE_AA);


    steer_angle_old =  steer_angle_new ;
	  //printf("result:%d\n", steer_angle_new);
	  imshow("image",img);

    // imshow("Camera Image", mat_image_org_color);
    /*imshow("Gray Image window", mat_image_org_gray);
      imshow("ROI Image window",mat_image_roi);
      imshow("Edge Image window",mat_image_canny_edge_roi);
    */

    if (waitKey(25) >= 0)
      break;

    //ros::spinOnce();
    //ros::spin();

    loop_rate.sleep();
    ++count;
  }

   return 0;
}


