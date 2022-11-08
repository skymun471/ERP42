#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include <sensor_msgs/Image.h>
 
using namespace cv;

Mat img,img1,img2;

void imageCallback1(const sensor_msgs::Image::ConstPtr &msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  img1 = cv_ptr->image;
  //ROS_INFO("image callback running!");
}

void imageCallback2(const sensor_msgs::Image::ConstPtr &msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  img2 = cv_ptr->image;
  //ROS_INFO("image callback running!");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "cam_stack");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub1 = it.subscribe("/usb_cam1/image_raw", 1, imageCallback1);
    image_transport::Subscriber sub2 = it.subscribe("/usb_cam2/image_raw", 1, imageCallback2);

    image_transport::Publisher pub = it.advertise("/stacked_img", 10);
    ros::Rate loop_rate(10);
 
    while (ros::ok()) {
 
      // 두개의 이미지를 합치기
      if (!img1.empty() && !img2.empty()){
        hconcat(img1, img2, img);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
        pub.publish(msg);
      }
      else ROS_INFO("can't subscribe usb_cam data!!!");
      ros::spinOnce();
      loop_rate.sleep();
    }
}