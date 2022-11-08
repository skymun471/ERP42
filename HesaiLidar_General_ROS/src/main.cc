#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "pandarGeneral_sdk/pandarGeneral_sdk.h"
#include <fstream>
#include "std_msgs/Int32.h"
#include "std_msgs/Int16.h"
// #define PRINT_FLAG
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <math.h>

using namespace std;

ros::Publisher pub;
ros::Publisher pub_obstacle;
ros::Publisher pub_right_angle;
ros::Publisher pub_right_range;
ros::Publisher pub_left_angle;
ros::Publisher pub_left_range;
ros::Publisher pub_ox;
ros::Publisher pub_range;
ros::Publisher pub_point_avoid_right;
ros::Publisher pub_point_avoid_left;

std_msgs::Int16 ox;
std_msgs::Int32 obstacle;
std_msgs::Int32 point_right_angle;
std_msgs::Int32 point_right_range;
std_msgs::Int32 point_left_angle;
std_msgs::Int32 point_left_range;
std_msgs::Int16 avg_range;
std_msgs::Int32 point_avoid_right;
std_msgs::Int32 point_avoid_left;

pcl::PointCloud<pcl::PointXYZI> cloud_copy;

double size = 0;
double x_sum = 0;
double y_sum = 0;
double x_right = 0;
double x_left = 0;
double y_right = 0;
double y_left = 0;
double num_right = 0;
double num_left = 0;
double temp_range = 0;
double right_angle = 999999999;
double left_angle = 999999999;
double right_range = 99999;
double left_range = 99999;

class HesaiLidarClient
{
public:
  HesaiLidarClient(ros::NodeHandle node, ros::NodeHandle nh)
  {
    lidarPublisher = node.advertise<sensor_msgs::PointCloud2>("pandar", 10);
    packetPublisher = node.advertise<hesai_lidar::PandarScan>("pandar_packets",10);

    string serverIp;
    int lidarRecvPort;
    int gpsPort;
    double startAngle;
    string lidarCorrectionFile;  // Get local correction when getting from lidar failed
    string lidarType;
    string frameId;
    int pclDataType;
    string pcapFile;
    string dataType;
    string multicastIp;
    bool coordinateCorrectionFlag;
    string targetFrame;
    string fixedFrame;

    nh.getParam("pcap_file", pcapFile);
    nh.getParam("server_ip", serverIp);
    nh.getParam("lidar_recv_port", lidarRecvPort);
    nh.getParam("gps_port", gpsPort);
    nh.getParam("start_angle", startAngle);
    nh.getParam("lidar_correction_file", lidarCorrectionFile);
    nh.getParam("lidar_type", lidarType);
    nh.getParam("frame_id", frameId);
    nh.getParam("pcldata_type", pclDataType);
    nh.getParam("publish_type", m_sPublishType);
    nh.getParam("timestamp_type", m_sTimestampType);
    nh.getParam("data_type", dataType);
    nh.getParam("multicast_ip", multicastIp);
    nh.getParam("coordinate_correction_flag", coordinateCorrectionFlag);
    nh.getParam("target_frame", targetFrame);
    nh.getParam("fixed_frame", fixedFrame);

    if(!pcapFile.empty()){
      hsdk = new PandarGeneralSDK(pcapFile, boost::bind(&HesaiLidarClient::lidarCallback, this, _1, _2, _3), \
      static_cast<int>(startAngle * 100 + 0.5), 0, pclDataType, lidarType, frameId, m_sTimestampType, lidarCorrectionFile, \
      coordinateCorrectionFlag, targetFrame, fixedFrame);
      if (hsdk != NULL) {
        std::ifstream fin(lidarCorrectionFile);
        if (fin.is_open()) {
          std::cout << "Open correction file " << lidarCorrectionFile << " succeed" << std::endl;
          int length = 0;
          std::string strlidarCalibration;
          fin.seekg(0, std::ios::end);
          length = fin.tellg();
          fin.seekg(0, std::ios::beg);
          char *buffer = new char[length];
          fin.read(buffer, length);
          fin.close();
          strlidarCalibration = buffer;
          int ret = hsdk->LoadLidarCorrectionFile(strlidarCalibration);
          if (ret != 0) {
            std::cout << "Load correction file from " << lidarCorrectionFile <<" failed" << std::endl;
          } else {
            std::cout << "Load correction file from " << lidarCorrectionFile << " succeed" << std::endl;
          }
        }
        else{
          std::cout << "Open correction file " << lidarCorrectionFile << " failed" << std::endl;
        }
      }
    }
    else if ("rosbag" == dataType){
      hsdk = new PandarGeneralSDK("", boost::bind(&HesaiLidarClient::lidarCallback, this, _1, _2, _3), \
      static_cast<int>(startAngle * 100 + 0.5), 0, pclDataType, lidarType, frameId, m_sTimestampType, \
      lidarCorrectionFile, coordinateCorrectionFlag, targetFrame, fixedFrame);
      if (hsdk != NULL) {
        packetSubscriber = node.subscribe("pandar_packets",10,&HesaiLidarClient::scanCallback, (HesaiLidarClient*)this, ros::TransportHints().tcpNoDelay(true));
      }
    }
    else {
      hsdk = new PandarGeneralSDK(serverIp, lidarRecvPort, gpsPort, \
        boost::bind(&HesaiLidarClient::lidarCallback, this, _1, _2, _3), \
        boost::bind(&HesaiLidarClient::gpsCallback, this, _1), static_cast<int>(startAngle * 100 + 0.5), 0, pclDataType, lidarType, frameId,\
         m_sTimestampType, lidarCorrectionFile, multicastIp, coordinateCorrectionFlag, targetFrame, fixedFrame);
    }

    if (hsdk != NULL) {
        hsdk->Start();
        // hsdk->LoadLidarCorrectionFile("...");  // parameter is stream in lidarCorrectionFile
    } else {
        printf("create sdk fail\n");
    }
  }

  void lidarCallback(boost::shared_ptr<PPointCloud> cld, double timestamp, hesai_lidar::PandarScanPtr scan) // the timestamp from first point cloud of cld
  {
    if(m_sPublishType == "both" || m_sPublishType == "points"){
      pcl_conversions::toPCL(ros::Time(timestamp), cld->header.stamp);
      sensor_msgs::PointCloud2 output;
      pcl::toROSMsg(*cld, output);
      lidarPublisher.publish(output);
#ifdef PRINT_FLAG
        printf("timestamp: %f, point size: %ld.\n",timestamp, cld->points.size());
#endif
    }
    if(m_sPublishType == "both" || m_sPublishType == "raw"){
      packetPublisher.publish(scan);
#ifdef PRINT_FLAG
        printf("raw size: %d.\n", scan->packets.size());
#endif
    }
  }

  void gpsCallback(int timestamp) {
#ifdef PRINT_FLAG
      printf("gps: %d\n", timestamp);
#endif
  }

  void scanCallback(const hesai_lidar::PandarScanPtr scan)
  {
    // printf("pandar_packets topic message received,\n");
    hsdk->PushScanPacket(scan);
  }

private:
  ros::Publisher lidarPublisher;
  ros::Publisher packetPublisher;
  PandarGeneralSDK* hsdk;
  string m_sPublishType;
  string m_sTimestampType;
  ros::Subscriber packetSubscriber;
};

void PointFilter(const sensor_msgs::PointCloud2 output)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(output, *input_cloud);

  pcl::PointCloud<pcl::PointXYZI>::Ptr crop_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::CropBox<pcl::PointXYZI> crop_filter;
  Eigen::Vector4f min_pt (-0.7f, -4.6f, -0.5f, 1.0f);
  Eigen::Vector4f max_pt (0.7f, -0.5f, 0.7f, 1.0f);
  crop_filter.setInputCloud(input_cloud);
  crop_filter.setMin(min_pt);
  crop_filter.setMax(max_pt);
  crop_filter.filter(*crop_cloud);

  pcl::PointCloud<pcl::PointXYZI>::Ptr voxel_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
  float voxelsize = 0.01; // 0.075 setting size 0.2 ~ 0.01
  voxel_filter.setInputCloud(crop_cloud); // input cloud
  voxel_filter.setLeafSize(voxelsize, voxelsize, voxelsize);
  voxel_filter.filter(*voxel_cloud);
  obstacle.data = voxel_cloud->points.size();

  cloud_copy = *voxel_cloud;
  size = voxel_cloud->points.size();

  if(size > 10){
    x_right = cloud_copy.points[0].x;
    x_left = cloud_copy.points[0].x;

    for(double i=0;i<size;i++){
      x_sum += cloud_copy.points[i].x;
      y_sum += cloud_copy.points[i].y;
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

    x_sum = x_sum/size;
    y_sum = y_sum/size;

    temp_range = (sqrt(pow(x_sum,2)+pow(y_sum,2)))*100;

    right_angle = ((atan2(y_right, x_right))*(180/M_PI)) + 90.0;
    left_angle = ((atan2(y_left, x_left))*(180/M_PI)) + 90.0;
    right_range = sqrt(pow(x_right,2)+pow(y_right,2));
    left_range = sqrt(pow(x_left,2)+pow(y_left,2));

    ox.data = 1;
    point_right_angle.data = right_angle;
    point_right_range.data = right_range*(double)100;
    point_left_angle.data = left_angle;
    point_left_range.data = left_range*(double)100;
    avg_range.data = temp_range;

  }
  else{
    ox.data = 0;
    size = 0;
    x_right = 0;
    x_left = 0;
    y_right = 0;
    y_left = 0;
    num_right = 0;
    num_left = 0;
    right_angle = 999999999;
    left_angle = 999999999;
    right_range = 99999;
    left_range = 99999;
    point_right_angle.data = 999999999;
    point_right_range.data = 9999900;
    point_left_angle.data = 999999999;
    point_left_range.data = 9999900;
    avg_range.data = 0;
  }

  pub_right_angle.publish(point_right_angle);
  pub_right_range.publish(point_right_range);
  pub_left_angle.publish(point_left_angle);
  pub_left_range.publish(point_left_range);

  sensor_msgs::PointCloud2 out_put;
  pcl::toROSMsg(*voxel_cloud, out_put);
  pub.publish (out_put);
  pub_ox.publish(ox);
  pub_obstacle.publish(obstacle);
  pub_range.publish(avg_range);

}

void PointFilter_avoid(const sensor_msgs::PointCloud2 output)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(output, *input_cloud);

  //avoid to right
  pcl::PointCloud<pcl::PointXYZI>::Ptr crop_cloud_r(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::CropBox<pcl::PointXYZI> crop_filter_r;
  Eigen::Vector4f min_pt_r (0.7f, -4.5f, -0.5f, 1.0f);
  Eigen::Vector4f max_pt_r (2.0f, -0.5f, 0.7f, 1.0f);
  crop_filter_r.setInputCloud(input_cloud);
  crop_filter_r.setMin(min_pt_r);
  crop_filter_r.setMax(max_pt_r);
  crop_filter_r.filter(*crop_cloud_r);

  pcl::PointCloud<pcl::PointXYZI>::Ptr voxel_cloud_r(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::VoxelGrid<pcl::PointXYZI> voxel_filter_r;
  float voxelsize_r = 0.01; // 0.075 setting size 0.2 ~ 0.01
  voxel_filter_r.setInputCloud(crop_cloud_r); // input cloud
  voxel_filter_r.setLeafSize(voxelsize_r, voxelsize_r, voxelsize_r);
  voxel_filter_r.filter(*voxel_cloud_r);

  //avoid to left
  pcl::PointCloud<pcl::PointXYZI>::Ptr crop_cloud_l(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::CropBox<pcl::PointXYZI> crop_filter_l;
  Eigen::Vector4f min_pt_l (-2.0f, -4.5f, -0.5f, 1.0f);
  Eigen::Vector4f max_pt_l (-0.7f, -0.5f, 0.7f, 1.0f);
  crop_filter_l.setInputCloud(input_cloud);
  crop_filter_l.setMin(min_pt_l);
  crop_filter_l.setMax(max_pt_l);
  crop_filter_l.filter(*crop_cloud_l);

  pcl::PointCloud<pcl::PointXYZI>::Ptr voxel_cloud_l(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::VoxelGrid<pcl::PointXYZI> voxel_filter_l;
  float voxelsize_l = 0.01; // 0.075 setting size 0.2 ~ 0.01
  voxel_filter_l.setInputCloud(crop_cloud_l); // input cloud
  voxel_filter_l.setLeafSize(voxelsize_l, voxelsize_l, voxelsize_l);
  voxel_filter_l.filter(*voxel_cloud_l);

  point_avoid_left.data = voxel_cloud_l->points.size();
  point_avoid_right.data = voxel_cloud_r->points.size();

  pub_point_avoid_left.publish(point_avoid_left);
  pub_point_avoid_right.publish(point_avoid_right);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pandar");
  ros::NodeHandle nh("~");
  ros::NodeHandle node;
  HesaiLidarClient pandarClient(node, nh);
  ros::Subscriber sub = node.subscribe ("/hesai/pandar", 1, PointFilter);
  ros::Subscriber sub2 = node.subscribe ("/hesai/pandar", 1, PointFilter_avoid);
  pub = node.advertise<sensor_msgs::PointCloud2> ("/out_put", 1);
  pub_obstacle = node.advertise<std_msgs::Int32> ("/obstacle",1);
  pub_ox = node.advertise<std_msgs::Int16> ("/point_ox",1);
  pub_right_angle = node.advertise<std_msgs::Int32>("/right_angle",1);
  pub_right_range = node.advertise<std_msgs::Int32>("/right_range",1);
  pub_left_angle = node.advertise<std_msgs::Int32>("/left_angle",1);
  pub_left_range = node.advertise<std_msgs::Int32>("/left_range",1);
  pub_range = node.advertise<std_msgs::Int16>("/avg_range",1);
  pub_point_avoid_left = node.advertise<std_msgs::Int32>("/avoid_to_left",1);
  pub_point_avoid_right = node.advertise<std_msgs::Int32>("/avoid_to_right",1);

  ros::spin();
  return 0;
}
