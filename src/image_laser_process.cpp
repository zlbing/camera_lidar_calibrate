#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl_ros/point_cloud.h>
#include <boost/foreach.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
using namespace message_filters;

image_transport::Publisher pub_img;
std::ofstream write_stream;

ros::Publisher pub_info_camera;
sensor_msgs::CameraInfoPtr lcam(new sensor_msgs::CameraInfo());

void callback_lidar(const sensor_msgs::PointCloud2ConstPtr& msg_pc){
  ROS_INFO_STREAM("[test] Velodyne scan received at " << msg_pc->header.stamp.toSec());
}

void callback_image(const sensor_msgs::ImageConstPtr& original_image){
  ROS_INFO_STREAM("[test] original_image received at " << original_image->header.stamp.toSec());
  //Create cv_brigde instance
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr=cv_bridge::toCvCopy(original_image, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Not able to convert sensor_msgs::Image to OpenCV::Mat format %s", e.what());
    return;
  }
  // cv::flip(cv_ptr->image, cv_ptr->image, -1);
  sensor_msgs::ImagePtr image_convert_msg = cv_ptr->toImageMsg();
  image_convert_msg->header.frame_id="camera_link";
  pub_img.publish(image_convert_msg);
  lcam->header.stamp = image_convert_msg->header.stamp;
  lcam->header.frame_id = image_convert_msg->header.frame_id;
  pub_info_camera.publish(lcam);
}


void imageLaserCallback(const sensor_msgs::PointCloud2ConstPtr& msg_pc,
                        const sensor_msgs::ImageConstPtr& original_image)
{
  //Create cv_brigde instance
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr=cv_bridge::toCvCopy(original_image, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Not able to convert sensor_msgs::Image to OpenCV::Mat format %s", e.what());
    return;
  } 
  // cv::flip(cv_ptr->image, cv_ptr->image, -1);
  sensor_msgs::ImagePtr image_convert_msg = cv_ptr->toImageMsg();
  image_convert_msg->header.frame_id="camera_link";
  pub_img.publish(image_convert_msg);

  pcl::PointCloud<pcl::PointXYZI> points;
  fromROSMsg(*msg_pc, points);

  std::string time_str = std::to_string(image_convert_msg->header.stamp.toNSec());
  std::string base_path = "/home/zzz/imgae_laser_data/data1/";
  std::string image_path = base_path +time_str+".jpg";
  cv::imwrite(image_path, cv_ptr->image);
  std::string pc_path = base_path +time_str+".xyz";

  write_stream.open(pc_path.c_str(), std::ios::out | std::ios::trunc);

  for(int i=0; i<static_cast<int>(points.size()); i++){
    if(std::isnan(points[i].x)||
       std::isnan(points[i].y)||
       std::isnan(points[i].z)){
      continue;
    }
    write_stream<<points[i].x<<" "
                <<points[i].y<<" "
                <<points[i].z<<"\n";
  }
  write_stream.close();
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"image_Laser_process");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  pub_img = it.advertise("/image_correct", 10);
  
  // // Image node and subscribcer
  // message_filters::Subscriber<sensor_msgs::Image> img_sub(nh, "/rgb_image_hd", 10);
  // message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, "/rslidar_points_origin", 10);
  // typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy;
  // Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), cloud_sub, img_sub);
  // sync.registerCallback(boost::bind(&imageLaserCallback, _1, _2));

  // ros::Subscriber sub_lidar = nh.subscribe("/rslidar_points_origin", 100, callback_lidar);
  // ros::Subscriber sub_image = nh.subscribe("/rgb_image_hd", 100, callback_image);

  Eigen::Matrix3f rotation;
  rotation<<0.0241,   -0.9986,   -0.0471,
            0.3402,    0.0525,   -0.9389,
            0.9400,    0.0066,    0.3410;

  Eigen::Vector3f rpy = rotation.eulerAngles(0,1,2);
  const double r = ((double)rpy[0]);
  const double p = ((double)rpy[1]);
  const double y = ((double)rpy[2]);
  std::cout<<"r="<<r<<" p="<<p<<" y="<<y<<std::endl;
  pub_info_camera = nh.advertise<sensor_msgs::CameraInfo>("/camera_info", 1); //
  lcam->height = 480;
  lcam->width = 640;
  lcam->distortion_model = "plumb_bob";
  std::vector<double> distortion;
  distortion.push_back(0.134304);
  distortion.push_back(-0.241875);
  distortion.push_back(0.001620);
  distortion.push_back(-0.011621);
  distortion.push_back(0);
   // {0.134304, -0.241875, 0.001620, -0.011621, 0.000000};
  lcam->D = distortion;
  lcam->K = {592.192489, 0.000000, 315.709148, 0.000000, 588.541881, 258.452450, 0.000000, 0.000000, 1.000000};
  lcam->R = {1.000000, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000, 0.000000, 0.000000, 1.000000};
  lcam->P = {602.986694, 0.000000, 309.627327, 0.000000, 0.000000, 604.378479, 259.127746, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000};
  ros::Subscriber sub_image = nh.subscribe("/rgb_image_hd", 100, callback_image);

  ros::spin();
  return 0;
}