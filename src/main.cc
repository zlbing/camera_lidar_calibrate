//
// Created by zzz on 19-4-18.
//
#include <cstdlib>
#include <cstdio>
#include <math.h>
#include <algorithm>
#include <map>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl_ros/point_cloud.h>
#include <boost/foreach.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>

#include "fusion_lidar_camera.h"
#include "lidar_camera_calibration/marker_6dof.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "find_transform");
  ros::NodeHandle n;
  GS::FusionLidarCamera fusion_handle;
  std::string VELODYNE_TOPIC;

  ROS_INFO_STREAM("Reading CameraInfo from configuration file");
  n.getParam("/lidar_camera_calibration/velodyne_topic", VELODYNE_TOPIC);
  ROS_INFO_STREAM("VELODYNE_TOPIC="<<VELODYNE_TOPIC);
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(n, VELODYNE_TOPIC, 10);
  message_filters::Subscriber<lidar_camera_calibration::marker_6dof> rt_sub(n, "lidar_camera_calibration_rt", 10);
  message_filters::Subscriber<sensor_msgs::Image> img_sub(n, "/image_raw", 10);

  typedef message_filters::sync_policies::ApproximateTime<
          sensor_msgs::Image,
          sensor_msgs::PointCloud2,
          lidar_camera_calibration::marker_6dof> MySyncPolicy;

  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), img_sub, cloud_sub, rt_sub);
  sync.registerCallback(boost::bind(&GS::FusionLidarCamera::callback,&fusion_handle, _1, _2, _3));

  ros::spin();
}