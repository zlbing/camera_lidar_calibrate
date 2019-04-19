//
// Created by zzz on 19-4-18.
//

#ifndef LIDAR_CAMERA_CALIBRATION_FUSION_LIDAR_CAMERA_H
#define LIDAR_CAMERA_CALIBRATION_FUSION_LIDAR_CAMERA_H
#include <boost/foreach.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>


#include "Config.h"
#include "lidar_camera_calibration/marker_6dof.h"
#include "compute_rt.h"
namespace GS{
  class FusionLidarCamera {
  public:
    FusionLidarCamera();

    FusionLidarCamera(const FusionLidarCamera&) = delete;
    FusionLidarCamera& operator=(const FusionLidarCamera&) = delete;

    void callback(const sensor_msgs::ImageConstPtr &original_image,
                  const sensor_msgs::PointCloud2ConstPtr& msg_pc,
                  const lidar_camera_calibration::marker_6dof::ConstPtr& msg_rt);

    std::vector<std::vector<std::vector<Eigen::Vector3d>>>
    getCorners(cv::Mat img, pcl::PointCloud<pcl::PointXYZ> scan, cv::Mat P, int num_of_markers, int MAX_ITERS);

    static void onMouse( int event, int x, int y, int f, void* g){
      cv::Point* P = static_cast<cv::Point*>(g);
      if(event == CV_EVENT_LBUTTONDOWN){
        std::cout <<"[onMouse] CV_EVENT_LBUTTONDOWN"<< x << " " << y << "\n";
      }
      if(event == CV_EVENT_LBUTTONUP){
        std::cout <<"[onMouse] CV_EVENT_LBUTTONUP"<< x << " " << y << "\n";
      }
      switch(event)
      {

        case  CV_EVENT_LBUTTONDOWN  :

          P->x=x;
          P->y=y;
          break;

        case  CV_EVENT_LBUTTONUP    :
          P->x=x;
          P->y=y;
          break;

        default                     :   break;


      }
    }

  private:
    Config config_;
    ComputeRT compute_rt_;
    std::vector< std::vector<cv::Point> > stored_corners;
    int iteration_count;

    ros::Publisher pub_filter_points_;

    ros::Publisher pub_camera_corner_,pub_lidar_corner_;
  };
}

#endif //LIDAR_CAMERA_CALIBRATION_FUSION_LIDAR_CAMERA_H
