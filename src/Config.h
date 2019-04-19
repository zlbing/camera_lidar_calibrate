//
// Created by zzz on 19-4-18.
//

#ifndef LIDAR_CAMERA_CALIBRATION_CONFIG_H
#define LIDAR_CAMERA_CALIBRATION_CONFIG_H
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <map>
#include <fstream>
#include <cmath>

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <ros/package.h>

#include <pcl_ros/point_cloud.h>
#include <boost/foreach.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/common/intersections.h>
struct myPointXYZRID{
  PCL_ADD_POINT4D
  ; // quad-word XYZ
  float intensity; ///< laser intensity reading
  uint16_t ring; ///< laser ring number
  float range;EIGEN_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
        myPointXYZRID, (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity) (uint16_t, ring, ring))

namespace GS{

  struct config_settings
  {
    cv::Size s;			                        //image width and height
    std::vector<std::pair<float,float>> xyz_;   //filter point cloud to region of interest
    float intensity_thresh;                     //intensity thresh
    int num_of_markers;
    bool useCameraInfo;
    cv::Mat P;
    int MAX_ITERS;
    std::vector<float> initialRot;

    void print()
    {
      std::cout <<  "Size of the frame: " << s.width << " x " << s.height << "\n";
      for(int i = 0; i < 3; i++)
      {
        std::cout << "Limits: " << xyz_[i].first << " to " << xyz_[i].second << std::endl;
      }
      std::cout << "Number of markers: " << num_of_markers << std::endl;
      std::cout << "Intensity threshold (between 0.0 and 1.0): " << intensity_thresh << "\nuseCameraInfo: " << useCameraInfo << "\n";
      std::cout << "Projection matrix: \n" << P << "\n";
      std::cout << "MAX_ITERS: " << MAX_ITERS << "\n";
    }
  };

  class Config {
  public:
    Config(){};
    Config(const Config&) = delete;
    Config& operator=(const Config&) = delete;

    void readConfig();

    pcl::PointCloud<pcl::PointXYZ>* toPointsXYZ(pcl::PointCloud<myPointXYZRID> point_cloud);
    pcl::PointCloud<myPointXYZRID> transform(pcl::PointCloud<myPointXYZRID> pc, float x, float y, float z, float rot_x, float rot_y, float rot_z);
    pcl::PointCloud<myPointXYZRID> normalizeIntensity(pcl::PointCloud<myPointXYZRID> point_cloud, float min, float max);
    pcl::PointCloud<myPointXYZRID> intensityByRangeDiff(pcl::PointCloud<myPointXYZRID> point_cloud);

    cv::Point project(const pcl::PointXYZ &pt, const cv::Mat &projection_matrix);
    cv::Mat project(cv::Mat projection_matrix, cv::Rect frame,
                            pcl::PointCloud<pcl::PointXYZ> point_cloud,
                            pcl::PointCloud<pcl::PointXYZ> *visible_points);


    config_settings conf_data_;
  };
}



#endif //LIDAR_CAMERA_CALIBRATION_CONFIG_H
