//
// Created by zzz on 19-4-18.
//

#ifndef LIDAR_CAMERA_CALIBRATION_COMPUTE_RT_H
#define LIDAR_CAMERA_CALIBRATION_COMPUTE_RT_H
#include <ros/package.h>
#include <ros/assert.h>
#include <cstdlib>
#include <cstdio>
#include <math.h>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <utility>

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>
#include <ceres/ceres.h>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

#include "lidar_costfunction.h"
namespace GS{
  class ComputeRT{
  public:
    ComputeRT();
    ComputeRT(const ComputeRT&) = delete;
    ComputeRT& operator=(const ComputeRT&) = delete;

    Eigen::Quaterniond addQ(Eigen::Quaterniond a, Eigen::Quaterniond b);
    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> readArray();
    Eigen::Matrix4d calc_RT(Eigen::MatrixXd lidar, Eigen::MatrixXd camera, int MAX_ITERS, Eigen::Matrix3d lidarToCamera);

    std::vector<std::vector<Eigen::Vector3d>>
    readArucoPose(std::vector<float> marker_info, int num_of_marker_in_config);

    Eigen::Matrix4d find_transformation(std::vector<float> marker_info, int num_of_marker_in_config,
                                 int MAX_ITERS, Eigen::Matrix3d lidarToCamera);

    Eigen::Matrix4d optimizeByCeres(Eigen::Matrix3d lidarToCamera);

    Eigen::Matrix4d findTByCeres(std::vector<float> marker_info,
                                 std::vector<std::vector<std::vector<Eigen::Vector3d>>> line_points,
                                 int num_of_marker_in_config,
                                 int MAX_ITERS,
                                 Eigen::Matrix3d lidarToCamera);

    Eigen::Vector3d translation_sum;
    Eigen::Quaterniond rotation_sum;

    Eigen::Matrix3d rotation_avg_by_mult;
    float rmse_avg;
    std::vector<std::vector<std::vector<Eigen::Vector3d>>> all_line_points;
    std::vector<std::vector<Eigen::Vector3d>> all_corner_points;
    int iteration_counter;
    std::string pkg_loc;
    Eigen::Quaterniond initial_rotation;
    Eigen::Vector3d initial_translation;

    tf2_ros::TransformBroadcaster br;

  };
}



#endif //LIDAR_CAMERA_CALIBRATION_COMPUTE_RT_H
