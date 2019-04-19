//
// Created by zzz on 19-4-18.
//

#include "Config.h"
namespace GS{
  void Config::readConfig(){
    std::string pkg_loc = ros::package::getPath("lidar_camera_calibration");
    //std::cout<< "The conf file location: " << pkg_loc <<"/conf/config_file.txt" << std::endl;
    std::ifstream infile(pkg_loc + "/conf/config_file.txt");
    float left_limit=0.0, right_limit=0.0;
    float angle;

    infile >> conf_data_.s.width >> conf_data_.s.height;
    for(int i = 0; i<3; i++)
    {
      infile >> left_limit >> right_limit;
      conf_data_.xyz_.push_back(std::pair<float,float>(left_limit, right_limit));
    }

    infile >> conf_data_.intensity_thresh >> conf_data_.num_of_markers >> conf_data_.useCameraInfo;
    float p[12];
    for(int i=0; i<12; i++)
    {
      infile >> p[i];
    }
    cv::Mat(3, 4, CV_32FC1, &p).copyTo(conf_data_.P);

    infile >> conf_data_.MAX_ITERS;

    for(int i = 0; i < 3; i++)
    {
      infile >> angle;
      conf_data_.initialRot.push_back(angle);
    }

    infile.close();
    conf_data_.print();
  }

  pcl::PointCloud<pcl::PointXYZ>* Config::toPointsXYZ(pcl::PointCloud<myPointXYZRID> point_cloud){
    pcl::PointCloud<pcl::PointXYZ> *new_cloud = new pcl::PointCloud<pcl::PointXYZ>();
    for (pcl::PointCloud<myPointXYZRID>::iterator pt = point_cloud.points.begin(); pt < point_cloud.points.end(); pt++)
    {
      new_cloud->push_back(pcl::PointXYZ(pt->x, pt->y, pt->z));
    }
    return new_cloud;
  }

  pcl::PointCloud<myPointXYZRID> Config::transform(pcl::PointCloud<myPointXYZRID> pc,
                                                   float x, float y, float z,
                                                   float rot_x, float rot_y, float rot_z){
    Eigen::Affine3f transf = pcl::getTransformation(x, y, z, rot_x, rot_y, rot_z);
    pcl::PointCloud<myPointXYZRID> new_cloud;
    pcl::transformPointCloud(pc, new_cloud, transf);
    return new_cloud;
  }

  pcl::PointCloud<myPointXYZRID> Config::normalizeIntensity(pcl::PointCloud<myPointXYZRID> point_cloud,
                                                            float min, float max){
    float min_found = 10e6;
    float max_found = -10e6;

    for (pcl::PointCloud<myPointXYZRID>::iterator pt = point_cloud.points.begin(); pt < point_cloud.points.end(); pt++)
    {
      max_found = MAX(max_found, pt->intensity);
      min_found = MIN(min_found, pt->intensity);
    }

    for (pcl::PointCloud<myPointXYZRID>::iterator pt = point_cloud.points.begin(); pt < point_cloud.points.end(); pt++)
    {
      pt->intensity = (pt->intensity - min_found) / (max_found - min_found) * (max - min) + min;
    }
    return point_cloud;
  }
  pcl::PointCloud<myPointXYZRID> Config::intensityByRangeDiff(pcl::PointCloud<myPointXYZRID> point_cloud){
    std::cout<<"[intensityByRangeDiff] pointcloud size="<<point_cloud.size()<<std::endl;
    std::vector<std::vector<myPointXYZRID*>> rings(16);

    for(pcl::PointCloud<myPointXYZRID>::iterator pt = point_cloud.points.begin() ; pt < point_cloud.points.end(); pt++){
      if(std::isnan(pt->x)|| std::isnan(pt->y) || std::isnan(pt->z)){
        continue;
      }
      // std::cout<<pt->x<<" "<<pt->y<<" "<<pt->z<<" "<<pt->ring<<std::endl;
      pt->range = (pt->x * pt->x + pt->y * pt->y + pt->z * pt->z);
      rings[pt->ring].push_back(&(*pt));
    }
    std::cout<<"[intensityByRangeDiff] get each ring point"<<std::endl;
    typedef std::vector<std::vector<myPointXYZRID*>>::iterator iter;
    for(iter ring = rings.begin(); ring < rings.end(); ring++){
      myPointXYZRID* prev, *succ;
      if (ring->empty())
      {
        continue;
      }
      float last_intensity = (*ring->begin())->intensity;
      float new_intensity;
      (*ring->begin())->intensity = 0;
      (*(ring->end() - 1))->intensity = 0;
      for (std::vector<myPointXYZRID*>::iterator pt = ring->begin() + 1; pt < ring->end() - 1; pt++)
      {
        prev = *(pt - 1);
        succ = *(pt + 1);


        (*pt)->intensity = MAX( MAX( prev->range-(*pt)->range, succ->range-(*pt)->range), 0) * 10;
      }
    }
    std::cout<<"[intensityByRangeDiff] before normalizeIntensity"<<std::endl;

    point_cloud = normalizeIntensity(point_cloud, 0.0, 1.0);
    std::cout<<"[intensityByRangeDiff] after normalizeIntensity"<<std::endl;

    pcl::PointCloud<myPointXYZRID> filtered;

    for(pcl::PointCloud<myPointXYZRID>::iterator pt = point_cloud.points.begin() ; pt < point_cloud.points.end(); pt++)
    {
      // std::cout<<" "<<pt->x<<" "<<pt->y<<" "<<pt->z<<" "<<pt->intensity<<std::endl;
      if(pt->intensity  >  conf_data_.intensity_thresh)
      {
        if(pt->x >= conf_data_.xyz_[0].first && pt->x <= conf_data_.xyz_[0].second && pt->y >= conf_data_.xyz_[1].first && pt->y <= conf_data_.xyz_[1].second && pt->z >= conf_data_.xyz_[2].first && pt->z <= conf_data_.xyz_[2].second)
        {
          filtered.push_back(*pt);
        }
      }
    }
    std::cout<<"[intensityByRangeDiff] get filtered pointcloud"<<std::endl;

    //pcl::io::savePCDFileASCII ("/home/vishnu/PCDs/filtered.pcd", *(toPointsXYZ(filtered)));
    return filtered;
  }

  cv::Point Config::project(const pcl::PointXYZ &pt, const cv::Mat &projection_matrix)
  {
    //cv::Point2f xy = projectf(pt, projection_matrix);
    cv::Mat pt_3D(4, 1, CV_32FC1);

    pt_3D.at<float>(0) = pt.x;
    pt_3D.at<float>(1) = pt.y;
    pt_3D.at<float>(2) = pt.z;
    pt_3D.at<float>(3) = 1.0f;

    cv::Mat pt_2D = projection_matrix * pt_3D;

    float w = pt_2D.at<float>(2);
    float x = pt_2D.at<float>(0) / w;
    float y = pt_2D.at<float>(1) / w;
    return cv::Point(x, y);
  }

  cv::Mat Config::project(cv::Mat projection_matrix, cv::Rect frame, pcl::PointCloud<pcl::PointXYZ> point_cloud, pcl::PointCloud<pcl::PointXYZ> *visible_points)
  {
    cv::Mat plane = cv::Mat::zeros(frame.size(), CV_32FC1);

    for (pcl::PointCloud<pcl::PointXYZ>::iterator pt = point_cloud.points.begin(); pt < point_cloud.points.end(); pt++)
    {

      // behind the camera
      if (pt->z < 0)
      {
        continue;
      }

      //float intensity = pt->intensity;
      cv::Point xy = project(*pt, projection_matrix);
      if (xy.inside(frame))
      {
        if (visible_points != NULL)
        {
          visible_points->push_back(*pt);
        }

        //cv::circle(plane, xy, 3, intensity, -1);
        //plane.at<float>(xy) = intensity;
        plane.at<float>(xy)=250;
      }
    }

    cv::Mat plane_gray;
    cv::normalize(plane, plane_gray, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::dilate(plane_gray, plane_gray, cv::Mat());

    return plane_gray;
  }
}