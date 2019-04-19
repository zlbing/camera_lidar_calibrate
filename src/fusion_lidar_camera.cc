//
// Created by zzz on 19-4-18.
//

#include "fusion_lidar_camera.h"
namespace GS{
  FusionLidarCamera::FusionLidarCamera(){
    config_.readConfig();
    iteration_count = 0;

    ros::NodeHandle n;

    pub_filter_points_ =  n.advertise<sensor_msgs::PointCloud2>("filter_points",10);
    pub_camera_corner_ =  n.advertise<sensor_msgs::PointCloud2>("camera_corner",10);
    pub_lidar_corner_ =  n.advertise<sensor_msgs::PointCloud2>("lidar_corner",10);
  }

  void FusionLidarCamera::callback(const sensor_msgs::ImageConstPtr &original_image,
                                   const sensor_msgs::PointCloud2ConstPtr& msg_pc,
                                   const lidar_camera_calibration::marker_6dof::ConstPtr& msg_rt){
    ROS_INFO_STREAM("Velodyne scan received at " << msg_pc->header.stamp.toSec());
    ROS_INFO_STREAM("marker_6dof received at " << msg_rt->header.stamp.toSec());
    ROS_INFO_STREAM("image received at " << original_image->header.stamp.toSec());

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

    // Loading Velodyne point cloud_sub
    pcl::PointCloud<pcl::PointXYZI> points;
    pcl::fromROSMsg(*msg_pc, points);
    pcl::PointCloud<myPointXYZRID> point_cloud;
    float lowerBound = -15;
    float upperBound = 15;
    float nScanRings = 16;
    float factor = (nScanRings - 1) / (upperBound - lowerBound);
    for(int i=0; i<points.points.size(); i++){
      Eigen::Vector4f point_in_sensor(points.points[i].x,
                                      points.points[i].y,
                                      points.points[i].z,
                                      points.points[i].intensity);
      if(std::isnan(point_in_sensor.y()) ||
         std::isnan(point_in_sensor.x())||
         std::isnan(point_in_sensor.z())){
        continue;
      }
      float angle = std::atan(point_in_sensor.z() / std::sqrt(point_in_sensor.x() * point_in_sensor.x()
                                                              + point_in_sensor.y() * point_in_sensor.y()));
      int scanID = int(((angle * 180 / M_PI) - lowerBound) * factor + 0.5);

      myPointXYZRID point;
      point.x = point_in_sensor[0];
      point.y = point_in_sensor[1];
      point.z = point_in_sensor[2];
      point.intensity = point_in_sensor[3];
      point.ring = scanID;
      point_cloud.push_back(point);
    }
    Eigen::Quaterniond qlidarToCamera;
    Eigen::Matrix3d lidarToCamera;
    //Rotation matrix to transform lidar point cloud to camera's frame
    std::cout<<"rotation angle x="<<config_.conf_data_.initialRot[0]
             <<" y="<<config_.conf_data_.initialRot[1]
             <<" z="<<config_.conf_data_.initialRot[2]<<std::endl;;
    qlidarToCamera = Eigen::AngleAxisd(config_.conf_data_.initialRot[2], Eigen::Vector3d::UnitZ())
                     *Eigen::AngleAxisd(config_.conf_data_.initialRot[1], Eigen::Vector3d::UnitY())
                     *Eigen::AngleAxisd(config_.conf_data_.initialRot[0], Eigen::Vector3d::UnitX());

    lidarToCamera = qlidarToCamera.matrix();

    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.rotate(qlidarToCamera);
    transform.translation() << 0, 0.0, 0.0;
    pcl::transformPointCloud(point_cloud, point_cloud, transform);

    pcl::PointCloud<pcl::PointXYZ> point_cloud_xyz = *(config_.toPointsXYZ(point_cloud));
    cv::Mat temp_image(config_.conf_data_.s, CV_8UC3);

    std::cout << "\n\nInitial Rot" << lidarToCamera << "\n";
    point_cloud = config_.intensityByRangeDiff(point_cloud);

    cv::Mat temp_mat(config_.conf_data_.s, CV_8UC3);
    pcl::PointCloud<pcl::PointXYZ> retval = *(config_.toPointsXYZ(point_cloud));
    std::vector<std::vector<std::vector<Eigen::Vector3d>>> line_points =
            getCorners(temp_mat, retval, config_.conf_data_.P,
               config_.conf_data_.num_of_markers,
               config_.conf_data_.MAX_ITERS);

    std::vector<float> marker_info;
    std::cout<<"[callback_noCam] get marker_info"<<std::endl;
    for(std::vector<float>::const_iterator it = msg_rt->dof.data.begin(); it != msg_rt->dof.data.end(); ++it)
    {
      marker_info.push_back(*it);
      std::cout << *it << " ";
    }
    std::cout << "\n";
    std::cout<<"[callback_noCam] marker_info size="<<marker_info.size()<<std::endl;

    Eigen::Matrix4d T_lidar_to_camera = compute_rt_.find_transformation(marker_info, config_.conf_data_.num_of_markers,
                                                     config_.conf_data_.MAX_ITERS, lidarToCamera);

    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> point_clouds = compute_rt_.readArray();

    Eigen::Matrix4d T_lidar_to_camera_ceres = compute_rt_.findTByCeres(marker_info,
                                                                       line_points,
                                                                       config_.conf_data_.num_of_markers,
                                                                       config_.conf_data_.MAX_ITERS,
                                                                       lidarToCamera);

    {
      pcl::PointCloud<pcl::PointXYZ> poing_edge;

      for(int i=0; i<compute_rt_.all_line_points.size(); i++){
        for(int j=0; j<compute_rt_.all_line_points[i].size(); j++){
          for(int k=0; k<compute_rt_.all_line_points[i][j].size(); k++){
            Eigen::Vector3d p = compute_rt_.all_line_points[i][j][k];
            pcl::PointXYZ pcl_p(p.x(),p.y(),p.z());
            poing_edge.push_back(pcl_p);
          }
        }
      }

      pcl::transformPointCloud(poing_edge, poing_edge, transform.inverse());
      sensor_msgs::PointCloud2 output;
      pcl::toROSMsg(poing_edge,output);
      output.header = msg_pc->header;
      output.header.frame_id = "rslidar";
      for(int i=0; i<10; i++){
        pub_filter_points_.publish(output);
        usleep(500);
      }
      Eigen::Matrix4d T_camera_to_lidar = T_lidar_to_camera_ceres.inverse();

      int camera_corner_count=0;
      pcl::PointCloud<pcl::PointXYZ> point_camera;
      for(int i=0; i<compute_rt_.all_corner_points.size(); i++){
        for(int j=0; j<compute_rt_.all_corner_points[i].size(); j++){
          Eigen::Vector3d p = compute_rt_.all_corner_points[i][j];
          pcl::PointXYZ pcl_p(p.x(),p.y(),p.z());
          point_camera.push_back(pcl_p);
          camera_corner_count++;
        }
      }
      std::cout<<"camera_corner_count="<<camera_corner_count<<std::endl;
      pcl::transformPointCloud(point_camera, point_camera, T_camera_to_lidar);
      pcl::transformPointCloud(point_camera, point_camera, transform.inverse());
      sensor_msgs::PointCloud2 output_camera_corner;
      pcl::toROSMsg(point_camera,output_camera_corner);
      output_camera_corner.header = msg_pc->header;
      output_camera_corner.header.frame_id = "rslidar";

      for(int i=0; i<10; i++){
        pub_camera_corner_.publish(output_camera_corner);
        usleep(500);
      }

    //投影lidar corner 点云
      pcl::PointCloud<pcl::PointXYZ> point_cloud_corner;
      for(int i=0; i<point_clouds.first.cols(); i++){
        pcl::PointXYZ lidar_e(point_clouds.first(0,i),
                              point_clouds.first(1,i),
                              point_clouds.first(2,i));
        point_cloud_corner.push_back(lidar_e);
      }

      pcl::transformPointCloud(point_cloud_corner, point_cloud_corner, transform.inverse());
      sensor_msgs::PointCloud2 output_lidar_corner;
      pcl::toROSMsg(point_cloud_corner,output_lidar_corner);
      output_lidar_corner.header = msg_pc->header;
      output_lidar_corner.header.frame_id = "rslidar";

      for(int i=0; i<10; i++){
        pub_lidar_corner_.publish(output_lidar_corner);
        usleep(500);
      }
    }
  }

  std::vector<std::vector<std::vector<Eigen::Vector3d>>>
  FusionLidarCamera::getCorners(cv::Mat img, pcl::PointCloud<pcl::PointXYZ> scan, cv::Mat P, int num_of_markers, int MAX_ITERS)
{
    ROS_INFO_STREAM("iteration number: " << iteration_count << "\n");

    /*Masking happens here */
    cv::Mat edge_mask = cv::Mat::zeros(img.size(), CV_8UC1);
    //edge_mask(cv::Rect(520, 205, 300, 250))=1;
    edge_mask(cv::Rect(0, 0, img.cols, img.rows))=1;
    img.copyTo(edge_mask, edge_mask);
    //pcl::io::savePCDFileASCII ("/home/vishnu/final1.pcd", scan.point_cloud);

    img = edge_mask;

    // cv:imwrite("/home/zzz/marker.png", edge_mask);

    pcl::PointCloud<pcl::PointXYZ> pc = scan;
    //scan = Velodyne::Velodyne(filtered_pc);

    cv::Rect frame(0, 0, img.cols, img.rows);

    // pcl::io::savePCDFileASCII("/home/vishnu/final2.pcd", scan.point_cloud);

    cv::Mat image_edge_laser = config_.project(P, frame, scan, NULL);
    std::cout<<"[getCorners] image_edge_laser"<<"scan size="<<scan.size()<<std::endl;
    // <<"\n"<<image_edge_laser<<std::endl;
    cv::threshold(image_edge_laser, image_edge_laser, 10, 255, 0);

//	cv::imshow("image_edge_laser", image_edge_laser);
//	cv::waitKey(0);


    cv::Mat combined_rgb_laser;
    std::vector<cv::Mat> rgb_laser_channels;

    rgb_laser_channels.push_back(image_edge_laser);
    rgb_laser_channels.push_back(cv::Mat::zeros(image_edge_laser.size(), CV_8UC1));
    rgb_laser_channels.push_back(img);

    cv::merge(rgb_laser_channels, combined_rgb_laser);
    // cv::namedWindow("combined", cv::WINDOW_NORMAL);
    // cv::imshow("combined", combined_rgb_laser);
    // cv::waitKey(5);

    std::map<std::pair<int, int>, std::vector<float> > c2D_to_3D;
    std::vector<float> point_3D;

    /* store correspondences */
    for(pcl::PointCloud<pcl::PointXYZ>::iterator pt = pc.points.begin(); pt < pc.points.end(); pt++)
    {

      // behind the camera
      if (pt->z < 0)
      {
        continue;
      }

      cv::Point xy = config_.project(*pt, P);
      if (xy.inside(frame))
      {
        //create a map of 2D and 3D points
        point_3D.clear();
        point_3D.push_back(pt->x);
        point_3D.push_back(pt->y);
        point_3D.push_back(pt->z);
        c2D_to_3D[std::pair<int, int>(xy.x, xy.y)] = point_3D;
      }
    }

    /* print the correspondences */
    std::cout<<"[getCorners] print the correspondences"<<std::endl;
    typedef std::map<std::pair<int, int>, std::vector<float> >::iterator map_iter;
    for(map_iter it=c2D_to_3D.begin(); it!=c2D_to_3D.end(); ++it)
    {
      std::cout << it->first.first << "," << it->first.second << " --> " << it->second[0] << "," <<it->second[1] << "," <<it->second[2] << "\n";
    }

    /* get region of interest */

    const int QUADS=num_of_markers;
    std::vector<int> LINE_SEGMENTS(QUADS, 4); //assuming each has 4 edges and 4 corners

    pcl::PointCloud<pcl::PointXYZ>::Ptr board_corners(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr marker(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<cv::Point3f> c_3D;
    std::vector<cv::Point2f> c_2D;


    cv::namedWindow("cloud", cv::WINDOW_NORMAL);
    cv::namedWindow("polygon", cv::WINDOW_NORMAL);
    //cv::namedWindow("combined", cv::WINDOW_NORMAL);

    std::string pkg_loc = ros::package::getPath("lidar_camera_calibration");
    std::ofstream outfile(pkg_loc + "/conf/points.txt", std::ios_base::trunc);
    outfile << QUADS*4 << "\n";
    std::vector<std::vector<std::vector<Eigen::Vector3d>>> line_points;
    line_points.resize(QUADS);
    for(int q=0; q<QUADS; q++)
    {
      std::cout << "---------Moving on to next marker--------\n";
      std::vector<Eigen::VectorXf> line_model;
      line_points[q].resize(LINE_SEGMENTS[q]);
      for(int i=0; i<LINE_SEGMENTS[q]; i++)
      {
        cv::Point _point_;
        std::vector<cv::Point> polygon;
        int collected;

        // get markings in the first iteration only
        if(iteration_count == 0)
        {
          polygon.clear();
          collected = 0;
          while(collected != LINE_SEGMENTS[q])
          {
            cv::setMouseCallback("cloud", onMouse, &_point_);
            std::cout<<"[getCorners] mouse get out"<<std::endl;
            cv::imshow("cloud", image_edge_laser);
            cv::waitKey(0);
            ++collected;
            std::cout <<"[getCorners] mouse stelected point="<< _point_.x << " " << _point_.y << "\n";
            polygon.push_back(_point_);
            std::cout<<"[getCorners] polygon size="<<polygon.size()<<std::endl;
          }
          stored_corners.push_back(polygon);
        }
        std::cout<<"[getCorners] stored_corners="<<stored_corners.size()<<std::endl;
        polygon = stored_corners[4*q+i];

        cv::Mat polygon_image = cv::Mat::zeros(image_edge_laser.size(), CV_8UC1);

        rgb_laser_channels.clear();
        rgb_laser_channels.push_back(image_edge_laser);
        rgb_laser_channels.push_back(cv::Mat::zeros(image_edge_laser.size(), CV_8UC1));
        rgb_laser_channels.push_back(cv::Mat::zeros(image_edge_laser.size(), CV_8UC1));
        cv::merge(rgb_laser_channels, combined_rgb_laser);

        for( int j = 0; j < 4; j++ )
        {
          cv::line(combined_rgb_laser, polygon[j], polygon[(j+1)%4], cv::Scalar(0, 255, 0));
        }

        // initialize PointClouds
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);

        for(map_iter it=c2D_to_3D.begin(); it!=c2D_to_3D.end(); ++it)
        {
          if (cv::pointPolygonTest(cv::Mat(polygon), cv::Point(it->first.first, it->first.second), true) > 0)
          {
            cloud->push_back(pcl::PointXYZ(it->second[0],it->second[1],it->second[2]));
            cv::rectangle(combined_rgb_laser, cv::Point(it->first.first, it->first.second), cv::Point(it->first.first, it->first.second), cv::Scalar(0, 0, 255), 3, 8, 0); // RED point
          }
        }


        cv::imshow("polygon", combined_rgb_laser);
        cv::waitKey(4);

        //pcl::io::savePCDFileASCII("/home/vishnu/line_cloud.pcd", *cloud);



        std::vector<int> inliers;
        Eigen::VectorXf model_coefficients;


        // created RandomSampleConsensus object and compute the appropriated model
        pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_l(new pcl::SampleConsensusModelLine<pcl::PointXYZ> (cloud));

        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_l);
        ransac.setDistanceThreshold (0.01);
        ransac.computeModel();
        ransac.getInliers(inliers);
        ransac.getModelCoefficients(model_coefficients);
        line_model.push_back(model_coefficients);

        std::cout << "Line coefficients are:" << "\n" << model_coefficients << "\n";
        // copies all inliers of the model computed to another PointCloud
        pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);
        //pcl::io::savePCDFileASCII("/home/vishnu/RANSAC_line_cloud.pcd", *final);
        *marker += *final;
        for(int k=0; k<final->points.size(); k++){
          Eigen::Vector3d p(final->points[k].x,final->points[k].y,final->points[k].z);
          line_points[q][i].push_back(p);
        }
      }

      /* calculate approximate intersection of lines */


      Eigen::Vector4f p1, p2, p_intersect;
      pcl::PointCloud<pcl::PointXYZ>::Ptr corners(new pcl::PointCloud<pcl::PointXYZ>);
      for(int i=0; i<LINE_SEGMENTS[q]; i++)
      {
        pcl::lineToLineSegment(line_model[i], line_model[(i+1)%LINE_SEGMENTS[q]], p1, p2);
        for(int j=0; j<4; j++)
        {
          p_intersect(j) = (p1(j) + p2(j))/2.0;
        }
        c_3D.push_back(cv::Point3f(p_intersect(0), p_intersect(1), p_intersect(2)));
        corners->push_back(pcl::PointXYZ(p_intersect(0), p_intersect(1), p_intersect(2)));
        std::cout << "Point of intersection is approximately: \n" << p_intersect << "\n";
        //std::cout << "Distance between the lines: " << (p1 - p2).squaredNorm () << "\n";
        std::cout << p_intersect(0) << " " << p_intersect(1) << " " << p_intersect(2) <<  "\n";
        outfile << p_intersect(0) << " " << p_intersect(1) << " " << p_intersect(2) <<  "\n";
      }

      *board_corners += *corners;

      std::cout << "Distance between the corners:\n";
      for(int i=0; i<4; i++)
      {
        std::cout <<

                  sqrt(
                          pow(c_3D[4*q+i].x - c_3D[4*q+(i+1)%4].x, 2)
                          + pow(c_3D[4*q+i].y - c_3D[4*q+(i+1)%4].y, 2)
                          + pow(c_3D[4*q+i].z - c_3D[4*q+(i+1)%4].z, 2)
                  )

                  << std::endl;
      }


    }
    outfile.close();

    iteration_count++;
    if(iteration_count == MAX_ITERS)
    {
      ros::shutdown();
    }
    return line_points;
  }
}