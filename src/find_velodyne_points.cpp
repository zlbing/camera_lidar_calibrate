#include <cstdlib>
#include <cstdio>
#include <math.h>
#include <algorithm>
#include <map>

#include "opencv2/opencv.hpp"

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
#include <velodyne_pointcloud/point_types.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>

#include "lidar_camera_calibration/Corners.h"
#include "lidar_camera_calibration/PreprocessUtils.h"
#include "lidar_camera_calibration/Find_RT.h"

#include "lidar_camera_calibration/marker_6dof.h"


using namespace cv;
using namespace std;
using namespace ros;
using namespace message_filters;
using namespace pcl;


string CAMERA_INFO_TOPIC;
string VELODYNE_TOPIC;


Mat projection_matrix;

pcl::PointCloud<myPointXYZRID> point_cloud;

Eigen::Quaterniond qlidarToCamera; 
Eigen::Matrix3d lidarToCamera;
ros::Publisher pub_filter_points_;

ros::Publisher pub_camera_corner_,pub_lidar_corner_;

geometry_msgs::TransformStamped transformStamped;

cv::Point cvproject(const pcl::PointXYZ &pt, const cv::Mat &projection_matrix)
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

void callback_lidar(const sensor_msgs::PointCloud2ConstPtr& msg_pc){
	ROS_INFO_STREAM("[test] Velodyne scan received at " << msg_pc->header.stamp.toSec());
}

void callback_marker_6dof(const lidar_camera_calibration::marker_6dof::ConstPtr& msg_rt){
	ROS_INFO_STREAM("[test] marker_6dof received at " << msg_rt->header.stamp.toSec());
}

void callback_noCam(const sensor_msgs::ImageConstPtr &original_image,
					const sensor_msgs::PointCloud2ConstPtr& msg_pc,
					const lidar_camera_calibration::marker_6dof::ConstPtr& msg_rt)
{
	ROS_INFO_STREAM("Velodyne scan received at " << msg_pc->header.stamp.toSec());
	ROS_INFO_STREAM("marker_6dof received at " << msg_rt->header.stamp.toSec());

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
	pcl::PointCloud<PointXYZI> points;
	fromROSMsg(*msg_pc, points);

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
	// point_cloud = transform(point_cloud, 0, 0, 0, config.initialRot[0], config.initialRot[1], config.initialRot[2]);

	//Rotation matrix to transform lidar point cloud to camera's frame
	std::cout<<"rotation angle x="<<config.initialRot[0]
			 <<" y="<<config.initialRot[1]
			 <<" z="<<config.initialRot[2]<<std::endl;;
	qlidarToCamera = Eigen::AngleAxisd(config.initialRot[2], Eigen::Vector3d::UnitZ())
					 *Eigen::AngleAxisd(config.initialRot[1], Eigen::Vector3d::UnitY())
					 *Eigen::AngleAxisd(config.initialRot[0], Eigen::Vector3d::UnitX());

	lidarToCamera = qlidarToCamera.matrix();

	Eigen::Affine3d transform = Eigen::Affine3d::Identity();
	transform.rotate(qlidarToCamera);
	transform.translation() << 0, 0.0, 0.0;
	pcl::transformPointCloud(point_cloud, point_cloud, transform);

	pcl::PointCloud<pcl::PointXYZ> point_cloud_xyz = *(toPointsXYZ(point_cloud));
	cv::Mat temp_image(config.s, CV_8UC3);
//	cvshowProjectPoints(temp_image, point_cloud_xyz, config.P);

	std::cout << "\n\nInitial Rot" << lidarToCamera << "\n";
	point_cloud = intensityByRangeDiff(point_cloud, config);
	// x := x, y := -z, z := y

	cv::Mat temp_mat(config.s, CV_8UC3);
	pcl::PointCloud<pcl::PointXYZ> retval = *(toPointsXYZ(point_cloud));

	std::vector<float> marker_info;
	std::cout<<"[callback_noCam] get marker_info"<<std::endl;
	for(std::vector<float>::const_iterator it = msg_rt->dof.data.begin(); it != msg_rt->dof.data.end(); ++it)
	{
		marker_info.push_back(*it);
		std::cout << *it << " ";
	}
	std::cout << "\n";
	std::cout<<"[callback_noCam] marker_info size="<<marker_info.size()<<std::endl;

	getCorners(temp_mat, retval, config.P, config.num_of_markers, config.MAX_ITERS);
	Matrix4d T_lidar_to_camera = find_transformation(marker_info, config.num_of_markers, config.MAX_ITERS, lidarToCamera);
	Matrix4d T_camera_to_lidar = T_lidar_to_camera.inverse();
	cv::namedWindow("prject_points", cv::WINDOW_NORMAL);
//	pcl::transformPointCloud(point_cloud_xyz, point_cloud_xyz, transform.inverse());
	pcl::transformPointCloud(point_cloud_xyz, point_cloud_xyz, T_lidar_to_camera);
	cv::Mat image = cv_ptr->image;
	cv::Rect frame(0, 0, image.cols+20, image.rows+20);
//	for (pcl::PointCloud<pcl::PointXYZ>::iterator pt = point_cloud_xyz.points.begin();
//		 pt < point_cloud_xyz.points.end(); pt++)
//	{
//		// behind the camera
//		if (pt->z < 0)
//		{
//			continue;
//		}
//		cv::Point point = cvproject(*pt,config.P);
//		if (point.inside(frame))
//		{
//			double len = std::min(1.0,(pt->z-1.0)/(3-1.0));
//			cv::circle(image,point,2,cv::Scalar(255*(1-len),255*(len),0),2);
//		}
//	}
	//投影edge 点云
	pcl::PointCloud<pcl::PointXYZ> poing_edge;
	pcl::transformPointCloud(retval, poing_edge, T_lidar_to_camera);
	for(int i=0;i<poing_edge.size();i++){
		if(poing_edge.points[i].z < 0){
			continue;
		}
		cv::Point p_edge =  cvproject(poing_edge.points[i],config.P);
		if (p_edge.inside(frame))
		{
			cv::circle(image,p_edge,2,cv::Scalar(100,100,0),2);
		}
	}

	pcl::transformPointCloud(retval, poing_edge, transform.inverse());
	sensor_msgs::PointCloud2 output;
	pcl::toROSMsg(poing_edge,output);
	output.header = msg_pc->header;
	output.header.frame_id = "rslidar";
	for(int i=0; i<10; i++){
		pub_filter_points_.publish(output);
		usleep(500);
	}

	std::pair<MatrixXd, MatrixXd> point_clouds = readArray();
	//投影camera corner 点云
	pcl::PointCloud<pcl::PointXYZ> point_camera;
	for(int i=0; i<point_clouds.first.cols(); i++){
		pcl::PointXYZ c(point_clouds.second(0,i),
						point_clouds.second(1,i),
						point_clouds.second(2,i));
		point_camera.push_back(c);
		cv::Point c_point = cvproject(c,config.P);
		cv::circle(image,c_point,2,cv::Scalar(255,0,0),2);
	}

	pcl::transformPointCloud(point_camera, point_camera, T_camera_to_lidar);
	pcl::transformPointCloud(point_camera, point_camera, transform.inverse());
	sensor_msgs::PointCloud2 output_camera_corner;
	pcl::toROSMsg(point_camera,output_camera_corner);
	output_camera_corner.header = msg_pc->header;
	output_camera_corner.header.frame_id = "rslidar";


	//投影lidar corner 点云
	pcl::PointCloud<pcl::PointXYZ> point_cloud_corner;
	for(int i=0; i<point_clouds.first.cols(); i++){
		pcl::PointXYZ lidar_e(point_clouds.first(0,i),
							  point_clouds.first(1,i),
							  point_clouds.first(2,i));
		point_cloud_corner.push_back(lidar_e);
	}

	pcl::transformPointCloud(point_cloud_corner, point_cloud_corner, T_lidar_to_camera);
	for(int i=0;i<point_cloud_corner.size(); i++){
		cv::Point l_point = cvproject(point_cloud_corner[i],config.P);
		std::cout<<"l_point xy="<<l_point.x<<" "<<l_point.y<<std::endl;
		cv::circle(image,l_point,2,cv::Scalar(0,0,255),2);
	}

	pcl::transformPointCloud(point_cloud_corner, point_cloud_corner, T_camera_to_lidar);
	pcl::transformPointCloud(point_cloud_corner, point_cloud_corner, transform.inverse());
	sensor_msgs::PointCloud2 output_lidar_corner;
	pcl::toROSMsg(point_cloud_corner,output_lidar_corner);
	output_lidar_corner.header = msg_pc->header;
	output_lidar_corner.header.frame_id = "rslidar";

	cv::imshow("prject_points",image);
	cv::waitKey(2);
	pub_camera_corner_.publish(output_camera_corner);
	pub_lidar_corner_.publish(output_lidar_corner);
	//ros::shutdown();
}

void callback(const sensor_msgs::CameraInfoConstPtr& msg_info,
			  const sensor_msgs::PointCloud2ConstPtr& msg_pc,
			  const lidar_camera_calibration::marker_6dof::ConstPtr& msg_rt)
{

	ROS_INFO_STREAM("Camera info received at " << msg_info->header.stamp.toSec());
	ROS_INFO_STREAM("Velodyne scan received at " << msg_pc->header.stamp.toSec());
	ROS_INFO_STREAM("marker_6dof received at " << msg_rt->header.stamp.toSec());

	float p[12];
	float *pp = p;
	for (boost::array<double, 12ul>::const_iterator i = msg_info->P.begin(); i != msg_info->P.end(); i++)
	{
	*pp = (float)(*i);
	pp++;
	}
	cv::Mat(3, 4, CV_32FC1, &p).copyTo(projection_matrix);



	// Loading Velodyne point cloud_sub
	fromROSMsg(*msg_pc, point_cloud);

	point_cloud = transform(point_cloud, 0, 0, 0, config.initialRot[0], config.initialRot[1], config.initialRot[2]);

	//Rotation matrix to transform lidar point cloud to camera's frame

	qlidarToCamera = Eigen::AngleAxisd(config.initialRot[2], Eigen::Vector3d::UnitZ())
		*Eigen::AngleAxisd(config.initialRot[1], Eigen::Vector3d::UnitY())
		*Eigen::AngleAxisd(config.initialRot[0], Eigen::Vector3d::UnitX());

	lidarToCamera = qlidarToCamera.matrix();

	point_cloud = intensityByRangeDiff(point_cloud, config);
	// x := x, y := -z, z := y

	//pcl::io::savePCDFileASCII ("/home/vishnu/PCDs/msg_point_cloud.pcd", pc);  


	cv::Mat temp_mat(config.s, CV_8UC3);
	pcl::PointCloud<pcl::PointXYZ> retval = *(toPointsXYZ(point_cloud));

	std::vector<float> marker_info;

	for(std::vector<float>::const_iterator it = msg_rt->dof.data.begin(); it != msg_rt->dof.data.end(); ++it)
	{
		marker_info.push_back(*it);
		std::cout << *it << " ";
	}
	std::cout << "\n";

	getCorners(temp_mat, retval, projection_matrix, config.num_of_markers, config.MAX_ITERS);
	find_transformation(marker_info, config.num_of_markers, config.MAX_ITERS, lidarToCamera);
	//ros::shutdown();
}


int main(int argc, char** argv)
{
	readConfig();
	ros::init(argc, argv, "find_transform");

	ros::NodeHandle n;

	pub_filter_points_ =  n.advertise<sensor_msgs::PointCloud2>("filter_points",10);
	pub_camera_corner_ =  n.advertise<sensor_msgs::PointCloud2>("camera_corner",10);
	pub_lidar_corner_ =  n.advertise<sensor_msgs::PointCloud2>("lidar_corner",10);
	if(config.useCameraInfo)
	{
		ROS_INFO_STREAM("Reading CameraInfo from topic");
		n.getParam("/lidar_camera_calibration/camera_info_topic", CAMERA_INFO_TOPIC);
		n.getParam("/lidar_camera_calibration/velodyne_topic", VELODYNE_TOPIC);

		message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(n, CAMERA_INFO_TOPIC, 1);
		message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(n, VELODYNE_TOPIC, 1);
		message_filters::Subscriber<lidar_camera_calibration::marker_6dof> rt_sub(n, "lidar_camera_calibration_rt", 1);

		std::cout << "done1\n";

		typedef sync_policies::ApproximateTime<sensor_msgs::CameraInfo, sensor_msgs::PointCloud2, lidar_camera_calibration::marker_6dof> MySyncPolicy;
		Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), info_sub, cloud_sub, rt_sub);
		sync.registerCallback(boost::bind(&callback, _1, _2, _3));

		ros::spin();
	}
	else
	{
		ROS_INFO_STREAM("Reading CameraInfo from configuration file");
  		n.getParam("/lidar_camera_calibration/velodyne_topic", VELODYNE_TOPIC);
  		ROS_INFO_STREAM("VELODYNE_TOPIC="<<VELODYNE_TOPIC);
		message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(n, VELODYNE_TOPIC, 10);
		message_filters::Subscriber<lidar_camera_calibration::marker_6dof> rt_sub(n, "lidar_camera_calibration_rt", 10);
		message_filters::Subscriber<sensor_msgs::Image> img_sub(n, "/image_raw", 10);

		typedef sync_policies::ApproximateTime<sensor_msgs::Image,
												sensor_msgs::PointCloud2,
												lidar_camera_calibration::marker_6dof> MySyncPolicy;
		Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), img_sub, cloud_sub, rt_sub);
		sync.registerCallback(boost::bind(&callback_noCam, _1, _2, _3));

		ros::Subscriber sub_lidar = n.subscribe(VELODYNE_TOPIC, 100, callback_lidar);
		ros::Subscriber sub_marker_6d0f = n.subscribe("/lidar_camera_calibration_rt", 100, callback_marker_6dof);
		ros::spin();
	}

	return EXIT_SUCCESS;
}

