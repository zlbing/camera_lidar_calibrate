#include    <ros/ros.h>
#include    <image_transport/image_transport.h>
#include    <aruco_mapping.h>
#include    <sensor_msgs/Image.h>
#include    <opencv2/highgui/highgui.hpp>
#include    "lidar_camera_calibration/marker_6dof.h"

image_transport::Publisher pub_img,pub_rect_img;
ros::Publisher pub_info_camera;
sensor_msgs::CameraInfo lcam;
cv::Mat Camera_Matrix(3,3,CV_64F);
cv::Mat Distortion_Coefficients(5, 1, CV_64F);
std::string calib_filename_;
bool need_flip;
void imageCallback(const sensor_msgs::ImageConstPtr &original_image)
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
  if(need_flip){
    cv::flip(cv_ptr->image, cv_ptr->image, -1);
  }

  sensor_msgs::ImagePtr image_convert_msg = cv_ptr->toImageMsg();
  image_convert_msg->header.frame_id="camera_link";
  pub_img.publish(image_convert_msg);

  cv::Mat u_image;
  cv::undistort(cv_ptr->image, u_image, Camera_Matrix, Distortion_Coefficients);
  cv_ptr->image = u_image;
  cv_ptr->header.frame_id = "camera_link";
  pub_rect_img.publish(cv_ptr->toImageMsg());

  lcam.header.stamp = image_convert_msg->header.stamp;
  lcam.header.frame_id = image_convert_msg->header.frame_id;
  pub_info_camera.publish(lcam);
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"image_process");
  ros::NodeHandle nh,p_nh("~");
  // Image node and subscribcer
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber img_sub = it.subscribe("/rgb_image_hd", 1, &imageCallback);
  pub_img = it.advertise("/image_correct", 10);
  pub_rect_img = it.advertise("/image_rectity", 10);
  pub_info_camera = nh.advertise<sensor_msgs::CameraInfo>("/camera_info", 1); //
  p_nh.getParam("calibration_file", calib_filename_);
  std::cout<<"calib_filename_="<<calib_filename_<<std::endl;
  p_nh.getParam("flip", need_flip);

  std::string camera_name = "camera";
  camera_calibration_parsers::readCalibrationIni(calib_filename_, camera_name, lcam);
  for(int i=0; i<Camera_Matrix.rows; i++){
    for(int j=0; j<Camera_Matrix.cols; j++){
      Camera_Matrix.at<double>(i,j) =lcam.K[i*Camera_Matrix.cols + j];
    }
  }
  for(int i=0; i<5; i++){
    Distortion_Coefficients.at<double>(i,0)=lcam.D[i];
  }

//  lcam->height = 480;
//  lcam->width = 640;
//  lcam->distortion_model = "plumb_bob";
//  std::vector<double> distortion;
//  distortion.push_back(0.146531);
//  distortion.push_back(-0.313771);
//  distortion.push_back(-0.002755);
//  distortion.push_back(-0.000606);
//  distortion.push_back(0);
//  for(int i=0; i<distortion.size(); i++){
//    Distortion_Coefficients.at<double>(i,0)=distortion[i];
//  }
//  lcam->D = distortion;
//  lcam->K = {629.618893, 0.000000, 303.660413, 0.000000, 630.227451, 231.450386, 0.000000, 0.000000, 1.000000};
//  lcam->R = {1.000000, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000, 0.000000, 0.000000, 1.000000};
//  lcam->P = {640.475647, 0.000000, 303.188514, 0.000000, 0.000000, 641.105835, 230.526435, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000};
//  for(int i=0; i<Camera_Matrix.rows; i++){
//    for(int j=0; j<Camera_Matrix.cols; j++){
//      Camera_Matrix.at<double>(i,j) =lcam->K[i*Camera_Matrix.cols + j];
//    }
//  }
  ros::spin();
  return 0;
}