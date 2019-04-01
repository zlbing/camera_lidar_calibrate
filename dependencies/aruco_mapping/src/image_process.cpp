#include    <ros/ros.h>
#include    <image_transport/image_transport.h>
#include    <aruco_mapping.h>
#include    <sensor_msgs/Image.h>

image_transport::Publisher pub_img;
ros::Publisher pub_info_camera;
sensor_msgs::CameraInfoPtr lcam(new sensor_msgs::CameraInfo());

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
  // cv::flip(cv_ptr->image, cv_ptr->image, -1);
  sensor_msgs::ImagePtr image_convert_msg = cv_ptr->toImageMsg();
  image_convert_msg->header.frame_id="camera_link";
  pub_img.publish(image_convert_msg);
  lcam->header.stamp = image_convert_msg->header.stamp;
  lcam->header.frame_id = image_convert_msg->header.frame_id;
  pub_info_camera.publish(lcam);
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"image_process");
  ros::NodeHandle nh;
  // Image node and subscribcer
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber img_sub = it.subscribe("/webcam/image_raw", 1, &imageCallback);
  pub_img = it.advertise("/image_correct", 10);
  pub_info_camera = nh.advertise<sensor_msgs::CameraInfo>("/camera_info", 1); //
  lcam->height = 480;
  lcam->width = 640;
  lcam->distortion_model = "plumb_bob";
  std::vector<double> distortion;
  distortion.push_back(0.029251);
  distortion.push_back(-0.169672);
  distortion.push_back(0.000825);
  distortion.push_back(0.004710);
  distortion.push_back(0);
   // {0.134304, -0.241875, 0.001620, -0.011621, 0.000000};
  lcam->D = distortion;
  lcam->K = {599.425037, 0.000000, 331.926573, 0.000000, 597.969408, 228.704189, 0.000000, 0.000000, 1.000000};
  lcam->R = {1.000000, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000, 0.000000, 0.000000, 1.000000};
  lcam->P = {595.430298, 0.000000, 334.968617, 0.000000, 0.000000, 598.599609, 228.841538, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000};
  ros::spin();
  return 0;
}