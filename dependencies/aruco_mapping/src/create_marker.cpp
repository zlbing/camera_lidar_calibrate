
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

int main(int argc, char **argv)
{
	cv::Mat markerImage;
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
	cv::aruco::drawMarker(dictionary, 23, 800, markerImage, 1) ;
	cv::imwrite("marker1.png",markerImage);

	cv::aruco::drawMarker(dictionary, 10, 800, markerImage, 1);
	cv::imwrite("marker2.png",markerImage);

	// std::cout<<markerImage<<std::endl;
	return(0);
}
