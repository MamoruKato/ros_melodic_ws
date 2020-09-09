#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "cv_bridge/cv_bridge.h"

#include <string>
#include <vector>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <geometry_msgs/Vector3.h>


const float actual_marker_length = 0.101;  // this should be in meters
const int wait_time = 10;
cv::Ptr<cv::aruco::DetectorParameters> detector_params;
cv::Ptr<cv::aruco::Dictionary> dictionary;
cv::Mat camera_matrix, dist_coeffs;
cv::Mat image, image_copy;
bool show_detections;
bool _break;
std::ostringstream vector_to_marker;



void finishDetection()
{
	cv::destroyAllWindows();
	ros::shutdown();
	exit(0);
}

void imageProcessingCallBack(const sensor_msgs::Image::ConstPtr& msg)
{
	std::string frame_id = msg->header.frame_id;
    auto image = cv_bridge::toCvShare(msg)->image;
    auto image_copy = image;
    cv::Mat display_image(image);

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners, rejected;
    cv::aruco::detectMarkers(image, dictionary, corners, ids, detector_params, rejected);

	if (ids.size() > 0)
    {

        cv::aruco::drawDetectedMarkers(image_copy, corners, ids);

        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners, actual_marker_length,
                camera_matrix, dist_coeffs, rvecs, tvecs);

		// draw axis for each marker
		
        for(int i=0; i < ids.size(); i++)
        {
            cv::aruco::drawAxis(image_copy, camera_matrix, dist_coeffs, 
            					rvecs[i], tvecs[i], 0.1);
            
            vector_to_marker.str(std::string());
            vector_to_marker << std::setprecision(4) 
                             << "x: " << std::setw(8)<<  tvecs[0](0);
            cv::putText(image_copy, vector_to_marker.str(),cvPoint(10, 30),
            			cv::FONT_HERSHEY_SIMPLEX, 0.6, cvScalar(0, 252, 124), 1, CV_AA);
            
            vector_to_marker.str(std::string());
            vector_to_marker << std::setprecision(4) 
                             << "y: " << std::setw(8) << tvecs[0](1); 
            cv::putText(image_copy, vector_to_marker.str(), 
                    cvPoint(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.6, 
                    cvScalar(0, 252, 124), 1, CV_AA);
            
            vector_to_marker.str(std::string());
            vector_to_marker << std::setprecision(4) 
                             << "z: " << std::setw(8) << tvecs[0](2);
            cv::putText(image_copy, vector_to_marker.str(), 
                    cvPoint(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.6, 
                    cvScalar(0, 252, 124), 1, CV_AA);

			//Set and publish aruco marker attributes and publishes it in the topic
			/*msg.ID = ids[i];
			msg.pose.x = tvecs[0](0);
			msg.pose.y = tvecs[0](1);
			msg.pose.z = tvecs[0](2);

			aruco_pose_pub.publish(msg);
			*/ 
			}

        cv::imshow("markers", image_copy);
            
    }

    cv::imshow("markers", image_copy);
    ids.clear();

    char key = (char) cv::waitKey(wait_time);
    if (key == 27)
    {
        _break = true;
        finishDetection();
    }
}

int main(int argc,char **argv)
{
	show_detections = false;
	_break = false;
	dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
	detector_params = cv::aruco::DetectorParameters::create();

	cv::FileStorage fs("/home/bruno/melodic_ws/src/simple_aruco_detector/config/calibration_params.yml", cv::FileStorage::READ);

    if(!fs.isOpened())
    {
    	std::cout << "Could not open calibration_params.yml" << std::endl;
    	return 0;
    }

	// Grab configuration parameters
    fs["camera_matrix"] >> camera_matrix;
    fs["distortion_coefficients"] >> dist_coeffs;

    fs.release();


	ros::init(argc, argv, "gazebo_aruco_detector");
	ros::NodeHandle n;

	ros::Subscriber image_sub = n.subscribe("my_robot/camera1/image_raw",1000, imageProcessingCallBack);
	ros::Rate loop_rate(1);

	cv::namedWindow("markers", cv::WINDOW_KEEPRATIO);

	while(ros::ok())
	{
		ros::spin();
		loop_rate.sleep();
	}

	return 0;
}