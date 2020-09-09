/*
 * Copyright (c) 2018 Flight Dynamics and Control Lab
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */


#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <string>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <simple_aruco_detector/aruco_msg.h>


int main(int argc, char **argv)
{

	//VARIABLE DECLARATION

    FILE *file = NULL;
    int wait_time = 10;
    char teste;
    char ardConnected;
    int camId = 0;
    char received;
    int send = 0;
    float actual_marker_length = 0.101;  // this should be in meters
    cv::VideoCapture in_video;
    cv::Mat camera_matrix, dist_coeffs;
    cv::Mat image, image_copy;

	//Initiate ros and ros node handle for publishing aruco marker pose in a topic named aruco_pose
   	ros::init(argc, argv, "aruco_marker_detector");
    ros::NodeHandle n;

    ros::Publisher aruco_pose_pub = n.advertise<simple_aruco_detector::aruco_msg>("aruco_pose", 1000); //simple_aruco_detector::aruco_msg -> msg created
    simple_aruco_detector::aruco_msg msg;

    //Get location of camera configuration file from param server
    //the location is seted at launch file aruco_marker_detector
    std::string config_file;
    n.getParam("/aruco_marker_detector_node/camera_config_file_value",config_file);

    std::cout << config_file << std::endl;
	//TODO get a better way for getting the configuration file
    cv::FileStorage fs(config_file.c_str(), cv::FileStorage::READ);

    if(!fs.isOpened())
    {
    	std::cout << "Could not open calibration_params.yml" << std::endl;
    	return 0;
    }

	// Grab configuration parameters
    fs["camera_matrix"] >> camera_matrix;
    fs["distortion_coefficients"] >> dist_coeffs;

    fs.release();

    std::cout << "camera_matrix\n" << camera_matrix << std::endl;
    std::cout << "\ndist coeffs\n" << dist_coeffs << std::endl;

    std::ostringstream vector_to_marker;
    std::string portFileName = "/dev/ttyACM0";

    std::cout << "camera Usb conectada ? : s/n \n" << std::endl;
    std::cin >> teste;

    //Set Usb camera ID
    if(teste == 's' || teste == 'S') camId = 1;

    in_video.open(camId);
    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);

    while (in_video.grab())
    {

        in_video.retrieve(image);
        image.copyTo(image_copy);
        std::vector<int> ids;
    	std::vector<std::vector<cv::Point2f> > corners;
        cv::aruco::detectMarkers(image, dictionary, corners, ids);

        cv::namedWindow("Detected Markers");

        // if at least one marker detected
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
                                 << "x: " << std::setw(8)<<  tvecs[0](0) << "//" << rvecs[0](0);
                cv::putText(image_copy, vector_to_marker.str(),
                        cvPoint(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cvScalar(0, 252, 124), 1, CV_AA);

                vector_to_marker.str(std::string());
                vector_to_marker << std::setprecision(4)
                                 << "y: " << std::setw(8) << tvecs[0](1) << "//" << rvecs[0](1);
                cv::putText(image_copy, vector_to_marker.str(),
                        cvPoint(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cvScalar(0, 252, 124), 1, CV_AA);

                vector_to_marker.str(std::string());
                vector_to_marker << std::setprecision(4)
                                 << "z: " << std::setw(8) << tvecs[0](2) << "//" << rvecs[0](2);
                cv::putText(image_copy, vector_to_marker.str(),
                        cvPoint(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cvScalar(0, 252, 124), 1, CV_AA);

				//Set and publish aruco marker attributes and publishes it in the topic
				msg.ID = ids[i];
				msg.pose.x = tvecs[0](0);
				msg.pose.y = tvecs[0](1);
				msg.pose.z = tvecs[0](2);

                //Todo, fix it to make it convert in the right form
                msg.orientation.x = rvecs[0](0);
                msg.orientation.y = rvecs[0](1);
                msg.orientation.z = rvecs[0](2);

				aruco_pose_pub.publish(msg);

            }

            cv::imshow("Detected Markers", image_copy);

        }
        cv::imshow("Detected Markers", image_copy);
        ids.clear();

        char key = (char) cv::waitKey(wait_time);
        if (key == 27)
            break;
    }

    if(file != NULL)
    	fclose(file);

    in_video.release();
}
