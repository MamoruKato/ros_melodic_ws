#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

class ImageConverter
{
public:
  ImageConverter();
  ~ImageConverter();
  void image_convert_callback(const sensor_msgs::Image::ConstPtr& msg);
  void camera_info_callback(const sensor_msgs::CameraInfo::ConstPtr& msg);

private:
  ros::NodeHandle _nh;
  ros::Publisher _image_pub_;
  ros::Publisher _cameraInfoPub;
  ros::Subscriber _imageConverter;
  ros::Subscriber _cameraInfoSub;
};

ImageConverter::ImageConverter()
{
  _image_pub_ = _nh.advertise<sensor_msgs::Image>("/converted/image_rect",100);
  _imageConverter = _nh.subscribe<sensor_msgs::Image>("/my_robot/camera1/image_rect",
                                  100, &ImageConverter::image_convert_callback, this);

  _cameraInfoPub = _nh.advertise<sensor_msgs::CameraInfo>("/converted/camera_info",100);
  _cameraInfoSub = _nh.subscribe<sensor_msgs::CameraInfo>("/my_robot/camera1/camera_info",100,
                                  &ImageConverter::camera_info_callback, this);

  return;

}

ImageConverter::~ImageConverter()
{
  return;
}

void ImageConverter::image_convert_callback(const sensor_msgs::Image::ConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
  ROS_INFO("Converted Image");
  _image_pub_.publish(cv_ptr->toImageMsg());

}

void ImageConverter::camera_info_callback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{

  _cameraInfoPub.publish(msg);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ig;
  ros::spin();
  return 0;
}
