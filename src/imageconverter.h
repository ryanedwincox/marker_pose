#ifndef IMAGECONVERTER_H
#define IMAGECONVERTER_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
public:
    ImageConverter();
    ~ImageConverter();
    void subscribe();
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat getImage();
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    cv_bridge::CvImagePtr cv_ptr;
};

#endif // IMAGECONVERTER_H
