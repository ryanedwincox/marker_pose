#ifndef IMAGECONVERTER_H
#define IMAGECONVERTER_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

class ImageConverter
{
public:
    ImageConverter();
    ~ImageConverter();
//    void imageCb(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr &ci);
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat getImage();
    void publishImage(cv::Mat img);
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

//    image_transport::CameraSubscriber cameraSubscriber;
//    image_transport::ImageTransport* it_Ptr;

    cv_bridge::CvImagePtr cv_ptr;
    cv_bridge::CvImagePtr cv_ptr_output;
};

#endif // IMAGECONVERTER_H
