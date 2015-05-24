#include "imageconverter.h"

ImageConverter::ImageConverter()
: it_(nh_)
{

    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/marker_pose/output_video", 1);
}

void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

//    // Output modified video stream
//    image_pub_.publish(cv_ptr->toImageMsg());
}

cv::Mat ImageConverter::getImage()
{
    return cv_ptr->image;
}

// TODO publish processed image
void ImageConverter::publishImage(cv::Mat img)
{
    cv_ptr_output->image = img;
//    image_pub_.publish(cv_ptr_output->toImageMsg());
}

ImageConverter::~ImageConverter()
{
}

