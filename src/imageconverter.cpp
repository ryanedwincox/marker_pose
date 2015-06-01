#include "imageconverter.h"

ImageConverter::ImageConverter()
: it_(nh_)
{

    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/marker_pose/output_video", 1);

//    cameraSubscriber = it_Ptr->subscribeCamera("/usb_cam/image_raw",3,&ImageConverter::imageCb,this,image_transport::TransportHints("compressed"));
}

//void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr &ci)
//{
//    // get image
//    try
//    {
//      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//    }
//    catch (cv_bridge::Exception& e)
//    {
//      ROS_ERROR("cv_bridge exception: %s", e.what());
//      return;
//    }

////    // get camera properties
////    cout << "camera distortion: " << ci->D.data() << endl;
////    cout << "camera matrix: " << ci->K.data() << endl;

////    // Output modified video stream
////    image_pub_.publish(cv_ptr->toImageMsg());
//}

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

