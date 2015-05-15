#ifndef KALMAN_H
#define KALMAN_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/imgproc/imgproc.hpp>
//#include "/opt/ros/groovy/include/opencv2/video/tracking.hpp"

#include <iostream>
#include <list>

// ******************************
//
// Currently not used
//
// *****************************

class Kalman
{
public:
    Kalman();
    cv::KalmanFilter createKalmanFilter(int x, int y);
    cv::Point runKalmanFilter(cv::KalmanFilter KF, cv::Point statePt, std::list<cv::Point> avgMatches);
};

#endif // KALMAN_H
