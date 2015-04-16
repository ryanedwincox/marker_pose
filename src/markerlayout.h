#ifndef MARKERLAYOUT_H
#define MARKERLAYOUT_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <list>
#include <vector>
#include <queue>

#include "holdpoint.h"

class MarkerLayout
{
public:
    MarkerLayout();
    void setImageCoord(std::vector<HoldPoint> H);
    cv::Mat getWorldCoord();
    cv::Mat getImageCoord(int orientation);
    std::vector<HoldPoint> sortPointsVertically(std::vector<HoldPoint> H);
    void averageVec (cv::Mat rvec, cv::Mat tvec);

    bool enoughMarkers;
    int numMarkers;

private:
    // sorting methods
    std::list<HoldPoint> mergesort(std::list<HoldPoint> H);
    std::list<HoldPoint> merge(std::list<HoldPoint> left, std::list<HoldPoint> right);

    // Variables
    int averageingWindow;
    cv::Mat worldCoord;
    cv::Mat imageCoord;
    cv::Mat imageCoord0;
    cv::Mat imageCoord1;
    cv::Mat imageCoord2;
    cv::Mat imageCoord3;
    cv::Mat imageCoord4;
    cv::Mat imageCoord5;
    cv::Mat imageCoord6;
    cv::Mat imageCoord7;
    std::vector<cv::Mat> imageCoordVec;

};

#endif // MARKERLAYOUT_H
