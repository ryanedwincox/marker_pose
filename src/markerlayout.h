#ifndef MARKERLAYOUT_H
#define MARKERLAYOUT_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <list>
#include <vector>

#include "holdpoint.h"

class MarkerLayout
{
public:
    MarkerLayout();
    void setImageCoord(std::vector<HoldPoint> H);
    cv::Mat getWorldCoord();
    cv::Mat getImageCoord(int orientation);
    std::vector<HoldPoint> sortPointsVertically(std::vector<HoldPoint> H);

    bool enoughMarkers;

private:
    // sorting methods
    std::list<HoldPoint> mergesort(std::list<HoldPoint> H);
    std::list<HoldPoint> merge(std::list<HoldPoint> left, std::list<HoldPoint> right);

    // Variables
    int numMarkers;
    cv::Mat worldCoord;
    cv::Mat imageCoord;
    std::vector<cv::Mat> imageCoordVec;

};

#endif // MARKERLAYOUT_H
