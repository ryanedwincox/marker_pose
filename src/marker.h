#ifndef Marker_H
#define Marker_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <list>
#include <vector>
#include <queue>

#include "holdpoint.h"
#include "barcode.h"

class Marker
{
public:
    Marker();
    void setImageCoord(std::vector<HoldPoint> H);
    cv::Mat getWorldCoord();
    cv::Mat getImageCoord(int orientation);
    std::vector<HoldPoint> sortPointsVertically(std::vector<HoldPoint> H);
    void averageVec();
    void setWorldTransform(cv::Mat worldTransform);
    cv::Mat getWorldTransform();
    void poseEstimation(cv::Mat imgBin, int w, int h, Barcode barcode);
    bool markerTransformationZero();
    cv::Mat projectAxis(cv::Mat img, Barcode barcode);
    cv::Mat projectBarcodeGrid(cv::Mat img, Barcode barcode);
    cv::Mat projectTransformAxis(cv::Mat img, Barcode barcode, cv::Mat newWorldTransform);
    void setWorldCoord();
    void rotateWorldCoord(int rot);

    cv::Mat rvec;
    cv::Mat tvec;
    bool enoughMarkers;
    int foundMarkers;
    int imgCoordOrientation;


private:
    // sorting methods
    std::list<HoldPoint> mergesort(std::list<HoldPoint> H);
    std::list<HoldPoint> merge(std::list<HoldPoint> left, std::list<HoldPoint> right);

    // Variables
    int averageingWindow;
    int numMarkers;
    int rotNum;
    int markerID;
    float targetSpacing;
    cv::Mat worldTransform;
    cv::Mat worldCoord;
    cv::Mat imageCoord;
    cv::Mat imageCoord0;
    cv::Mat imageCoord1;
    cv::Mat axis;
    std::vector<cv::Point2f> projectedAxis;
    std::vector<cv::Point2f> projectedGrid;
    std::vector<cv::Mat> imageCoordVec;

};

#endif // Marker_H
