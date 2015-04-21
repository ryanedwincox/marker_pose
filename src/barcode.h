#ifndef BARCODE_H
#define BARCODE_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <string>
#include "math.h"

#include "markerlayout.h"

#define PI 3.1415926
#define NUM_BARCODES 8

class Barcode
{
public:
    Barcode();
    cv::Mat projectAxis(cv::Mat img, cv::Mat rvec, cv::Mat tvec, MarkerLayout marker);
    cv::Mat projectBarcodeGrid(cv::Mat img, cv::Mat rvec, cv::Mat tvec);
    void projectSamplePoints(cv::Mat rvec, cv::Mat tvec);
    bool zDirection(cv::Mat rvec);
    void setCameraParmeters(cv::Mat cameraMatrix, cv::Mat distCoeffs, int w, int h);
    int getMarkerNumber(cv::Mat imgBin);
    int getSectionValue(cv::Mat img, cv::Point2f samplePoint, int w, int h);
    void rotateOrigin(int num, cv::Mat* rvec, cv::Mat* tvec);

private:
    int w;
    int h;
    int markerNumber;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    cv::Mat axis;
    cv::Mat barcodeGrid;
    cv::Mat samplePoints;

    std::vector<cv::Point2f> projectedAxis;
    std::vector<cv::Point2f> projectedGrid;
    std::vector<cv::Point2f> projectedSamplePoints;

    int barcodes [NUM_BARCODES][9];
};

#endif // BARCODE_H
