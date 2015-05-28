#ifndef BARCODE_H
#define BARCODE_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <string>
#include "math.h"

#define PI 3.1415926
#define NUM_BARCODES 8

class Barcode
{
public:
    Barcode();


    void projectSamplePoints(cv::Mat rvec, cv::Mat tvec);
    void projectSampleRegions(cv::Mat rvec, cv::Mat tvec);
    bool zDirection(cv::Mat rvec);
    void setCameraParmeters(cv::Mat cameraMatrix, cv::Mat distCoeffs, int w, int h);
    int getMarkerNumber(cv::Mat imgBin);
    int getRegionValue(cv::Mat img, cv::Point2f samplePoint);
    int getAveragedRegionValue(cv::Mat img, cv::Mat debug, cv::Point2f TL, cv::Point2f TR, cv::Point2f BL, cv::Point2f BR);
    void rotateOrigin(int num, cv::Mat* rvec, cv::Mat* tvec);
    int getImageWidth();
    int getImageHeight();
    bool pointInFrame(cv::Point2f point);

    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    cv::Mat barcodeGrid;
    float barcodeGridWidth;

private:
    int w;
    int h;
    int markerNumber;

    cv::Mat samplePoints;
    cv::Mat sampleRegions;
    std::vector<cv::Point2f> projectedSamplePoints;
    std::vector<cv::Point2f> projectedSampleRegions;

    int barcodes [NUM_BARCODES][9];
};

#endif // BARCODE_H
