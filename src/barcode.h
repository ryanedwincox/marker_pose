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
    bool zDirection(cv::Mat rvec);
    void setCameraParmeters(cv::Mat cameraMatrix, cv::Mat distCoeffs, int w, int h);
    int getMarkerNumber(cv::Mat imgBin);
    int getSectionValue(cv::Mat img, cv::Point2f samplePoint, int w, int h);
    void rotateOrigin(int num, cv::Mat* rvec, cv::Mat* tvec);
    int getImageWidth();
    int getImageHeight();

    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    cv::Mat barcodeGrid;

private:
    int w;
    int h;
    int markerNumber;

    cv::Mat samplePoints;
    std::vector<cv::Point2f> projectedSamplePoints;

    int barcodes [NUM_BARCODES][9];
};

#endif // BARCODE_H
