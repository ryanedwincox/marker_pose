#ifndef BARCODE_H
#define BARCODE_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

#define NUM_BARCODES 2

class Barcode
{
public:
    Barcode();
    cv::Mat projectAxis(cv::Mat img, cv::Mat rvec, cv::Mat tvec);
    cv::Mat projectBarcodeGrid(cv::Mat img, cv::Mat rvec, cv::Mat tvec);
    void projectSamplePoints(cv::Mat rvec, cv::Mat tvec);
    void setCameraParmeters(cv::Mat cameraMatrix, cv::Mat distCoeffs, int w, int h);
    int getMarkerNumber(cv::Mat imgBin);
    int getSectionValue(cv::Mat img, cv::Point2f samplePoint, int w, int h);

private:
    int w;
    int h;
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
