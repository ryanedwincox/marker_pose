#include "barcode.h"

Barcode::Barcode()
{
    cameraMatrix = cv::Mat(3,3,cv::DataType<double>::type);
    distCoeffs = cv::Mat(5,1,cv::DataType<double>::type);
    axis = cv::Mat(4,1,cv::DataType<cv::Point3f>::type);
    barcodeGrid = cv::Mat(4,1,cv::DataType<cv::Point3f>::type);
    samplePoints = cv::Mat(9,1,cv::DataType<cv::Point3f>::type);

    projectedAxis = std::vector<cv::Point2f>();
    projectedGrid = std::vector<cv::Point2f>();
    projectedSamplePoints = std::vector<cv::Point2f>();

    // define barcode patterns
    barcodes[0][0] = 0; barcodes[0][1] = 0; barcodes[0][2] = 0;
    barcodes[0][3] = 1; barcodes[0][4] = 0; barcodes[0][5] = 0;
    barcodes[0][6] = 0; barcodes[0][7] = 0; barcodes[0][8] = 0;

//    barcodes[1][0] = 0; barcodes[1][1] = 1; barcodes[1][2] = 0;
//    barcodes[1][3] = 0; barcodes[1][4] = 0; barcodes[1][5] = 0;
//    barcodes[1][6] = 0; barcodes[1][7] = 0; barcodes[1][8] = 0;

//    barcodes[2][0] = 0; barcodes[2][1] = 0; barcodes[2][2] = 0;
//    barcodes[2][3] = 0; barcodes[2][4] = 0; barcodes[2][5] = 1;
//    barcodes[2][6] = 0; barcodes[2][7] = 0; barcodes[2][8] = 0;

//    barcodes[3][0] = 0; barcodes[3][1] = 0; barcodes[3][2] = 0;
//    barcodes[3][3] = 0; barcodes[3][4] = 0; barcodes[3][5] = 0;
//    barcodes[3][6] = 0; barcodes[3][7] = 1; barcodes[3][8] = 0;
}

void Barcode::setCameraParmeters(cv::Mat cameraMatrix, cv::Mat distCoeffs, int w, int h)
{
    this->cameraMatrix = cameraMatrix;
    this->distCoeffs = distCoeffs;
    this->w = w;
    this->h = h;
}

cv::Mat Barcode::projectAxis(cv::Mat img, cv::Mat rvec, cv::Mat tvec)
{
    // project axis
    axis.at<cv::Point3f>(0) = (cv::Point3f){0,0,0};
    axis.at<cv::Point3f>(1) = (cv::Point3f){0.1,0,0};
    axis.at<cv::Point3f>(2) = (cv::Point3f){0,0.1,0};
    axis.at<cv::Point3f>(3) = (cv::Point3f){0,0,0.1};

    cv::projectPoints(axis, rvec, tvec, cameraMatrix, distCoeffs, projectedAxis);

    cv::line(img, projectedAxis[0], projectedAxis[1], cv::Scalar(0,0,255), 2);
    cv::line(img, projectedAxis[0], projectedAxis[2], cv::Scalar(0,255,0), 2);
    cv::line(img, projectedAxis[0], projectedAxis[3], cv::Scalar(255,0,0), 2);

    return img;
}

cv::Mat Barcode::projectBarcodeGrid(cv::Mat img, cv::Mat rvec, cv::Mat tvec)
{
    // Project barcode layout
    barcodeGrid.at<cv::Point3f>(0) = (cv::Point3f){0.04,0.09,0};
    barcodeGrid.at<cv::Point3f>(1) = (cv::Point3f){0.16,0.09,0};
    barcodeGrid.at<cv::Point3f>(2) = (cv::Point3f){0.04,0.21,0};
    barcodeGrid.at<cv::Point3f>(3) = (cv::Point3f){0.16,0.21,0};

    cv::projectPoints(barcodeGrid, rvec, tvec, cameraMatrix, distCoeffs, projectedGrid);

    cv::line(img, projectedGrid[0], projectedGrid[1], cv::Scalar(0,0,255), 2);
    cv::line(img, projectedGrid[1], projectedGrid[3], cv::Scalar(0,255,0), 2);
    cv::line(img, projectedGrid[2], projectedGrid[0], cv::Scalar(255,0,0), 2);
    cv::line(img, projectedGrid[3], projectedGrid[2], cv::Scalar(255,255,0), 2);

    return img;
}

void Barcode::projectSamplePoints(cv::Mat rvec, cv::Mat tvec)
{
    samplePoints.at<cv::Point3f>(0) = (cv::Point3f){0.06,0.11,0};
    samplePoints.at<cv::Point3f>(1) = (cv::Point3f){0.10,0.11,0};
    samplePoints.at<cv::Point3f>(2) = (cv::Point3f){0.14,0.11,0};
    samplePoints.at<cv::Point3f>(3) = (cv::Point3f){0.06,0.15,0};
    samplePoints.at<cv::Point3f>(4) = (cv::Point3f){0.10,0.15,0};
    samplePoints.at<cv::Point3f>(5) = (cv::Point3f){0.14,0.15,0};
    samplePoints.at<cv::Point3f>(6) = (cv::Point3f){0.06,0.19,0};
    samplePoints.at<cv::Point3f>(7) = (cv::Point3f){0.10,0.19,0};
    samplePoints.at<cv::Point3f>(8) = (cv::Point3f){0.14,0.19,0};

    cv::projectPoints(samplePoints, rvec, tvec, cameraMatrix, distCoeffs, projectedSamplePoints);
}

int Barcode::getMarkerNumber(cv::Mat imgBin)
{
    // Get barcode value
    int barcodeInput [9];
    for (int i = 0; i < 8; i++)
    {
        barcodeInput[i] = getSectionValue(imgBin, projectedSamplePoints[i], w, h);
//        std::cout << barcodeInput[i] << std::endl;
    }

    for (int j = 0; j < NUM_BARCODES; j++)
    {
    int j = 0;
        bool match = true;
        for (int i = 0; i < 8; i++)
        {
            if (barcodeInput[i] != barcodes[j][i])
            {
                match = false;
            }
        }
        if (match)
        {
            return j;
        }
    }
    return -1;

}

// TODO sample and average several points
int Barcode::getSectionValue(cv::Mat imgBin, cv::Point2f samplePoint, int w, int h)
{
    if (samplePoint.x >= 0 && samplePoint.y >= 0 && samplePoint.x <= w && samplePoint.y <= h)
    {
        return (int)imgBin.at<uchar>(samplePoint.y,samplePoint.x) / 255;
    }
    else
    {
        return -1;
    }
}
