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

    barcodes[1][0] = 0; barcodes[1][1] = 1; barcodes[1][2] = 0;
    barcodes[1][3] = 0; barcodes[1][4] = 0; barcodes[1][5] = 0;
    barcodes[1][6] = 0; barcodes[1][7] = 0; barcodes[1][8] = 0;

    barcodes[2][0] = 0; barcodes[2][1] = 0; barcodes[2][2] = 0;
    barcodes[2][3] = 0; barcodes[2][4] = 0; barcodes[2][5] = 1;
    barcodes[2][6] = 0; barcodes[2][7] = 0; barcodes[2][8] = 0;

    barcodes[3][0] = 0; barcodes[3][1] = 0; barcodes[3][2] = 0;
    barcodes[3][3] = 0; barcodes[3][4] = 0; barcodes[3][5] = 0;
    barcodes[3][6] = 0; barcodes[3][7] = 1; barcodes[3][8] = 0;

    barcodes[4][0] = 0; barcodes[4][1] = 0; barcodes[4][2] = 0;
    barcodes[4][3] = 1; barcodes[4][4] = 0; barcodes[4][5] = 0;
    barcodes[4][6] = 0; barcodes[4][7] = 1; barcodes[4][8] = 0;

    barcodes[5][0] = 0; barcodes[5][1] = 1; barcodes[5][2] = 0;
    barcodes[5][3] = 1; barcodes[5][4] = 0; barcodes[5][5] = 0;
    barcodes[5][6] = 0; barcodes[5][7] = 0; barcodes[5][8] = 0;

    barcodes[6][0] = 0; barcodes[6][1] = 1; barcodes[6][2] = 0;
    barcodes[6][3] = 0; barcodes[6][4] = 0; barcodes[6][5] = 1;
    barcodes[6][6] = 0; barcodes[6][7] = 0; barcodes[6][8] = 0;

    barcodes[7][0] = 0; barcodes[7][1] = 0; barcodes[7][2] = 0;
    barcodes[7][3] = 0; barcodes[7][4] = 0; barcodes[7][5] = 1;
    barcodes[7][6] = 0; barcodes[7][7] = 1; barcodes[7][8] = 0;
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
    barcodeGrid.at<cv::Point3f>(0) = (cv::Point3f){-0.031,-0.031,0};
    barcodeGrid.at<cv::Point3f>(1) = (cv::Point3f){-0.031, 0.031,0};
    barcodeGrid.at<cv::Point3f>(2) = (cv::Point3f){ 0.031,-0.031,0};
    barcodeGrid.at<cv::Point3f>(3) = (cv::Point3f){ 0.031, 0.031,0};

    cv::projectPoints(barcodeGrid, rvec, tvec, cameraMatrix, distCoeffs, projectedGrid);

    cv::line(img, projectedGrid[0], projectedGrid[1], cv::Scalar(0,0,255), 2);
    cv::line(img, projectedGrid[1], projectedGrid[3], cv::Scalar(0,255,0), 2);
    cv::line(img, projectedGrid[2], projectedGrid[0], cv::Scalar(255,0,0), 2);
    cv::line(img, projectedGrid[3], projectedGrid[2], cv::Scalar(255,255,0), 2);

    // TODO print marker number on image
//    char *intStr = std::itoa(markerNumber);
//    string str = string(intStr);
//    cv::putText(img, str, projectedGrid[0], cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,255), 2, 1);

    return img;
}

void Barcode::projectSamplePoints(cv::Mat rvec, cv::Mat tvec)
{
    float d = 0.02;
    samplePoints.at<cv::Point3f>(0) = (cv::Point3f){-d,-d,0};
    samplePoints.at<cv::Point3f>(1) = (cv::Point3f){ 0,-d,0};
    samplePoints.at<cv::Point3f>(2) = (cv::Point3f){ d,-d,0};
    samplePoints.at<cv::Point3f>(3) = (cv::Point3f){-d, 0,0};
    samplePoints.at<cv::Point3f>(4) = (cv::Point3f){ 0, 0,0};
    samplePoints.at<cv::Point3f>(5) = (cv::Point3f){ d, 0,0};
    samplePoints.at<cv::Point3f>(6) = (cv::Point3f){-d, d,0};
    samplePoints.at<cv::Point3f>(7) = (cv::Point3f){ 0, d,0};
    samplePoints.at<cv::Point3f>(8) = (cv::Point3f){ d, d,0};

    cv::projectPoints(samplePoints, rvec, tvec, cameraMatrix, distCoeffs, projectedSamplePoints);
}

// return true if z axis of marker transform is pointed towards the camera
bool Barcode::zDirection(cv::Mat rvec)
{
    cv::Mat R(3,3,cv::DataType<double>::type);
    cv::Mat Z(3,1,cv::DataType<double>::type);
    cv::Mat zVec(3,1,cv::DataType<double>::type);

    Z.at<double>(0) = 0;
    Z.at<double>(1) = 0;
    Z.at<double>(2) = 1;

    cv::Rodrigues(rvec, R);
    zVec = R*Z;

//    std::cout << zVec << std::endl;

    return zVec.at<double>(2) <= 0;
}

int Barcode::getMarkerNumber(cv::Mat imgBin)
{
    // Get barcode value
    int numSamples = 9; // for 3x3 grid
    int barcodeInput [numSamples];

//    std::cout << "desired barcode: ";
//    for (int i = 0; i < numSamples; i++)
//    {
//        std::cout << barcodes[1][i];
//    }
//    std::cout << std::endl;

//    std::cout << "barcode input: ";
    for (int i = 0; i < numSamples; i++)
    {
        barcodeInput[i] = getSectionValue(imgBin, projectedSamplePoints[i], w, h);
//        std::cout << barcodeInput[i];
    }

    for (int j = 0; j < NUM_BARCODES; j++)
    {
        bool match = true;
        for (int i = 0; i < numSamples; i++)
        {
            if (barcodeInput[i] != barcodes[j][i])
            {
                match = false;
            }
        }
        if (match)
        {
//            std::cout << " match #: " << j << std::endl;
            markerNumber = j;
            return j;
        }
    }
//    std::cout << std::endl;
    return -1;

}

void Barcode::rotateOrigin(int num, cv::Mat* rvec, cv::Mat* tvec)
{
    for (int i = 0; i < num; i++)
    {
        // Use barcode number to correct origin
        cv::Mat rMat(3,3,cv::DataType<double>::type);
        cv::Mat affineRotation(3,3,cv::DataType<double>::type);
        cv::Mat translate(3,1,cv::DataType<double>::type);
        double theta = -PI/2; // radians
        affineRotation.at<double>(0,0) = cos(theta); affineRotation.at<double>(0,1) = sin(theta); affineRotation.at<double>(0,2) = 0;
        affineRotation.at<double>(1,0) = -sin(theta); affineRotation.at<double>(1,1) = cos(theta); affineRotation.at<double>(1,2) = 0;
        affineRotation.at<double>(2,0) = 0; affineRotation.at<double>(2,1) = 0; affineRotation.at<double>(2,2) = 1;


        translate.at<double>(0) = 0;
        translate.at<double>(1) = 0;
        translate.at<double>(2) = 0;

        // Apply affine transformation
        cv::Rodrigues(*rvec, rMat);
        *tvec = *tvec+rMat*translate;
        rMat = rMat*affineRotation;
        cv::Rodrigues(rMat, *rvec);
    }
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
