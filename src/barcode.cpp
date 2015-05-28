#include "barcode.h"

Barcode::Barcode()
{
    cameraMatrix = cv::Mat(3,3,cv::DataType<double>::type);
    distCoeffs = cv::Mat(5,1,cv::DataType<double>::type);

    barcodeGrid = cv::Mat(4,1,cv::DataType<cv::Point3f>::type);
    samplePoints = cv::Mat(9,1,cv::DataType<cv::Point3f>::type);
    sampleRegions = cv::Mat(36,1,cv::DataType<cv::Point3f>::type);

    barcodeGridWidth = 0.062;

    projectedSamplePoints = std::vector<cv::Point2f>();
    projectedSampleRegions = std::vector<cv::Point2f>();

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

void Barcode::projectSamplePoints(cv::Mat rvec, cv::Mat tvec)
{
    float d = barcodeGridWidth/3; // the width of 1 box
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

void Barcode::projectSampleRegions(cv::Mat rvec, cv::Mat tvec)
{
    float r = barcodeGridWidth/10; // barcodeGridWidth/6 is the radius of one box but probably want slightly smaller than that
    for (int i = 0; i < 9; i++)
    {
        cv::Point3f samplePoint = samplePoints.at<cv::Point3f>(i);
        sampleRegions.at<cv::Point3f>(4*i+0) = (cv::Point3f){samplePoint.x-r,samplePoint.y-r,0};
        sampleRegions.at<cv::Point3f>(4*i+1) = (cv::Point3f){samplePoint.x+r,samplePoint.y-r,0};
        sampleRegions.at<cv::Point3f>(4*i+2) = (cv::Point3f){samplePoint.x-r,samplePoint.y+r,0};
        sampleRegions.at<cv::Point3f>(4*i+3) = (cv::Point3f){samplePoint.x+r,samplePoint.y+r,0};
    }

    cv::projectPoints(sampleRegions, rvec, tvec, cameraMatrix, distCoeffs, projectedSampleRegions);
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

    cv::Mat debug(480,640,cv::DataType<uchar>::type,cv::Scalar(0,0,0));

//    std::cout << "barcode input: ";
    for (int i = 0; i < numSamples; i++)
    {
//        barcodeInput[i] = getRegionValue(imgBin, projectedSamplePoints[i]);
        barcodeInput[i] = getAveragedRegionValue(imgBin, debug, projectedSampleRegions[4*i+0], projectedSampleRegions[4*i+1], projectedSampleRegions[4*i+2], projectedSampleRegions[4*i+3]);
//        std::cout << barcodeInput[i];
    }

//    cv::imshow("Debug Window", debug);

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
        affineRotation.at<double>(0,0) = cos(theta);  affineRotation.at<double>(0,1) = sin(theta); affineRotation.at<double>(0,2) = 0;
        affineRotation.at<double>(1,0) = -sin(theta); affineRotation.at<double>(1,1) = cos(theta); affineRotation.at<double>(1,2) = 0;
        affineRotation.at<double>(2,0) = 0;           affineRotation.at<double>(2,1) = 0;          affineRotation.at<double>(2,2) = 1;

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

// Returns value of center point
int Barcode::getRegionValue(cv::Mat imgBin, cv::Point2f point)
{
    if (pointInFrame(point))
//    if (point.x >= 0 && point.y >= 0 && point.x <= w && point.y <= h)
    {
        return (int)imgBin.at<uchar>(point.y,point.x) / 255;
    }
    else
    {
        return -1;
    }
}

// Returns the averaged value of barcode region looking at the average along the two diagonals
// parameters TopLeft, TopRight, BottomLeft, BottomRight
int Barcode::getAveragedRegionValue(cv::Mat imgBin, cv::Mat debug, cv::Point2f TL, cv::Point2f TR, cv::Point2f BL, cv::Point2f BR)
{
    if (pointInFrame(TL) && pointInFrame(TR) && pointInFrame(BL) && pointInFrame(BR))
    {
//        cv::circle(imgBin,TL,1,cv::Scalar(255,255,255),-1);
//        cv::circle(imgBin,TR,1,cv::Scalar(255,255,255),-1);
//        cv::circle(imgBin,BL,1,cv::Scalar(255,255,255),-1);
//        cv::circle(imgBin,BR,1,cv::Scalar(255,255,255),-1);
        double m = (TR.y-BL.y)/(TR.x-BL.x);
        double b = TR.y - m*TR.x;
//        double sum = 0;
//        double count = 0;
        int countHigh = 0;
        int countLow = 0;
        for (int x = BL.x; x < TR.x; x++)
        {
            int y = m*x+b;
            if ((y < TR.y && y > BL.y) || (y > TR.y && y < BL.y))
            {
    //                cv::circle(imgBin,(cv::Point2i){x,y},1,cv::Scalar(255,255,255),-1);
//                    cv::circle(debug,(cv::Point2i){x,y},1,cv::Scalar(255,255,255),-1);
    //                sum += (double)imgBin.at<uchar>(y,x);
    //                count++;
                if ((int)imgBin.at<uchar>(y,x) / 255 == 1)
                {
                    countHigh++;
                }
                else
                {
                    countLow++;
                }
            }
        }

        double m1 = (TL.y-BR.y)/(TL.x-BR.x);
        double b1 = TL.y - m1*TL.x;
        for (int x = TL.x; x < BR.x; x++)
        {
            int y = m1*x+b1;
            if ((y < TL.y && y > BR.y) || (y > TL.y && y < BR.y))
            {
//                cv::circle(imgBin,(cv::Point2i){x,y},1,cv::Scalar(255,255,255),-1);
//                cv::circle(debug,(cv::Point2i){x,y},1,cv::Scalar(255,255,255),-1);
                if ((int)imgBin.at<uchar>(y,x) / 255 == 1)
                {
                    countHigh++;
                }
                else
                {
                    countLow++;
                }
            }
        }

        if (countHigh > countLow)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
    else
    {
        return -1;
    }
}

bool Barcode::pointInFrame(cv::Point2f point)
{
    return point.x >= 0 && point.y >= 0 && point.x <= w && point.y <= h;
}

int Barcode::getImageWidth()
{
    return w;
}

int Barcode::getImageHeight()
{
    return h;
}
