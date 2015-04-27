#include "marker.h"

Marker::Marker()
{
    numMarkers = 4;
    targetSpacing = 0.105; // in meters

    // Define rvec tvec
    rvec = cv::Mat(3,1,cv::DataType<double>::type);
    tvec = cv::Mat(3,1,cv::DataType<double>::type);

    axis = cv::Mat(4,1,cv::DataType<cv::Point3f>::type);
    projectedAxis = std::vector<cv::Point2f>();
    projectedGrid = std::vector<cv::Point2f>();

    // Define 3D coordinates of markers
    worldCoord = cv::Mat(numMarkers,1,cv::DataType<cv::Point3f>::type);
    // rectangle marker pattern


    // Define 2D corrdinates of markers detected in image
    imageCoord = cv::Mat(numMarkers,1,cv::DataType<cv::Point2f>::type);
    imageCoord0 = cv::Mat(numMarkers,1,cv::DataType<cv::Point2f>::type);
    imageCoord1 = cv::Mat(numMarkers,1,cv::DataType<cv::Point2f>::type);
//    imageCoordVec = std::vector<cv::Mat>(8, imageCoord);

    enoughMarkers = false;
    averageingWindow = 5;
}

void Marker::setWorldCoord()
{
    worldCoord.at<cv::Point3f>(0) = (cv::Point3f){-targetSpacing/2,-targetSpacing/2,0};
    worldCoord.at<cv::Point3f>(1) = (cv::Point3f){ targetSpacing/2,-targetSpacing/2,0};
    worldCoord.at<cv::Point3f>(2) = (cv::Point3f){-targetSpacing/2, targetSpacing/2,0};
    worldCoord.at<cv::Point3f>(3) = (cv::Point3f){ targetSpacing/2, targetSpacing/2,0};
}

void Marker::rotateWorldCoord(int rot)
{
    for (int j = 0; j < rot; j++)
    {
        for (int i = 0; i < 4; i++)
        {
            cv::Mat worldPoint = cv::Mat(3,1,cv::DataType<double>::type);
            worldPoint.at<double>(0) = (double)worldCoord.at<cv::Point3f>(i).x;
            worldPoint.at<double>(1) = (double)worldCoord.at<cv::Point3f>(i).y;
            worldPoint.at<double>(2) = (double)worldCoord.at<cv::Point3f>(i).z;
    //        worldPoint.at<double>(3) = 1;

            cv::Mat affineRotation(3,3,cv::DataType<double>::type);
            double theta = PI/2; // radians
            affineRotation.at<double>(0,0) = cos(theta);  affineRotation.at<double>(0,1) = sin(theta); affineRotation.at<double>(0,2) = 0;
            affineRotation.at<double>(1,0) = -sin(theta); affineRotation.at<double>(1,1) = cos(theta); affineRotation.at<double>(1,2) = 0;
            affineRotation.at<double>(2,0) = 0;           affineRotation.at<double>(2,1) = 0;          affineRotation.at<double>(2,2) = 1;

    //                    std::cout << "WorldPoint: " << worldPoint << std::endl;
    //                    std::cout << "transform: " << marker2.getWorldTransform() << std::endl;

            worldPoint = affineRotation * worldPoint;
            std::cout << "worldPoint: " << worldPoint << std::endl;

            worldCoord.at<cv::Point3f>(i).x = (float)worldPoint.at<double>(0);
            worldCoord.at<cv::Point3f>(i).y = (float)worldPoint.at<double>(1);
            worldCoord.at<cv::Point3f>(i).z = (float)worldPoint.at<double>(2);
        }
    }

//    for (int i = 0; i < rot; i++)
//    {
//        cv::Point3f temp = worldCoord.at<cv::Point3f>(0);
//        worldCoord.at<cv::Point3f>(0) = worldCoord.at<cv::Point3f>(1);
//        worldCoord.at<cv::Point3f>(1) = worldCoord.at<cv::Point3f>(2);
//        worldCoord.at<cv::Point3f>(2) = worldCoord.at<cv::Point3f>(3);
//        worldCoord.at<cv::Point3f>(3) = temp;
//    }
}

// takes a vertically sorted vector of HoldPoints and puts them in imageCoordVec in every possible order
void Marker::setImageCoord(std::vector<HoldPoint> H)
{
//    std::cout << H.size() << std::endl;
    if (foundMarkers >= numMarkers)
    {
        enoughMarkers = true;

        // fill in all imageCoordVec permutations.  There should be 8 total
        // TODO there could actually be more if you're looking in from the side closely
        imageCoord0.at<cv::Point2f>(0) = H.at(0).heldMatch;
        imageCoord0.at<cv::Point2f>(1) = H.at(1).heldMatch;
        imageCoord0.at<cv::Point2f>(2) = H.at(2).heldMatch;
        imageCoord0.at<cv::Point2f>(3) = H.at(3).heldMatch;

        imageCoord1.at<cv::Point2f>(0) = H.at(0).heldMatch;
        imageCoord1.at<cv::Point2f>(1) = H.at(2).heldMatch;
        imageCoord1.at<cv::Point2f>(2) = H.at(1).heldMatch;
        imageCoord1.at<cv::Point2f>(3) = H.at(3).heldMatch;
    }
    else
    {
        enoughMarkers = false;
    }
}

void Marker::setWorldTransform(cv::Mat worldTransform)
{
    this->worldTransform  = worldTransform;
}

cv::Mat Marker::getWorldTransform()
{
    return worldTransform;
}

cv::Mat Marker::getWorldCoord()
{
    return worldCoord;
}

cv::Mat Marker::getImageCoord(int orientation)
{
    if (orientation == 0)
    {
        return imageCoord0;
    }
    else if (orientation == 1)
    {
        return imageCoord1;
    }
    else
    {
        std::cout << "error: requested invalid image coordinate orientation" << std::endl;
    }
//    return imageCoordVec.at(orientation);
}

void Marker::poseEstimation(cv::Mat imgBin, int w, int h, Barcode barcode)
{

//    std::cout << "pose estimation" << std::endl;

    int flags = cv::ITERATIVE;
    bool useExtrinsicGuess = false;
    int numOrientations = 2;

    int foundMarker = -1;
    markerID = -1;

    // Iterate through both possible orientations
    for (int i = 0; i < numOrientations; i++)
    {
        cv::solvePnP(getWorldCoord(), getImageCoord(i), barcode.cameraMatrix, barcode.distCoeffs, rvec, tvec, useExtrinsicGuess, flags);

        // if rvec and tvec != 0
        if (!markerTransformationZero())
        {
            barcode.projectSamplePoints(rvec, tvec);

            // Get barcode value
            foundMarker = barcode.getMarkerNumber(imgBin);
            markerID = foundMarker/4;
            rotNum = foundMarker%4;

            barcode.rotateOrigin(rotNum, &rvec, &tvec);
            rotateWorldCoord (rotNum);

            // if marker is found and not looking from behind
            if (foundMarker != -1 && barcode.zDirection(rvec))
            {
                std::cout << "found marker number: " << markerID << std::endl;
                imgCoordOrientation = i;
//                std::cout << "mod: " << rotNum << std::endl;

                i = 100; // break out of loop
            }
        }
    }
    if (foundMarker == -1)
    {
        std::cout << "no marker found" << std::endl;

        // reset rvec and tvec
        rvec.at<double>(0) = 0;
        rvec.at<double>(1) = 0;
        rvec.at<double>(2) = 0;

        tvec.at<double>(0) = 0;
        tvec.at<double>(1) = 0;
        tvec.at<double>(2) = 0;
    }

}

cv::Mat Marker::projectAxis(cv::Mat img, Barcode barcode)
{
    // project axis
    axis.at<cv::Point3f>(0) = (cv::Point3f){0,0,0};
    axis.at<cv::Point3f>(1) = (cv::Point3f){0.1,0,0};
    axis.at<cv::Point3f>(2) = (cv::Point3f){0,0.1,0};
    axis.at<cv::Point3f>(3) = (cv::Point3f){0,0,0.1};

    cv::projectPoints(axis, rvec, tvec, barcode.cameraMatrix, barcode.distCoeffs, projectedAxis);

    cv::line(img, projectedAxis[0], projectedAxis[1], cv::Scalar(0,0,255), 2);
    cv::line(img, projectedAxis[0], projectedAxis[2], cv::Scalar(0,255,0), 2);
    cv::line(img, projectedAxis[0], projectedAxis[3], cv::Scalar(255,0,0), 2);

    return img;
}

cv::Mat Marker::projectTransformAxis(cv::Mat img, Barcode barcode, cv::Mat newWorldTransform)
{

    // create input transform from rvec tvec
    cv::Mat rMat = cv::Mat(3,3,cv::DataType<double>::type);
    cv::Rodrigues(rvec, rMat);
    cv::Mat markerTransform = cv::Mat(4,4,cv::DataType<double>::type);;
    markerTransform.at<double>(0,0) = rMat.at<double>(0,0); markerTransform.at<double>(0,1) = rMat.at<double>(0,1); markerTransform.at<double>(0,2) = rMat.at<double>(0,2); markerTransform.at<double>(0,3) = tvec.at<double>(0);
    markerTransform.at<double>(1,0) = rMat.at<double>(1,0); markerTransform.at<double>(1,1) = rMat.at<double>(1,1); markerTransform.at<double>(1,2) = rMat.at<double>(1,2); markerTransform.at<double>(1,3) = tvec.at<double>(1);
    markerTransform.at<double>(2,0) = rMat.at<double>(2,0); markerTransform.at<double>(2,1) = rMat.at<double>(2,1); markerTransform.at<double>(2,2) = rMat.at<double>(2,2); markerTransform.at<double>(2,3) = tvec.at<double>(2);
    markerTransform.at<double>(3,0) = 0;                    markerTransform.at<double>(3,1) = 0;                    markerTransform.at<double>(3,2) = 0;                    markerTransform.at<double>(3,3) = 1;

    // transform rvec and tvec to create new rvec tvec
    cv::Mat markerWorld = markerTransform * newWorldTransform;

    cv::Mat rMatNew = cv::Mat(3,3,cv::DataType<double>::type);;
    cv::Mat rvecNew = cv::Mat(3,1,cv::DataType<double>::type);;
    cv::Mat tvecNew = cv::Mat(3,1,cv::DataType<double>::type);;

    rMatNew.at<double>(0,0) = markerWorld.at<double>(0,0); rMatNew.at<double>(0,1) = markerWorld.at<double>(0,1); rMatNew.at<double>(0,2) = markerWorld.at<double>(0,2);
    rMatNew.at<double>(1,0) = markerWorld.at<double>(1,0); rMatNew.at<double>(1,1) = markerWorld.at<double>(1,1); rMatNew.at<double>(1,2) = markerWorld.at<double>(1,2);
    rMatNew.at<double>(2,0) = markerWorld.at<double>(2,0); rMatNew.at<double>(2,1) = markerWorld.at<double>(2,1); rMatNew.at<double>(2,2) = markerWorld.at<double>(2,2);

    cv::Rodrigues(rMatNew, rvecNew);

    tvecNew.at<double>(0) = markerWorld.at<double>(0,3);
    tvecNew.at<double>(1) = markerWorld.at<double>(1,3);
    tvecNew.at<double>(2) = markerWorld.at<double>(2,3);

    // project axis
    axis.at<cv::Point3f>(0) = (cv::Point3f){0,0,0};
    axis.at<cv::Point3f>(1) = (cv::Point3f){0.1,0,0};
    axis.at<cv::Point3f>(2) = (cv::Point3f){0,0.1,0};
    axis.at<cv::Point3f>(3) = (cv::Point3f){0,0,0.1};

    cv::projectPoints(axis, rvecNew, tvecNew, barcode.cameraMatrix, barcode.distCoeffs, projectedAxis);

    cv::line(img, projectedAxis[0], projectedAxis[1], cv::Scalar(0,0,255), 2);
    cv::line(img, projectedAxis[0], projectedAxis[2], cv::Scalar(0,255,0), 2);
    cv::line(img, projectedAxis[0], projectedAxis[3], cv::Scalar(255,0,0), 2);

    return img;
}

cv::Mat Marker::projectBarcodeGrid(cv::Mat img, Barcode barcode)
{
    // Project barcode layout
    barcode.barcodeGrid.at<cv::Point3f>(0) = (cv::Point3f){-0.031,-0.031,0};
    barcode.barcodeGrid.at<cv::Point3f>(1) = (cv::Point3f){-0.031, 0.031,0};
    barcode.barcodeGrid.at<cv::Point3f>(2) = (cv::Point3f){ 0.031,-0.031,0};
    barcode.barcodeGrid.at<cv::Point3f>(3) = (cv::Point3f){ 0.031, 0.031,0};

    cv::projectPoints(barcode.barcodeGrid, rvec, tvec, barcode.cameraMatrix, barcode.distCoeffs, projectedGrid);

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

// return true if the transformation is not valid
bool Marker::markerTransformationZero()
{
    return rvec.at<double>(0) == 0 && rvec.at<double>(1) == 0 && rvec.at<double>(2) == 0 &&
           tvec.at<double>(0) == 0 && tvec.at<double>(1) == 0 && tvec.at<double>(2) == 0;
}

std::vector<HoldPoint> Marker::sortPointsVertically(std::vector<HoldPoint> H)
{
    // copy H vector into a list for ordering
    std::list<HoldPoint> HList;
    int jj = 0;
    for (std::vector<HoldPoint>::iterator it = H.begin(); it != H.end(); it++)
    {
        if (jj < numMarkers)
        {
            HList.push_back(*it);
            imageCoord.at<cv::Point2f>(jj) = it->heldMatch;
            jj++;
        }
    }

    // sort holdpoints
    std::list<HoldPoint> Hsorted = mergesort(HList);

    // copy sorted points back into vector
    std::vector<HoldPoint> Hvector;
    jj = 0;
    for (std::list<HoldPoint>::iterator it = Hsorted.begin(); it != Hsorted.end(); it++)
    {
        if (jj < numMarkers)
        {
//            std::cout << it->heldMatch << std::endl;
            Hvector.push_back(*it);
            imageCoord.at<cv::Point2f>(jj) = it->heldMatch;
            jj++;
        }
    }

    return Hvector;
}

std::list<HoldPoint> Marker::mergesort(std::list<HoldPoint> H)
{
    std::list<HoldPoint> left;
    std::list<HoldPoint> right;
    std::list<HoldPoint> result;
    if (H.size() <= 1)
    {
        return H;
    }
    else
    {
        int middle = H.size() / 2;
        int i = 0;
        for (std::list<HoldPoint>::iterator it = H.begin(); it != H.end(); it++)
        {
            if (i < middle)
            {
                left.push_back(*it);
            }
            else
            {
                right.push_back(*it);
            }
            i++;
        }
    }

    left = mergesort(left);
    right = mergesort(right);
    result = merge(left, right);

    return result;
}

std::list<HoldPoint> Marker::merge(std::list<HoldPoint> left, std::list<HoldPoint> right)
{
    std::list<HoldPoint> result;

    while (left.size() > 0 && right.size() > 0)
    {
        if (left.front().heldMatch.y <= right.front().heldMatch.y)
        {
            result.push_back(left.front());
            left.pop_front();
        }
        else
        {
            result.push_back(right.front());
            right.pop_front();
        }
    }
    while (left.size() > 0)
    {
        result.push_back(left.front());
        left.pop_front();
    }
    while (right.size() > 0)
    {
        result.push_back(right.front());
        right.pop_front();
    }
    return result;
}

void Marker::averageVec ()
{
    static std::queue<double> rvecQueue0;
    static std::queue<double> rvecQueue1;
    static std::queue<double> rvecQueue2;
    static std::queue<double> tvecQueue0;
    static std::queue<double> tvecQueue1;
    static  std::queue<double> tvecQueue2;

    static double rvecSum0;
    static double rvecSum1;
    static  double rvecSum2;
    static double tvecSum0;
    static double tvecSum1;
    static double tvecSum2;

    // Add vectors to sum
    rvecSum0 = rvecSum0 + rvec.at<double>(0);
    rvecSum1 = rvecSum1 + rvec.at<double>(1);
    rvecSum2 = rvecSum2 + rvec.at<double>(2);
    tvecSum0 = tvecSum0 + tvec.at<double>(0);
    tvecSum1 = tvecSum1 + tvec.at<double>(1);
    tvecSum2 = tvecSum2 + tvec.at<double>(2);

    // Add vectors to queue
    rvecQueue0.push(rvec.at<double>(0));
    rvecQueue1.push(rvec.at<double>(1));
    rvecQueue2.push(rvec.at<double>(2));
    tvecQueue0.push(tvec.at<double>(0));
    tvecQueue1.push(tvec.at<double>(1));
    tvecQueue2.push(tvec.at<double>(2));

    if (rvecQueue0.size() >= averageingWindow)
    {
        double rvecOld0 = rvecQueue0.front();
        double rvecOld1 = rvecQueue1.front();
        double rvecOld2 = rvecQueue2.front();
        double tvecOld0 = tvecQueue0.front();
        double tvecOld1 = tvecQueue1.front();
        double tvecOld2 = tvecQueue2.front();

        rvecQueue0.pop();
        rvecQueue1.pop();
        rvecQueue2.pop();
        tvecQueue0.pop();
        tvecQueue1.pop();
        tvecQueue2.pop();

        rvecSum0 = rvecSum0 - rvecOld0;
        rvecSum1 = rvecSum1 - rvecOld1;
        rvecSum2 = rvecSum2 - rvecOld2;
        tvecSum0 = tvecSum0 - tvecOld0;
        tvecSum1 = tvecSum1 - tvecOld1;
        tvecSum2 = tvecSum2 - tvecOld2;

        rvec.at<double>(0) = rvecSum0 / rvecQueue0.size();
        rvec.at<double>(1) = rvecSum1 / rvecQueue1.size();
        rvec.at<double>(2) = rvecSum2 / rvecQueue2.size();
        tvec.at<double>(0) = tvecSum0 / tvecQueue0.size();
        tvec.at<double>(1) = tvecSum1 / tvecQueue1.size();
        tvec.at<double>(2) = tvecSum2 / tvecQueue2.size();
    }
}
