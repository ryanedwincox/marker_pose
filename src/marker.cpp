#include "marker.h"

Marker::Marker()
{
    numMarkers = 4; // targets per marker
    targetSpacing = 0.105; // in meters

    foundMarker = -1;

    // Initialize rvec tvec
    rvec = cv::Mat(3,1,cv::DataType<double>::type);
    tvec = cv::Mat(3,1,cv::DataType<double>::type);

    // reset rvec and tvec
    rvec.at<double>(0) = 0;
    rvec.at<double>(1) = 0;
    rvec.at<double>(2) = 0;

    tvec.at<double>(0) = 0;
    tvec.at<double>(1) = 0;
    tvec.at<double>(2) = 0;

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

    imgCoordOrientation = -1;
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

            worldPoint = affineRotation * worldPoint;
//            std::cout << "worldPoint: " << worldPoint << std::endl;

            worldCoord.at<cv::Point3f>(i).x = (float)worldPoint.at<double>(0);
            worldCoord.at<cv::Point3f>(i).y = (float)worldPoint.at<double>(1);
            worldCoord.at<cv::Point3f>(i).z = (float)worldPoint.at<double>(2);
        }
    }
}

// takes a vertically sorted vector of HoldPoints and puts them in imageCoordVec in every possible order
void Marker::setImageCoord(std::vector<HoldPoint> H)
{
//    std::cout << H.size() << std::endl;
    if (foundMarkers >= numMarkers)
    {
        enoughMarkers = true;

        // There are two possible orders for the SSLs sorted vertically to make a square
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

// returns world coordinates transformed by the world transform
cv::Mat Marker::getWorldCoord()
{
    return worldCoord;
}

// returns world coordinates transformed by the world transform
cv::Mat Marker::getWorldCoordTransformed()
{
    cv::Mat worldCoordCopy = cv::Mat(numMarkers,1,cv::DataType<cv::Point3f>::type);
    for (int i = 0; i < 4; i++)
    {
        cv::Mat temp = cv::Mat(4,1,cv::DataType<double>::type);
        temp.at<double>(0) = (double)worldCoord.at<cv::Point3f>(i).x;
        temp.at<double>(1) = (double)worldCoord.at<cv::Point3f>(i).y;
        temp.at<double>(2) = (double)worldCoord.at<cv::Point3f>(i).z;
        temp.at<double>(3) = 1;

        temp = worldTransform * temp;

        worldCoordCopy.at<cv::Point3f>(i).x = (float)temp.at<double>(0);
        worldCoordCopy.at<cv::Point3f>(i).y = (float)temp.at<double>(1);
        worldCoordCopy.at<cv::Point3f>(i).z = (float)temp.at<double>(2);
    }
//    std::cout << "worldCoordCopy ***: " << worldCoordCopy << std::endl;
    return worldCoordCopy;
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

    foundMarker = -1;
    markerID = -1;

    // Iterate through both possible orientations
    for (int i = 0; i < numOrientations; i++)
    {
        cv::solvePnP(getWorldCoord(), getImageCoord(i), barcode.cameraMatrix, barcode.distCoeffs, rvec, tvec, useExtrinsicGuess, flags);

        // if rvec and tvec != 0
        if (!markerTransformationZero())
        {
            barcode.projectSamplePoints(rvec, tvec);
            barcode.projectSampleRegions(rvec, tvec);

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

int Marker::getMarkerID()
{
    return markerID;
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
    barcode.barcodeGrid.at<cv::Point3f>(0) = (cv::Point3f){-barcode.barcodeGridWidth/2,-barcode.barcodeGridWidth/2,0};
    barcode.barcodeGrid.at<cv::Point3f>(1) = (cv::Point3f){-barcode.barcodeGridWidth/2, barcode.barcodeGridWidth/2,0};
    barcode.barcodeGrid.at<cv::Point3f>(2) = (cv::Point3f){ barcode.barcodeGridWidth/2,-barcode.barcodeGridWidth/2,0};
    barcode.barcodeGrid.at<cv::Point3f>(3) = (cv::Point3f){ barcode.barcodeGridWidth/2, barcode.barcodeGridWidth/2,0};

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



MatrixXd Marker::cvMatToEigen(cv::Mat input, int rows, int cols)
{
    MatrixXd output(rows, cols);
    for (int r = 0; r < rows; r++)
    {
        for (int c = 0; c < cols; c++)
        {
            output(r,c) = input.at<double>(r,c);
        }
    }
    return output;
}

cv::Mat Marker::eigenToCvMat(MatrixXd input, int rows, int cols)
{
    cv::Mat output = cv::Mat(rows,cols,cv::DataType<double>::type);
    for (int r = 0; r < rows; r++)
    {
        for (int c = 0; c < cols; c++)
        {
            output.at<double>(r,c) = input(r,c);
        }
    }
    return output;
}
