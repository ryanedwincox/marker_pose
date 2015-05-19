#include "markermanager.h"

MarkerManager::MarkerManager(int numMarkers, Barcode barcode)
{
    this->numMarkers = numMarkers;
    this->barcode = barcode;

    markers = vector<Marker> (numMarkers);

    // populate markers
    for (int i = 0; i < numMarkers; i++)
    {
        Marker newMarker;
        markers.at(i) = newMarker;
    }

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

    averageingWindow = 25;
    validPoseEstimate = false;

    // define all marker world transforms
    markerWorldTransforms = vector<Matrix4d> (numMarkers);

    Matrix4d marker0WorldTransform;
    marker0WorldTransform << 1,0,0,0,
                             0,1,0,0,
                             0,0,1,0,
                             0,0,0,1;
    markerWorldTransforms[0] = marker0WorldTransform;

    Matrix4d marker1WorldTransform;
    double theta = PI/2;
    marker1WorldTransform << cos(theta), 0, -sin(theta), 0.10795, // office
                             0,          1, 0,           0,
                             sin(theta), 0, cos(theta),  0.3556,
                             0,          0, 0,           1;
//    marker1WorldTransform << 1,0,0,0,
//                             0,1,0,0,
//                             0,0,1,0,
//                             0,0,0,1;
//    marker1WorldTransform << 1,0,0,0.3683,
//                             0,1,0,0,
//                             0,0,1,0,
//                             0,0,0,1;
   markerWorldTransforms[1] = marker1WorldTransform;

   poseEst = SolveP3P();
}

void MarkerManager::setMarkerTransforms()
{
    for (int i = 0; i < numMarkers; i++)
    {
        // for every marker set transform based on that markers ID
        markers[i].setWorldTransform(markers[i].eigenToCvMat(markerWorldTransforms[markers[i].getMarkerID()],4,4));
    }
}

vector<Matrix4d> MarkerManager::getMarkerWorldTransforms()
{
    return markerWorldTransforms;
}

// returns a vector of all markers
vector<Marker> MarkerManager::getMarkers()
{
    return markers;
}

// TODO make sure order of markerIDs in H matches order of markers which is 1234...
void MarkerManager::clusterTargetInputs(vector<HoldPoint> H)
{
    this->H = H;
    int numMatches = H.size();

    // find the distances from the first target to every other target
    if (numMatches > 0)
    {
        // find all clusters of target
        for (int i = 0; i < numMatches/4; i++)
        {
            if (i < numMarkers)
            {
                vector<HoldPoint> cluster;
                cluster = findTargetCluster();

                drawTargets(cluster, cv::Scalar(255,0+(255/(numMatches/4))*i,0));  // temporary ****************
                std::vector<HoldPoint> clusterSorted = markers.at(i).sortPointsVertically(cluster);

                markers.at(i).foundMarkers = clusterSorted.size();
                markers.at(i).setImageCoord(clusterSorted);
                markers.at(i).setWorldCoord();
            }
        }
    }
}

vector<HoldPoint> MarkerManager::findTargetCluster()
{
    int numMatches = H.size();

    // stores the distance from the first target to every other target in dist
    dist = vector<int>(numMatches);
    cv::Point start = H.at(0).heldMatch;
    for (int i = 0; i < numMatches; i++)
    {
        cv::Point end =  H.at(i).heldMatch;
        dist.at(i) = sqrt(pow(end.x - start.x, 2) + pow(end.y - start.y, 2));
    }

    // loop through and find the shortest distance 3 times
    std::vector<HoldPoint> newCluster;
    newCluster.push_back(H.at(0));

    dist[0] = INT_MAX;
    for (int i = 0; i < 3; i++)
    {
        int min = INT_MAX;
        int indexOfMin = -1;
        for (int j = 1; j < numMatches; j++) // exclude the distance from the first element to the first element because it is zero
        {
            if (dist.at(j) < min && dist.at(j) > 10) // > 20 to avoid duplicate points
            {
                min = dist.at(j);
                indexOfMin = j;
            }
        }
        if (indexOfMin >= 0)
        {
            newCluster.push_back(H.at(indexOfMin));
            dist.at(indexOfMin) = INT_MAX; // Make sure the same min value is not used again
        }
    }

    // remove all used values from H and dist
    for (int i = 0; i < numMatches; i++)
    {
        if (dist.at(i) == INT_MAX)
        {
            dist.erase(dist.begin() + i);
            H.erase(H.begin() + i);
            i--;
            numMatches--;
        }
    }
    return newCluster;
}

Matrix4d MarkerManager::estimateWorldPose()
{
    for (int i = 0; i < numMarkers; i++)
    {
        if (markers[i].enoughMarkers)
        {
            // estimate pose
            markers[i].poseEstimation(imgBin, barcode.getImageWidth(), barcode.getImageHeight(), barcode);

//            cout << "markers 0 rvec: " << markers[0].rvec << endl;
//            cout << "markers 0 tvec: " << markers[0].tvec << endl;

            if (!markers[i].markerTransformationZero())
            {
    //                marker1.averageVec(); // average later

                img = markers[i].projectAxis(img, barcode);

    //                img = marker1.projectTransformAxis(img, barcode, marker2WorldTransform);

                img = markers[i].projectBarcodeGrid(img, barcode);
            }
        }
    }


    int validMarkers = 0;

    // find number of valid markers
    for (int i = 0; i < numMarkers; i++)
    {
        if (!markers[i].markerTransformationZero())
        {
            validMarkers++;
        }
    }

//    cout << "valid markers: " << validMarkers << endl;

    cv::Mat totalImageCoord = cv::Mat(4*validMarkers,1,cv::DataType<cv::Point2f>::type);
    cv::Mat totalWorldCoord = cv::Mat(4*validMarkers,1,cv::DataType<cv::Point3f>::type);

    int validIndex = 0; // A index for the number of valid markers found which is different from numMarker index i below

    // for each marker that has been defined in the world add it to totalImgCoord and totalWorldCoord if it is valid
    for (int i = 0; i < numMarkers; i++)
    {
//        cout << "i: " << i << endl;
        // if valid match found add coordinates to totalImgCoord and totalWorldCoord
        if (!markers[i].markerTransformationZero())
        {
            cv::Mat newImageCoord = cv::Mat(4,1,cv::DataType<cv::Point2f>::type);
            cv::Mat newWorldCoord = cv::Mat(4,1,cv::DataType<cv::Point3f>::type);

            newImageCoord = markers[i].getImageCoord(markers[i].imgCoordOrientation);
//            cout << "orientation: " << markers[i].imgCoordOrientation << endl;
            newWorldCoord = markers[i].getWorldCoordTransformed();

//            cout << "new image coord: " << newImageCoord << endl;
//            cout << "new world coord: " << newWorldCoord << endl;

            totalImageCoord.at<cv::Point2f>(4*validIndex+0) = newImageCoord.at<cv::Point2f>(0);
            totalImageCoord.at<cv::Point2f>(4*validIndex+1) = newImageCoord.at<cv::Point2f>(1);
            totalImageCoord.at<cv::Point2f>(4*validIndex+2) = newImageCoord.at<cv::Point2f>(2);
            totalImageCoord.at<cv::Point2f>(4*validIndex+3) = newImageCoord.at<cv::Point2f>(3);

            totalWorldCoord.at<cv::Point3f>(4*validIndex+0) = newWorldCoord.at<cv::Point3f>(0);
            totalWorldCoord.at<cv::Point3f>(4*validIndex+1) = newWorldCoord.at<cv::Point3f>(1);
            totalWorldCoord.at<cv::Point3f>(4*validIndex+2) = newWorldCoord.at<cv::Point3f>(2);
            totalWorldCoord.at<cv::Point3f>(4*validIndex+3) = newWorldCoord.at<cv::Point3f>(3);

            validIndex++;
        }
    }

    //**
//    cout << "total image coord: " << totalImageCoord << endl;
//    cout << "total world coord: " << totalWorldCoord << endl;

//    // draw world coord for sanity check
//    cv::Mat temp = cv::Mat(8,1,cv::DataType<cv::Point2f>::type);
//    cv::projectPoints(totalWorldCoord, marker1.rvec, marker1.tvec, barcode.cameraMatrix, barcode. distCoeffs, temp);
//    for (int i = 0; i < 8; i++)
//    {
//        cv::circle(img, temp.at<cv::Point2f>(i), 3, cv::Scalar(255,255,0), -1);
//    }
    //**

    if (validMarkers > 0)
    {
        validPoseEstimate = true;
        int flags = cv::ITERATIVE;
        bool useExtrinsicGuess = false;

        // use opencv solve pnp function
        cv::solvePnP(totalWorldCoord, totalImageCoord, barcode.cameraMatrix, barcode.distCoeffs, rvec, tvec, useExtrinsicGuess, flags);

        // my own solve pnp function
        poseEst.solveP3P(totalImageCoord, totalWorldCoord, barcode.cameraMatrix, barcode.distCoeffs);

//        std::cout << "totalWorldCoord: " << totalWorldCoord << std::endl;
//        std::cout << "totalImageCoord: " << totalImageCoord << std::endl;
//        std::cout << "rvec: " << rvec << std::endl;
//        std::cout << "tvec: " << tvec << std::endl;

//        // check z direction
//        std::cout << "zdirection: " << barcode.zDirection(rvec) << std::endl;

        // project axis
        cv::Mat axis = cv::Mat(4,1,cv::DataType<cv::Point3f>::type); // 1 marker
        std::vector<cv::Point2f> projectedAxis;
        axis.at<cv::Point3f>(0) = (cv::Point3f){0,0,0};
        axis.at<cv::Point3f>(1) = (cv::Point3f){0.1,0,0};
        axis.at<cv::Point3f>(2) = (cv::Point3f){0,0.1,0};
        axis.at<cv::Point3f>(3) = (cv::Point3f){0,0,0.1};

        cv::projectPoints(axis, rvec, tvec, barcode.cameraMatrix, barcode.distCoeffs, projectedAxis);

        cv::line(img, projectedAxis[0], projectedAxis[1], cv::Scalar(0,0,255), 2);
        cv::line(img, projectedAxis[0], projectedAxis[2], cv::Scalar(0,255,0), 2);
        cv::line(img, projectedAxis[0], projectedAxis[3], cv::Scalar(255,0,0), 2);
    }
    else
    {
        validPoseEstimate = false;
    }

    // convert tvec to Eigen
    MatrixXd tvecE(3,1);
    tvecE = markers[0].cvMatToEigen(tvec,3,1);

    // convert rvec to rMat then to Eigen
    cv::Mat rMat(3,3,cv::DataType<double>::type);
    cv::Rodrigues(rvec, rMat);
    MatrixXd rMatE(3,3);
    rMatE = markers[0].cvMatToEigen(rMat,3,3);

    // Combine Eigen matricies of rvec and tvec into T
    Matrix4d T;
    T.topLeftCorner(3,3) = rMatE;
    T.topRightCorner(3,1) = tvecE;
    T.bottomLeftCorner(1,4) << 0,0,0,1;

    return T;
}

void MarkerManager::setImage(cv::Mat img, cv::Mat imgBin)
{
    this->img = img;
    this->imgBin = imgBin;
}

cv::Mat MarkerManager::getImage()
{
    return img;
}

void MarkerManager::drawTargets(std::vector<HoldPoint> H, cv::Scalar color)
{
    // Draw red taget over averaged matches
    for (std::vector<HoldPoint>::iterator it = H.begin(); it != H.end(); it++)
    {
        int l = 10; //radius of cross
        cv::Point center = it->heldMatch;

        cv::line(img, (cv::Point){center.x-l,center.y}, (cv::Point){center.x+l,center.y}, color, 2);
        cv::line(img, (cv::Point){center.x,center.y-l}, (cv::Point){center.x,center.y+l}, color, 2);
    }
}

Matrix4d MarkerManager::averageVec (Matrix4d T)
{

    cv::Mat rvecCur = cv::Mat(3,1,cv::DataType<double>::type);
    cv::Mat tvecCur = cv::Mat(3,1,cv::DataType<double>::type);

    cv::Mat rMat = cv::Mat(3,3,cv::DataType<double>::type);
    rMat = markers[0].eigenToCvMat(T.topLeftCorner(3,3),3,3);
    cv::Rodrigues(rMat, rvecCur);

    tvecCur = markers[0].eigenToCvMat(T.topRightCorner(3,1),3,1);

    static std::queue<double> rvecQueue0;
    static std::queue<double> rvecQueue1;
    static std::queue<double> rvecQueue2;
    static std::queue<double> tvecQueue0;
    static std::queue<double> tvecQueue1;
    static std::queue<double> tvecQueue2;

    static double rvecSum0;
    static double rvecSum1;
    static double rvecSum2;
    static double tvecSum0;
    static double tvecSum1;
    static double tvecSum2;

    // Add vectors to sum
    rvecSum0 = rvecSum0 + rvecCur.at<double>(0);
    rvecSum1 = rvecSum1 + rvecCur.at<double>(1);
    rvecSum2 = rvecSum2 + rvecCur.at<double>(2);
    tvecSum0 = tvecSum0 + tvecCur.at<double>(0);
    tvecSum1 = tvecSum1 + tvecCur.at<double>(1);
    tvecSum2 = tvecSum2 + tvecCur.at<double>(2);

    // Add vectors to queue
    rvecQueue0.push(rvecCur.at<double>(0));
    rvecQueue1.push(rvecCur.at<double>(1));
    rvecQueue2.push(rvecCur.at<double>(2));
    tvecQueue0.push(tvecCur.at<double>(0));
    tvecQueue1.push(tvecCur.at<double>(1));
    tvecQueue2.push(tvecCur.at<double>(2));

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

        rvecCur.at<double>(0) = rvecSum0 / rvecQueue0.size();
        rvecCur.at<double>(1) = rvecSum1 / rvecQueue1.size();
        rvecCur.at<double>(2) = rvecSum2 / rvecQueue2.size();
        tvecCur.at<double>(0) = tvecSum0 / tvecQueue0.size();
        tvecCur.at<double>(1) = tvecSum1 / tvecQueue1.size();
        tvecCur.at<double>(2) = tvecSum2 / tvecQueue2.size();
    }
    // convert back to eigen transform to return
    cv::Rodrigues(rvecCur, rMat);
    T.topLeftCorner(3,3) = markers[0].cvMatToEigen(rMat,3,3);
    T.topRightCorner(3,1) = markers[0].cvMatToEigen(tvecCur,3,1);
    return T;
}


