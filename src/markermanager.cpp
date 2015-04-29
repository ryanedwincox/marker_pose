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
}

// defines transforms from world origin to each marker and its number
void MarkerManager::createMarkers()
{
    Matrix4d marker0WorldTransform;
    marker0WorldTransform << 1,0,0,0,
                             0,1,0,0,
                             0,0,1,0,
                             0,0,0,1;
    markers.at(0).setWorldTransform(eigenToCvMat(marker0WorldTransform, 4, 4));

    Matrix4d marker1WorldTransform;
    double theta = PI/2;
    marker1WorldTransform << cos(theta), 0, -sin(theta), 0.10795,
                             0,          1, 0,           0,
                             sin(theta), 0, cos(theta),  0.3556,
                             0,          0, 0,           1;
    markers.at(1).setWorldTransform(eigenToCvMat(marker1WorldTransform, 4, 4));
}

// returns a vector of all markers
vector<Marker> MarkerManager::getMarkers()
{
    return markers;
}

void MarkerManager::clusterTargetInputs(vector<HoldPoint> H)
{
    this->H = H;
    int numMatches = H.size();

    // find the distances from the first target to every other target
    if (numMatches > 0)
    {
        // find all clusters of target
        for (int i = 0; i < numMatches/4; i++)
//        for (int i = 0; i < 1; i++)
        {
            vector<HoldPoint> cluster;
            cluster = findTargetCluster();

            drawTargets(cluster, cv::Scalar(255,0+255*i,0));  // temporary ****************
            std::vector<HoldPoint> clusterSorted = markers.at(i).sortPointsVertically(cluster);

            // TODO set to the correct markers position based on the marker number
            markers.at(i).foundMarkers = clusterSorted.size();
            markers.at(i).setImageCoord(clusterSorted);
            markers.at(i).setWorldCoord();
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

void MarkerManager::estimateWorldPose()
{
    for (int i = 0; i < numMarkers; i++)
    {
        if (markers[i].enoughMarkers)
        {
            // estimate pose
            markers[i].poseEstimation(imgBin, barcode.getImageWidth(), barcode.getImageHeight(), barcode);

            if (!markers[i].markerTransformationZero())
            {
    //                marker1.averageVec(); // average later

                img = markers[i].projectAxis(img, barcode);

    //                img = marker1.projectTransformAxis(img, barcode, marker2WorldTransform);

                img = markers[i].projectBarcodeGrid(img, barcode);
            }
        }
    }

//    // if rvec and tvec != 0
//    if (!marker1.markerTransformationZero())
//    {
//        cv::Mat totalImgCoord;
//        cv::Mat totalWorldCoord;
//        if (!marker2.markerTransformationZero())
//        {
////                std::cout << "found both markers" << std::endl;

//            // Define all image coordinates
//            totalImgCoord = cv::Mat(8,1,cv::DataType<cv::Point2f>::type); // 2 markers
//            cv::Mat imgCoord1 = cv::Mat(4,1,cv::DataType<cv::Point2f>::type);
//            cv::Mat imgCoord2 = cv::Mat(4,1,cv::DataType<cv::Point2f>::type);
//            imgCoord1 = marker1.getImageCoord(marker1.imgCoordOrientation);
//            imgCoord2 = marker2.getImageCoord(marker2.imgCoordOrientation);
//            totalImgCoord.at<cv::Point2f>(0) = imgCoord1.at<cv::Point2f>(0);
//            totalImgCoord.at<cv::Point2f>(1) = imgCoord1.at<cv::Point2f>(1);
//            totalImgCoord.at<cv::Point2f>(2) = imgCoord1.at<cv::Point2f>(2);
//            totalImgCoord.at<cv::Point2f>(3) = imgCoord1.at<cv::Point2f>(3);
//            totalImgCoord.at<cv::Point2f>(4) = imgCoord2.at<cv::Point2f>(0);
//            totalImgCoord.at<cv::Point2f>(5) = imgCoord2.at<cv::Point2f>(1);
//            totalImgCoord.at<cv::Point2f>(6) = imgCoord2.at<cv::Point2f>(2);
//            totalImgCoord.at<cv::Point2f>(7) = imgCoord2.at<cv::Point2f>(3);

//            // Define all world coordinates
//            totalWorldCoord = cv::Mat(8,1,cv::DataType<cv::Point3f>::type); // 2 markers
//            cv::Mat worldCoord1 = cv::Mat(4,1,cv::DataType<cv::Point2f>::type);
//            cv::Mat worldCoord2 = cv::Mat(4,1,cv::DataType<cv::Point2f>::type);
//            cv::Mat newWorldCoord2 = cv::Mat(4,1,cv::DataType<cv::Point2f>::type);
//            worldCoord1 = marker1.getWorldCoord();
//            worldCoord2 = marker2.getWorldCoord();

//            // transform world coordinates for marker 2
//            for (int i = 0; i < 4; i++)
//            {
//                cv::Mat worldPoint = cv::Mat(4,1,cv::DataType<double>::type);
//                worldPoint.at<double>(0) = (double)worldCoord2.at<cv::Point3f>(i).x;
//                worldPoint.at<double>(1) = (double)worldCoord2.at<cv::Point3f>(i).y;
//                worldPoint.at<double>(2) = (double)worldCoord2.at<cv::Point3f>(i).z;
//                worldPoint.at<double>(3) = 1;

////                    std::cout << "WorldPoint: " << worldPoint << std::endl;
////                    std::cout << "transform: " << marker2.getWorldTransform() << std::endl;

//                worldPoint = marker2.getWorldTransform() * worldPoint;
////                    std::cout << "worldPoint: " << worldPoint << std::endl;

//                newWorldCoord2.at<cv::Point3f>(i).x = (float)worldPoint.at<double>(0);
//                newWorldCoord2.at<cv::Point3f>(i).y = (float)worldPoint.at<double>(1);
//                newWorldCoord2.at<cv::Point3f>(i).z = (float)worldPoint.at<double>(2);
//            }

////                std::cout << "worldCoord2: " << newWorldCoord2 << std::endl;

//            totalWorldCoord.at<cv::Point3f>(0) = worldCoord1.at<cv::Point3f>(0);
//            totalWorldCoord.at<cv::Point3f>(1) = worldCoord1.at<cv::Point3f>(1);
//            totalWorldCoord.at<cv::Point3f>(2) = worldCoord1.at<cv::Point3f>(2);
//            totalWorldCoord.at<cv::Point3f>(3) = worldCoord1.at<cv::Point3f>(3);
//            totalWorldCoord.at<cv::Point3f>(4) = newWorldCoord2.at<cv::Point3f>(0);
//            totalWorldCoord.at<cv::Point3f>(5) = newWorldCoord2.at<cv::Point3f>(1);
//            totalWorldCoord.at<cv::Point3f>(6) = newWorldCoord2.at<cv::Point3f>(2);
//            totalWorldCoord.at<cv::Point3f>(7) = newWorldCoord2.at<cv::Point3f>(3);

////                // draw world coord for sanity check
////                cv::Mat temp = cv::Mat(8,1,cv::DataType<cv::Point2f>::type);
////                cv::projectPoints(totalWorldCoord, marker1.rvec, marker1.tvec, barcode.cameraMatrix, barcode. distCoeffs, temp);
////                for (int i = 0; i < 8; i++)
////                {
////                    cv::circle(img, temp.at<cv::Point2f>(i), 3, cv::Scalar(255,255,0), -1);
////                }
//        }
//        else
//        {
//            totalImgCoord = cv::Mat(4,1,cv::DataType<cv::Point2f>::type); // 1 marker
//            cv::Mat imgCoord1 = cv::Mat(4,1,cv::DataType<cv::Point2f>::type);
//            imgCoord1 = marker1.getImageCoord(marker1.imgCoordOrientation);
//            totalImgCoord.at<cv::Point2f>(0) = imgCoord1.at<cv::Point2f>(0);
//            totalImgCoord.at<cv::Point2f>(1) = imgCoord1.at<cv::Point2f>(1);
//            totalImgCoord.at<cv::Point2f>(2) = imgCoord1.at<cv::Point2f>(2);
//            totalImgCoord.at<cv::Point2f>(3) = imgCoord1.at<cv::Point2f>(3);

//            totalWorldCoord = cv::Mat(4,1,cv::DataType<cv::Point3f>::type); // 1 marker
//            cv::Mat worldCoord1 = cv::Mat(4,1,cv::DataType<cv::Point2f>::type);
//            worldCoord1 = marker1.getWorldCoord();
//            totalWorldCoord.at<cv::Point3f>(0) = worldCoord1.at<cv::Point3f>(0);
//            totalWorldCoord.at<cv::Point3f>(1) = worldCoord1.at<cv::Point3f>(1);
//            totalWorldCoord.at<cv::Point3f>(2) = worldCoord1.at<cv::Point3f>(2);
//            totalWorldCoord.at<cv::Point3f>(3) = worldCoord1.at<cv::Point3f>(3);
//        }

//        int flags = cv::ITERATIVE;
//        bool useExtrinsicGuess = false;
//        std::cout << totalWorldCoord << std::endl;
////            rvec = marker1.rvec;
////            tvec = marker1.tvec;
//        cv::solvePnP(totalWorldCoord, totalImgCoord, barcode.cameraMatrix, barcode.distCoeffs, rvec, tvec, useExtrinsicGuess, flags);


//        // project axis
//        cv::Mat axis = cv::Mat(4,1,cv::DataType<cv::Point3f>::type); // 1 marker
//        std::vector<cv::Point2f> projectedAxis;
//        axis.at<cv::Point3f>(0) = (cv::Point3f){0,0,0};
//        axis.at<cv::Point3f>(1) = (cv::Point3f){0.1,0,0};
//        axis.at<cv::Point3f>(2) = (cv::Point3f){0,0.1,0};
//        axis.at<cv::Point3f>(3) = (cv::Point3f){0,0,0.1};

//        cv::projectPoints(axis, rvec, tvec, barcode.cameraMatrix, barcode.distCoeffs, projectedAxis);

//        cv::line(img, projectedAxis[0], projectedAxis[1], cv::Scalar(0,0,255), 2);
//        cv::line(img, projectedAxis[0], projectedAxis[2], cv::Scalar(0,255,0), 2);
//        cv::line(img, projectedAxis[0], projectedAxis[3], cv::Scalar(255,0,0), 2);
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

MatrixXd MarkerManager::cvMatToEigen(cv::Mat input, int rows, int cols)
{
    MatrixXd output(rows, cols);
    for (int r = 0; r < rows; r++)
    {
        for (int c = 0; c < cols; c++)
        {
            output << input.at<double>(r,c);
        }
    }
    return output;
}

cv::Mat MarkerManager::eigenToCvMat(Matrix4d input, int rows, int cols)
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
