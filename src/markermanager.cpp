#include "markermanager.h"

MarkerManager::MarkerManager(int numMarkers)
{
    this->numMarkers = numMarkers;

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
    // put all holdpoints into an array
    int numMatches = H.size();
//        std::cout << "numMatches: " << numMatches << std::endl;
    holdPointsArray = new HoldPoint [numMatches];
    int k = 0;
//        std::cout << "targets" << std::endl;
    for (std::vector<HoldPoint>::iterator it = H.begin(); it != H.end(); it++)
    {
        holdPointsArray[k] = *it;
//            std::cout << holdPointsArray[k].heldMatch << std::endl;
        k++;
    }

    // find the distances from the first target to every other target
    dist = new int [numMatches]; // stores the distance from the first target to every other target
    cv::Point start = holdPointsArray[0].heldMatch;
//        std::cout << "distances" << std::endl;
    for (int i = 0; i < numMatches; i++)
    {
        cv::Point end =  holdPointsArray[i].heldMatch;
        dist[i] = sqrt(pow(end.x - start.x, 2) + pow(end.y - start.y, 2));
//            std::cout << dist[i] << std::endl;
    }

//    // loop through and find the shortest distance 3 times
//    std::vector<HoldPoint> H1;
//    H1.push_back(holdPointsArray[0]);
////        std::cout << "shortest" << std::endl;
//    for (int i = 0; i < 3; i++)
//    {
//        int min = INT_MAX;
//        int indexOfMin;
//        for (int j = 1; j < numMatches; j++) // exclude the distance from the first element to the first element because it is zero
//        {
//            if (dist[j] < min && dist[j] > 20) // > 20 to avoid duplicate points
//            {
////                    std::cout << dist[j] <<std::endl;
//                min = dist[j];
//                indexOfMin = j;
//            }
//        }
////            std::cout << dist[indexOfMin] << std::endl;
//        H1.push_back(holdPointsArray[indexOfMin]);
//        dist[indexOfMin] = INT_MAX; // Make sure the same min value is not used again
//    }
//    dist[0] = INT_MAX;

    // find next target
    std::vector<HoldPoint> H2;
    if (numMatches >= 8)
    {
        // create new array with all elements with distance not set to INT_MAX
        HoldPoint holdPointsArray2 [numMatches-4];
        int dist2 [numMatches-4];
        int g = 0;
//            std::cout << "reused targets " << std::endl;
        for (int i = 0; i < numMatches; i++)
        {
            if (dist[i] < INT_MAX)
            {
                holdPointsArray2[g] = holdPointsArray[i];
                dist2[g] = dist[i];
//                    std::cout << holdPointsArray2[g].heldMatch << std::endl;
                g++;
            }
        }

        // loop through and find the shortest distance 3 times

        H2.push_back(holdPointsArray2[0]);
//        std::cout << "shortest" << std::endl;
        for (int i = 0; i < 3; i++)
        {
            int min = INT_MAX;
            int indexOfMin;
            for (int j = 1; j < numMatches; j++) // exclude the distance from the first element to the first element because it is zero
            {
                if (dist2[j] < min && dist2[j] > 20) // > 20 to avoid duplicate points
                {
//                    std::cout << dist[j] <<std::endl;
                    min = dist2[j];
                    indexOfMin = j;
                }
            }
//            std::cout << dist[indexOfMin] << std::endl;
            H2.push_back(holdPointsArray2[indexOfMin]);
            dist2[indexOfMin] = INT_MAX; // Make sure the same min value is not used again
        }
        dist2[0] = INT_MAX;
    }
    // Draw targets over averaged matches
    img = drawTargets(img, H1, cv::Scalar(0,0,255));
    img = drawTargets(img, H2, cv::Scalar(0,255,0));

    std::vector<HoldPoint> H1sorted = marker1.sortPointsVertically(H1);
    std::vector<HoldPoint> H2sorted = marker2.sortPointsVertically(H2);

    // ******** not sure where this belongs
    marker1.foundMarkers = H1.size();
    marker1.setImageCoord(H1sorted);
    marker2.foundMarkers = H2.size();
    marker2.setImageCoord(H2sorted);

    marker1.setWorldCoord();
    marker2.setWorldCoord();
}

void MarkerManager::findTargetCluster()
{
    // loop through and find the shortest distance 3 times
    std::vector<HoldPoint> H1;
    H1.push_back(holdPointsArray[0]);
//        std::cout << "shortest" << std::endl;
    for (int i = 0; i < 3; i++)
    {
        int min = INT_MAX;
        int indexOfMin;
        for (int j = 1; j < numMatches; j++) // exclude the distance from the first element to the first element because it is zero
        {
            if (dist[j] < min && dist[j] > 20) // > 20 to avoid duplicate points
            {
//                    std::cout << dist[j] <<std::endl;
                min = dist[j];
                indexOfMin = j;
            }
        }
//            std::cout << dist[indexOfMin] << std::endl;
        H1.push_back(holdPointsArray[indexOfMin]);
        dist[indexOfMin] = INT_MAX; // Make sure the same min value is not used again
    }
    dist[0] = INT_MAX;
}

cv::Mat MarkerManager::estimateWorldPose(vector<Marker>)
{

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
