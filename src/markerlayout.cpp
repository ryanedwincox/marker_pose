#include "markerlayout.h"

MarkerLayout::MarkerLayout()
{
    numMarkers = 4;

    // Define 3D coordinates of markers
    worldCoord = cv::Mat(numMarkers,1,cv::DataType<cv::Point3f>::type);
    // rectangle marker pattern
    // TODO remap into meters
    worldCoord.at<cv::Point3f>(0) = (cv::Point3f){0,0,0};
    worldCoord.at<cv::Point3f>(1) = (cv::Point3f){0.2,0,0};
    worldCoord.at<cv::Point3f>(2) = (cv::Point3f){0,0.3,0};
    worldCoord.at<cv::Point3f>(3) = (cv::Point3f){0.2,0.3,0};

    // Define 2D corrdinates of markers detected in image
    imageCoord = cv::Mat(numMarkers,1,cv::DataType<cv::Point2f>::type);
    imageCoordVec = std::vector<cv::Mat>(numMarkers, imageCoord);

    enoughMarkers = false;
}

// takes a vertically sorted vector of HoldPoints and puts them in imageCoordVec in every possible order
void MarkerLayout::setImageCoord(std::vector<HoldPoint> H)
{
//    std::cout << H.size() << std::endl;
    if (H.size() >= numMarkers)
    {
        enoughMarkers = true;

        // fill in all imageCoordVec permutations.  There should be 8 total
        imageCoordVec.at(0).at<cv::Point2f>(0) = H.at(0).heldMatch;
        imageCoordVec.at(0).at<cv::Point2f>(1) = H.at(1).heldMatch;
        imageCoordVec.at(0).at<cv::Point2f>(2) = H.at(2).heldMatch;
        imageCoordVec.at(0).at<cv::Point2f>(3) = H.at(3).heldMatch;
    }
    else
    {
        enoughMarkers = false;
    }
}

cv::Mat MarkerLayout::getWorldCoord()
{
    return worldCoord;
}

cv::Mat MarkerLayout::getImageCoord(int orientation)
{
    return imageCoordVec.at(orientation);
}

std::vector<HoldPoint> MarkerLayout::sortPointsVertically(std::vector<HoldPoint> H)
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
            Hvector.push_back(*it);
            imageCoord.at<cv::Point2f>(jj) = it->heldMatch;
            jj++;
        }
    }

    return Hvector;
}

std::list<HoldPoint> MarkerLayout::mergesort(std::list<HoldPoint> H)
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

std::list<HoldPoint> MarkerLayout::merge(std::list<HoldPoint> left, std::list<HoldPoint> right)
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
