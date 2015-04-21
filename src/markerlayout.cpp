#include "markerlayout.h"

MarkerLayout::MarkerLayout()
{
    numMarkers = 4;
    targetSpacing = 0.105; // in meters

    // Define 3D coordinates of markers
    worldCoord = cv::Mat(numMarkers,1,cv::DataType<cv::Point3f>::type);
    // rectangle marker pattern
    // TODO remap into meters
    worldCoord.at<cv::Point3f>(0) = (cv::Point3f){-targetSpacing/2,-targetSpacing/2,0};
    worldCoord.at<cv::Point3f>(1) = (cv::Point3f){ targetSpacing/2,-targetSpacing/2,0};
    worldCoord.at<cv::Point3f>(2) = (cv::Point3f){-targetSpacing/2, targetSpacing/2,0};
    worldCoord.at<cv::Point3f>(3) = (cv::Point3f){ targetSpacing/2, targetSpacing/2,0};

    // Define 2D corrdinates of markers detected in image
    imageCoord = cv::Mat(numMarkers,1,cv::DataType<cv::Point2f>::type);
    imageCoord0 = cv::Mat(numMarkers,1,cv::DataType<cv::Point2f>::type);
    imageCoord1 = cv::Mat(numMarkers,1,cv::DataType<cv::Point2f>::type);
//    imageCoordVec = std::vector<cv::Mat>(8, imageCoord);

    enoughMarkers = false;
    averageingWindow = 5;
}

// takes a vertically sorted vector of HoldPoints and puts them in imageCoordVec in every possible order
void MarkerLayout::setImageCoord(std::vector<HoldPoint> H)
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

void MarkerLayout::setWorldTransform(cv::Mat worldTransform)
{
    this->worldTransform  = worldTransform;
}

cv::Mat MarkerLayout::getWorldTransform()
{
    return worldTransform;
}

cv::Mat MarkerLayout::getWorldCoord()
{
    return worldCoord;
}

cv::Mat MarkerLayout::getImageCoord(int orientation)
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
        std::cout << "error" << std::endl;
    }
//    return imageCoordVec.at(orientation);
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
//            std::cout << it->heldMatch << std::endl;
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

void MarkerLayout::averageVec (cv::Mat rvec, cv::Mat tvec)
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
