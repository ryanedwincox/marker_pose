#ifndef MARKERMANAGER_H
#define MARKERMANAGER_H

#include <vector>
#include <Eigen/Eigen>

#include "marker.h"

using namespace Eigen;
using namespace std;

typedef Matrix<double, 4, 4> Matrix4d;

class MarkerManager
{
public:
    MarkerManager(int numMarkers, Barcode barcode);
    void createMarkers();
    vector<Marker> getMarkers();
    void clusterTargetInputs(vector<HoldPoint> H);
    vector<HoldPoint> findTargetCluster();
    Matrix4d estimateWorldPose();
    void projectAxis();
    void publishTF();
    void drawTargets(std::vector<HoldPoint> H, cv::Scalar color);
    void setImage(cv::Mat img, cv::Mat imgBin);
    cv::Mat getImage();
    void publishMarkerTFs();

private:
    cv::Mat rvec;
    cv::Mat tvec;
    cv::Mat img;
    cv::Mat imgBin;
    Barcode barcode;
    int numMarkers;
    vector<HoldPoint> H;
    vector<Marker> markers;
    vector<int> dist;
};

#endif // MARKERMANAGER_H
