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
    MarkerManager(Barcode barcode);
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
    void setMarkerTransforms();
    vector<Matrix4d> getMarkerWorldTransforms();

    bool validPoseEstimate;

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
    vector<Matrix4d> markerWorldTransforms;
};

#endif // MARKERMANAGER_H
