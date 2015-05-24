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
    Matrix4d averageVec(Matrix4d T);
    void setMarkerTransforms();
    vector<Matrix4d> getMarkerWorldTransforms();

    bool validPoseEstimate;

private:
    int averageingWindow;
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
};

#endif // MARKERMANAGER_H
