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
    MarkerManager(int numMarkers);
    void createMarkers();
    vector<Marker> getMarkers();
    cv::Mat clusterTargetInputs(vector<HoldPoint> H, cv::Mat img);
    vector<HoldPoint> findTargetCluster();
    cv::Mat estimateWorldPose(vector<Marker>);
    void projectAxis();
    void publishTF();
    cv::Mat drawTargets(cv::Mat img, std::vector<HoldPoint> H, cv::Scalar color);

private:
    MatrixXd cvMatToEigen(cv::Mat input, int rows, int cols);
    cv::Mat eigenToCvMat(Matrix4d input, int rows, int cols);

    int numMarkers;
    vector<HoldPoint> H;
    vector<Marker> markers;
    vector<int> dist;
};

#endif // MARKERMANAGER_H
