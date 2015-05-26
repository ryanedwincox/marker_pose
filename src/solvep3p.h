//*********
// Tutorial link: http://iplimage.com/blog/p3p-perspective-point-overview/
//*********

#ifndef SOLVEP3P_H
#define SOLVEP3P_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <math.h>
#include <Eigen/Eigen>

#include "poly34.h"

#define DEBUG false

using namespace std;
using namespace Eigen;

class SolveP3P
{
public:
    SolveP3P();
    Matrix4d solveP3P(cv::Mat worldCoord, cv::Mat imageCoord, cv::Mat cameraMatrix, cv::Mat distCoeffs, cv::Mat rvec, cv::Mat tvec, cv::Mat img);
    void normalizeImagePoints(cv::Mat cameraMatrix, cv::Mat distCoeffs);
    void setUpP3PEquationSystem();
    cv::Mat solveP3PEquationSystem();
    Matrix4d chooseBestSolution();
    Matrix4d rigidTransform(cv::Mat N, cv::Mat M);

    // other functions
    MatrixXd cvMatToEigen(cv::Mat input, int rows, int cols);
    cv::Mat eigenToCvMat(MatrixXd input, int rows, int cols);
    cv::Mat rvecFromT(Matrix4d T);
    cv::Mat tvecFromT(Matrix4d T);

private:
    cv::Mat img;
    cv::Mat normalizedCoord;
    cv::Mat threeWorldCoord;
    cv::Mat threeImageCoord;
    cv::Mat imageCoord;

    // camera parameters
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    double fx;
    double fy;
    double cx;
    double cy;

    // points onimage plane
    cv::Point3f u;
    cv::Point3f v;
    cv::Point3f w;

    // angles
    double uv;
    double uw;
    double vw;

    // side lengths
    double AB;
    double BC;
    double CA;

    // coefficients
    double aa;
    double bb;

    // Wu Ritt's coefficients
    double a4;
    double a3;
    double a2;
    double a1;
    double a0;
    double b1;
    double b0;

    // solve for
    double PA;
    double PB;
    double PC;

    // Estimated 3D points
    cv::Point3f AA;
    cv::Point3f BB;
    cv::Point3f CC;

    // iterate through solutions
    int xSolu;
    int ySolu;
};

#endif // SOLVEP3P_H
