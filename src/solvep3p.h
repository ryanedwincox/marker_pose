#ifndef SOLVEP3P_H
#define SOLVEP3P_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <math.h>

#include "poly34.h"

using namespace std;

class SolveP3P
{
public:
    SolveP3P();
    void solveP3P(cv::Mat imageCoord, cv::Mat worldCoord, cv::Mat cameraMatrix, cv::Mat distCoeffs);
    void normalizeImagePoints(cv::Mat imageCoord, cv::Mat cameraMatrix, cv::Mat distCoeffs);
    void setUpP3PEquationSystem();
    cv::Mat solveP3PEquationSystem();
    void rigidTransform(cv::Mat N, cv::Mat M);

private:
    cv::Mat normalizedCoord;

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
    double a;
    double b;

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
    cv::Point3f A;
    cv::Point3f B;
    cv::Point3f C;
};

#endif // SOLVEP3P_H
