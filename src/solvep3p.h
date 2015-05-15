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
    void solveP3P(cv::Mat imageCoord, cv::Mat cameraMatrix, cv::Mat distCoeffs);
    void normalizeImagePoints(cv::Mat imageCoord, cv::Mat cameraMatrix, cv::Mat distCoeffs);
    void setUpP3PEquationSystem();
    void solveP3PEquationSystem();

private:
    cv::Mat normalizedCoord;

    // camera parameters
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    double fx;
    double fy;
    double cx;
    double cy;

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
};

#endif // SOLVEP3P_H
