//#include <QApplication>
//#include <CL/cl.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <vector>
#include <algorithm>
#include <queue>
#include "math.h"
#include "limits.h"
#include <sstream>

#include "search.h"
#include "holdpoint.h"
#include "marker.h"
#include "barcode.h"
#include "combinations.h"
#include "markermanager.h"
#include "/opt/ros/groovy/include/opencv2/video/tracking.hpp"

// ROS includes
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>

// declare local methods
std::list<cv::Point> readMatches(cv::Mat img, Search s, std::list<cv::Point> matches, int matchIndex, bool horz);
std::list<cv::Point> averageMatches(std::list<cv::Point> matches);
cv::Mat drawTargets(cv::Mat img, std::vector<HoldPoint> H, cv::Scalar color);
void poseEstimation(cv::Mat imgBin, cv::Mat rvec, cv::Mat tvec, int w, int h, cv::Mat cameraMatrix, cv::Mat distCoeffs, Marker marker, Barcode barcode, Combinations comb);
void publishMarkerTFs(vector<Matrix4d> markerTransforms, const char* parent);
void publishTF(Matrix4d T, const char* parent, const char* node);
void comb(int N, int K);

int main(int argc, char *argv[])
{
    //Create video capture object
    int cameraNum = 0;
//    const char* filename = "/home/pierre/Dropbox/uh/uh1/ros_ws/marker_pose/Video/Portage Bay Marker Test 4 27 15.avi";
    const char* filename = "/home/pierre/Dropbox/uh/uh1/ros_ws/marker_pose/Video/PortageBayMarkerDD.avi";
    cv::VideoCapture cap(cameraNum);
    if(!cap.isOpened())  // check if we succeeded
    {
        std::cout << "camera not found" << std::endl;
        return -1;
    }
//    cap.set(CV_CAP_PROP_FPS,30);

//    double fps=cap.get(CV_CAP_PROP_FPS);
//    std::cout << fps << std::endl;

    // define kernel files
    const char * findSSLClPath = "/home/pierre/Dropbox/uh/uh1/ros_ws/marker_pose/cl/findSSL.cl";

    // Initialize OpenCL
    Search s1;
    Search s2;
    cl_int win = 40;
    cl_double p = 0.5;
    s1.buildProgram(findSSLClPath, win, p);
    s2.buildProgram(findSSLClPath, win, p);

    // Create vector of holdPoint filters for each marker
    HoldPoint hold;
    std::vector<HoldPoint> H;


    // firstTime is used to insure the image buffers are only created once
    bool firstTime = true;

    cv::Mat rvec(3,1,cv::DataType<double>::type);
    cv::Mat tvec(3,1,cv::DataType<double>::type);

    // Camera matrices from camera calibration
    cv::Mat cameraMatrix(3,3,cv::DataType<double>::type);
    cameraMatrix.at<double>(0,0) = 644.50;
    cameraMatrix.at<double>(0,1) = 0;
    cameraMatrix.at<double>(0,2) = 339.18;
    cameraMatrix.at<double>(1,0) = 0;
    cameraMatrix.at<double>(1,1) = 600.9586;
    cameraMatrix.at<double>(1,2) = 244.52;
    cameraMatrix.at<double>(2,0) = 0;
    cameraMatrix.at<double>(2,1) = 0;
    cameraMatrix.at<double>(2,2) = 1;

    cv::Mat distCoeffs(5,1,cv::DataType<double>::type);
    distCoeffs.at<double>(0) = 0.09386;
    distCoeffs.at<double>(1) = 0.03747;
    distCoeffs.at<double>(2) = 0.0026472;
    distCoeffs.at<double>(3) = 0.00422;
    distCoeffs.at<double>(4) = -0.4924;  

    ros::init(argc, argv, "marker_tf_broadcaster");

    ros::NodeHandle nh;
    //ros::Subscriber sub = nh.subscribe("marker/pose", 10, &poseCallback);

    cv::Mat img;
    cap >> img;

    int w = img.cols;
    int h = img.rows;

    Barcode barcode;
    barcode.setCameraParmeters(cameraMatrix, distCoeffs, w, h);

    int numMarkers = 2;
    MarkerManager markerManager(numMarkers, barcode);

    while (cap.isOpened())
    {
        cap >> img;


        if (VERBOSE)
        {
            std::cout << "image width: " << w << " image height: " << h << std::endl;
        }

        // convert to grayscale
        cv::Mat imgGray;
        cvtColor(img, imgGray, CV_BGR2GRAY);

        // convert to binary
        int blockSize = 75;
        int c = 0;
        cv::Mat imgBin;
        cv::adaptiveThreshold(imgGray, imgBin, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, blockSize, c);
//        double thresh = 175;
//        cv::threshold(imgGray, imgBin, thresh, 255, cv::THRESH_BINARY);

        // transpose for verticle detection
        cv::Mat imgBinVert;
        cv::transpose(imgBin, imgBinVert);

        if (firstTime)
        {
            s1.setImage(imgBin);
            s2.setImage(imgBinVert);
            firstTime = false;
        }

        // Run OpenCV kernel to find markers
        s1.runProgram(imgBin);
        s2.runProgram(imgBinVert);
        // newDataPointer is used to display image
//        unsigned char* newDataPointer1 = (unsigned char*) s1.readOutput();
//        unsigned char* newDataPointer2 = (unsigned char*) s2.readOutput();
        int matchIndex1 = s1.readMatchesIndexOutput();
        int matchIndex2 = s2.readMatchesIndexOutput();

        // read matches from kernel
        std::list< cv::Point > matches;
        matches = readMatches(img, s1, matches, matchIndex1, true);
        matches = readMatches(img, s2, matches, matchIndex2, false);

//        std::cout << "Number of matches: " << matches.size() <<std::endl;

        // Average clusters
        std::list<cv::Point> avgMatches = averageMatches(matches);

        H = hold.holdPoints(H, avgMatches);

        markerManager.setImage(img, imgBin);

        markerManager.drawTargets(H, cv::Scalar(0,0,255));

        markerManager.clusterTargetInputs(H);

        markerManager.setMarkerTransforms();

        Matrix4d T = markerManager.estimateWorldPose();

        T = markerManager.averageVec(T);

//        std::cout << "t: " << T << std::endl;

//        img = markerManager.getImage(); // don't seem to need this

        // Take inverse of T
        // convert rvec to rMat then to Eigen and find inverse
        MatrixXd rMat(3,3);
        MatrixXd rMatTrans(3,3);
        rMat = T.topLeftCorner(3,3);
        rMatTrans = rMat.transpose();

        // convert tvec to Eigen and find inverse
        MatrixXd tvec(3,1);
        tvec = T.topRightCorner(3,1);
        tvec = -rMatTrans*tvec;

        Matrix4d camTf;
        camTf.topLeftCorner(3,3) = rMatTrans;
        camTf.topRightCorner(3,1) = tvec;
        camTf.bottomLeftCorner(1,4) << 0,0,0,1;

        Matrix4d markerOrigin;
        markerOrigin << 1,0,0,0,
                        0,1,0,1,
                        0,0,1,0,
                        0,0,0,1;

        double theta = PI/2;
        Matrix4d rotX;
        rotX << 1,0,0,0,
                0,cos(theta),-sin(theta),0,
                0,sin(theta),cos(theta),0,
                0,0,0,1;
        Matrix4d rotY;
        rotY << cos(theta),0,-sin(theta),0,
                0,1,0,0,
                sin(theta),0,cos(theta),0,
                0,0,0,1;
        Matrix4d rotZ;
        rotZ << cos(theta),-sin(theta),0,0,
                sin(theta),cos(theta),0,0,
                0,0,1,0,
                0,0,0,1;
        markerOrigin = markerOrigin*rotZ*rotX;

        // publish tf
        publishMarkerTFs(markerManager.getMarkerWorldTransforms(), "marker_origin");
        publishTF(markerOrigin, "world", "marker_origin");
        if (markerManager.validPoseEstimate)
        {
            publishTF(camTf, "marker_origin", "camera");
        }


        // Display images
        cv::imshow("Binary Image", imgBin);

        cv::imshow("Original Image", img);

        // keep window open until any key is pressed
//        if(cv::waitKey(150) >= 0) break; // for recorded video
        if(cv::waitKey(1) >= 0) break; // from USB cam
    }
}

void publishTF(Matrix4d T, const char* parent, const char* node)
{
    tfScalar m00 = T(0,0); tfScalar m01 = T(0,1); tfScalar m02 = T(0,2);
    tfScalar m10 = T(1,0); tfScalar m11 = T(1,1); tfScalar m12 = T(1,2);
    tfScalar m20 = T(2,0); tfScalar m21 = T(2,1); tfScalar m22 = T(2,2);
    tf::Matrix3x3 rotMat(m00,m01,m02,
                        m10,m11,m12,
                        m20,m21,m22);
    tf::Vector3 transVec(T(0,3), T(1,3), T(2,3));

    static tf::TransformBroadcaster br;
    tf::Transform transform(rotMat, transVec);
    tf::Quaternion Q = transform.getRotation();
    Q.normalize();
    transform.setRotation(Q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent, node));
}

// publish all static world marker transforms
void publishMarkerTFs(vector<Matrix4d> markerTransforms, const char* parent)
{
    for (int i = 0; i < markerTransforms.size(); i++)
    {
        ostringstream ostr;
        ostr << "marker" << i;
        publishTF(markerTransforms[i], parent, ostr.str().c_str());
    }
}

// TODO move to search
std::list<cv::Point> readMatches(cv::Mat img, Search s, std::list<cv::Point> matches, int matchIndex, bool horz)
{
    // Read all matches from OpenCL kernel
    if (matchIndex > 0)
    {
        unsigned int* newMatchesPointer = s.readMatchesOutput(matchIndex);

        // loop through matches
        for (int i = 0; i < matchIndex; i++)
        {
            cv::Point match;
            // need to know if the kernel ran horizontally or vertically because x and y are flipped
            if (horz)
            {
                match = (cv::Point){newMatchesPointer[2*i], newMatchesPointer[2*i+1]};
            }
            else //vertical
            {
                match = (cv::Point){newMatchesPointer[2*i+1], newMatchesPointer[2*i]};
            }
            matches.push_front(match);

            // Color a point at each match
            cv::circle(img, matches.front(), 1, cv::Scalar(0,255,0), -1);
        }
    }
//    cv::circle(img, matches.front(), 15, cv::Scalar(0,255,0), 1);
    return matches;
}

// TODO move to search
std::list<cv::Point> averageMatches(std::list<cv::Point> matches)
{
    // Creates a list to store all averaged matches
    std::list< cv::Point > avgMatches;
    while (!matches.empty())
    {
        int xsum = 0;
        int ysum = 0;

        // get current cluster and remove first corrdinate from list
        cv::Point cluster = matches.front();
        matches.pop_front();

        int i = 0;
        int count = 0;
        int radius = 20;

        // Compare all remaining matches and if they are close to the current match then they are in the same cluster
        while (i < matches.size())
        {
            cv::Point match = matches.front();
            if (abs(match.x - cluster.x) < radius && abs(match.y - cluster.y) < radius)
            {
                matches.pop_front();
                xsum+= match.x;
                ysum+= match.y;
                i--;
                count++;
            }
            i++;
        }

        // only count matches if there are several in a cluster
        int minClusterSize = 12;
//        std::cout << "count" << count << std::endl;
        if (count > minClusterSize)
        {
            cv::Point avgMatch (xsum/count, ysum/count);
            avgMatches.push_front(avgMatch);
        }
    }
    return avgMatches;
}

//cv::Mat drawTargets(cv::Mat img, std::vector<HoldPoint> H, cv::Scalar color)
//{
//    // Draw red taget over averaged matches
//    for (std::vector<HoldPoint>::iterator it = H.begin(); it != H.end(); it++)
//    {
//        int l = 10; //radius of cross
//        cv::Point center = it->heldMatch;

//        cv::line(img, (cv::Point){center.x-l,center.y}, (cv::Point){center.x+l,center.y}, color, 2);
//        cv::line(img, (cv::Point){center.x,center.y-l}, (cv::Point){center.x,center.y+l}, color, 2);
//    }
//    return img;
//}

// Not Used
//void comb(int N, int K)
//{
//    std::string bitmask(K, 1); // K leading 1's
//    bitmask.resize(N, 0); // N-K trailing 0's

//    // print integers and permute bitmask
//    do {
//        for (int i = 0; i < N; ++i) // [0..N-1] integers
//        {
//            if (bitmask[i]) std::cout << " " << i;
//        }
//        std::cout << std::endl;
//    } while (std::prev_permutation(bitmask.begin(), bitmask.end()));
//}


