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

#include "search.h"
#include "holdpoint.h"
#include "markerlayout.h"
#include "barcode.h"
#include "/opt/ros/groovy/include/opencv2/video/tracking.hpp"

// ROS includes
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>

// declare local methods
std::list<cv::Point> readMatches(Search s, std::list<cv::Point> matches, int matchIndex, bool horz);
std::list<cv::Point> averageMatches(std::list<cv::Point> matches);
cv::Mat drawTargets(cv::Mat img, std::vector<HoldPoint> H, cv::Scalar color);
std::vector<HoldPoint> holdPoints(std::vector<HoldPoint> H, std::list<cv::Point> avgMatches);
cv::KalmanFilter createKalmanFilter(int x, int y);
cv::Point runKalmanFilter(cv::KalmanFilter KF, cv::Point statePt, std::list<cv::Point> avgMatches);
std::list<HoldPoint> mergesort(std::list<HoldPoint> H);
std::list<HoldPoint> merge(std::list<HoldPoint> left, std::list<HoldPoint> right);
cv::Mat poseEstimation(cv::Mat img, cv::Mat rvec, cv::Mat tvec, int w, int h, cv::Mat cameraMatrix, cv::Mat distCoeffs, cv::Mat worldCoord, cv::Mat imageCoord);
void averageVec (cv::Mat rvec, cv::Mat tvec/*, double rvecSum0, double rvecSum1, double rvecSum2, std::queue<double> rvecQueue0,
                      std::queue<double> rvecQueue1, std::queue<double> rvecQueue2*/);
void publishCameraTF(cv::Mat rMat, cv::Mat tvec);
void publishMarkerTF();

int main(int argc, char *argv[])
{
    //Create video capture object
    int cameraNum = 0;
    const char* filename = "/home/pierre/Dropbox/uh/uh1/ros_ws/marker_pose/Video/SSLBarcodePortageBay_20150411_13 59 32.avi";
    cv::VideoCapture cap(cameraNum);
    if(!cap.isOpened())  // check if we succeeded
    {
        std::cout << "camera not found" << std::endl;
        return -1;
    }
    cap.set(CV_CAP_PROP_FPS,30);

//    double fps=cap.get(CV_CAP_PROP_FPS);
//    std::cout << fps << std::endl;de

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

    int newFrame = 0;
    cv::Mat img;
    cap >> img;

    int w = img.cols;
    int h = img.rows;

    MarkerLayout marker;

    Barcode barcode;
    barcode.setCameraParmeters(cameraMatrix, distCoeffs, w, h);

    while (cap.isOpened())
    {
//        // repeat each frame once for recorded video
//        if (newFrame > 3)
//        {
//            // Get new frame
//            cap >> img;

//            newFrame = 0;
//        }
//        else
//        {
//            newFrame++;
//        }
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
        std::list< cv::Point > matches1;
        matches1 = readMatches(s1, matches1, matchIndex1, true);
        matches1 = readMatches(s2, matches1, matchIndex2, false);

        // Average clusters
        std::list<cv::Point> avgMatches1 = averageMatches(matches1);

        H = holdPoints(H, avgMatches1);

        // Draw targets over averaged matches
        img = drawTargets(img, H, cv::Scalar(0,0,255));

        std::vector<HoldPoint> Hsorted = marker.sortPointsVertically(H);

        marker.setImageCoord(Hsorted);


        // reset rvec and tvec
        rvec.at<double>(0) = 0;
        rvec.at<double>(1) = 0;
        rvec.at<double>(2) = 0;

        tvec.at<double>(0) = 0;
        tvec.at<double>(1) = 0;
        tvec.at<double>(2) = 0;

        if (marker.enoughMarkers)
        {
            // estimate pose
            // TODO get pose estimation for each orientation
            img = poseEstimation(img, rvec, tvec, w, h, cameraMatrix, distCoeffs, marker.getWorldCoord(), marker.getImageCoord(0));
        }

        averageVec (rvec, tvec);

        // if rvec and tvec != 0
        if (!(rvec.at<double>(0) == 0 && rvec.at<double>(1) == 0 && rvec.at<double>(2) == 0 &&
            tvec.at<double>(0) == 0 && tvec.at<double>(1) == 0 && tvec.at<double>(2) == 0))
        {
            // project axis
            img = barcode.projectAxis(img, rvec, tvec);
//            cv::Mat axis(4,1,cv::DataType<cv::Point3f>::type);
//            axis.at<cv::Point3f>(0) = (cv::Point3f){0,0,0};
//            axis.at<cv::Point3f>(1) = (cv::Point3f){0.1,0,0};
//            axis.at<cv::Point3f>(2) = (cv::Point3f){0,0.1,0};
//            axis.at<cv::Point3f>(3) = (cv::Point3f){0,0,0.1};

//            std::vector<cv::Point2f> projectedAxis;
//            cv::projectPoints(axis, rvec, tvec, cameraMatrix, distCoeffs, projectedAxis);

//            cv::line(img, projectedAxis[0], projectedAxis[1], cv::Scalar(0,0,255), 2);
//            cv::line(img, projectedAxis[0], projectedAxis[2], cv::Scalar(0,255,0), 2);
//            cv::line(img, projectedAxis[0], projectedAxis[3], cv::Scalar(255,0,0), 2);

            // Project barcode layout
            img = barcode.projectBarcodeGrid(img, rvec, tvec);
//            cv::Mat barcodeGrid(4,1,cv::DataType<cv::Point3f>::type);
//            barcodeGrid.at<cv::Point3f>(0) = (cv::Point3f){0.04,0.09,0};
//            barcodeGrid.at<cv::Point3f>(1) = (cv::Point3f){0.16,0.09,0};
//            barcodeGrid.at<cv::Point3f>(2) = (cv::Point3f){0.04,0.21,0};
//            barcodeGrid.at<cv::Point3f>(3) = (cv::Point3f){0.16,0.21,0};

//            std::vector<cv::Point2f> projectedGrid;
//            cv::projectPoints(barcodeGrid, rvec, tvec, cameraMatrix, distCoeffs, projectedGrid);

//            cv::line(img, projectedGrid[0], projectedGrid[1], cv::Scalar(0,0,255), 2);
//            cv::line(img, projectedGrid[1], projectedGrid[3], cv::Scalar(0,255,0), 2);
//            cv::line(img, projectedGrid[2], projectedGrid[0], cv::Scalar(255,0,0), 2);
//            cv::line(img, projectedGrid[3], projectedGrid[2], cv::Scalar(255,255,0), 2);

    //            // Project sample points
//            cv::Mat samplePoints(9,1,cv::DataType<cv::Point3f>::type);
//            samplePoints.at<cv::Point3f>(0) = (cv::Point3f){0.06,0.11,0};
//            samplePoints.at<cv::Point3f>(1) = (cv::Point3f){0.10,0.11,0};
//            samplePoints.at<cv::Point3f>(2) = (cv::Point3f){0.14,0.11,0};
//            samplePoints.at<cv::Point3f>(3) = (cv::Point3f){0.06,0.15,0};
//            samplePoints.at<cv::Point3f>(4) = (cv::Point3f){0.10,0.15,0};
//            samplePoints.at<cv::Point3f>(5) = (cv::Point3f){0.14,0.15,0};
//            samplePoints.at<cv::Point3f>(6) = (cv::Point3f){0.06,0.19,0};
//            samplePoints.at<cv::Point3f>(7) = (cv::Point3f){0.10,0.19,0};
//            samplePoints.at<cv::Point3f>(8) = (cv::Point3f){0.14,0.19,0};

//            std::vector<cv::Point2f> projectedSamplePoints;
//            cv::projectPoints(samplePoints, rvec, tvec, cameraMatrix, distCoeffs, projectedSamplePoints);

            barcode.projectSamplePoints(rvec, tvec);

    //        cv::circle(img, projectedSamplePoints[0], 2, cv::Scalar(0,0,255), -1);
    //        cv::circle(img, projectedSamplePoints[1], 2, cv::Scalar(0,0,255), -1);
    //        cv::circle(img, projectedSamplePoints[2], 2, cv::Scalar(0,0,255), -1);
    //        cv::circle(img, projectedSamplePoints[3], 2, cv::Scalar(0,0,255), -1);
    //        cv::circle(img, projectedSamplePoints[4], 2, cv::Scalar(0,0,255), -1);
    //        cv::circle(img, projectedSamplePoints[5], 2, cv::Scalar(0,0,255), -1);
    //        cv::circle(img, projectedSamplePoints[6], 2, cv::Scalar(0,0,255), -1);
    //        cv::circle(img, projectedSamplePoints[7], 2, cv::Scalar(0,0,255), -1);
    //        cv::circle(img, projectedSamplePoints[8], 2, cv::Scalar(0,0,255), -1);

            // Get barcode value
            int foundMarker = barcode.getMarkerNumber(imgBin);

            std::cout << foundMarker << std::endl;
//            int barcode [9];
//            for (int i = 0; i < 8; i++)
//            {
//                barcode[i] = getSectionValue(imgBin, projectedSamplePoints[i], w, h);
//    //            std::cout << barcode[i] << std::endl;
//            }

//            // Compare barcodes
//            int barcode1 [9] = {0,  0,  0,
//                                255,0,  0,
//                                0,  0,  0};

//            bool code1 = true;
//            for (int i = 0; i < 8; i++)
//            {
//                if (barcode[i] != barcode1[i])
//                {
//                    code1 = false;
//                }
//            }

//            if (code1)
//            {
//                std::cout << "barcode #1" << std::endl;
//            }
//            else
//            {
//                std::cout << "no barcode found" << std::endl;
//            }



            // inverse pose estimation to get camera position
            cv::Mat rMat(3,3,cv::DataType<double>::type);
            cv::Mat rMatTrans(3,3,cv::DataType<double>::type);
            cv::Mat tvecCam(3,1,cv::DataType<double>::type);

            cv::Rodrigues(rvec, rMat);
            cv::transpose(rMat, rMatTrans);
            tvecCam = -rMatTrans * tvec;

            // publish tf if a pose estimation is possible
            if (H.size() >= 4)
            {
                publishMarkerTF();
                publishCameraTF(rMatTrans, tvecCam);
            }
        }

        // Display images
        cv::imshow("Binary Image", imgBin);

        cv::imshow("Original Image", img);

        // keep window open until any key is pressed
//        cv::waitKey(1);
        if(cv::waitKey(1) >= 0) break;
//        break;

    }
}

void publishCameraTF(cv::Mat rMat, cv::Mat tvec)
{
    tfScalar m00 = rMat.at<double>(0,0); tfScalar m01 = rMat.at<double>(0,1); tfScalar m02 = rMat.at<double>(0,2);
    tfScalar m10 = rMat.at<double>(1,0); tfScalar m11 = rMat.at<double>(1,1); tfScalar m12 = rMat.at<double>(1,2);
    tfScalar m20 = rMat.at<double>(2,0); tfScalar m21 = rMat.at<double>(2,1); tfScalar m22 = rMat.at<double>(2,2);
    tf::Matrix3x3 rotMat(m00,m01,m02,
                        m10,m11,m12,
                        m20,m21,m22);

    static tf::TransformBroadcaster br;
    tf::Transform transform(rotMat, tf::Vector3(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)));
//    transform.setOrigin(tf::Vector3(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)));
//    tf::Quaternion q;
//    q.setRPY(rvec.at<double>(0), -rvec.at<double>(2), rvec.at<double>(1)); // GBR
//    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "marker", "camera"));
}
void publishMarkerTF()
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0,1,0));
    tf::Quaternion q;
    q.setRPY(3.1415/2, 0, 3.1415/2);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "marker"));
}

void averageVec (cv::Mat rvec, cv::Mat tvec/*, double rvecSum0, double rvecSum1, double rvecSum2, double tvecSum0, double tvecSum1, double tvecSum2, std::queue<double> rvecQueue0,
                      std::queue<double> rvecQueue1, std::queue<double> rvecQueue2, std::queue<double> tvecQueue0, std::queue<double> tvecQueue1, std::queue<double> tvecQueue2*/)
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

//    std::cout << "rvec" << std::endl;
//    std::cout << rvec << std::endl;
//    std::cout << "tvec" << std::endl;
//    std::cout << tvec << std::endl;

//    std::cout << "queue size: " << rvecQueue0.size() << std::endl;

    if (rvecQueue0.size() >= 5)
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

// POSE ESTIMATION
cv::Mat poseEstimation(cv::Mat img, cv::Mat rvec, cv::Mat tvec, int w, int h, cv::Mat cameraMatrix, cv::Mat distCoeffs, cv::Mat worldCoord, cv::Mat imageCoord)
{
    // Define marker points in world coordinates (3D)
    int numMarkers = 4;
//    cv::Mat worldCoord(numMarkers,1,cv::DataType<cv::Point3f>::type);

////        int perm[] = {0,1,2,3};

////        do
////        {

//        // quad markers
//        worldCoord.at<cv::Point3f>(0) = (cv::Point3f){0,0,0};
//        worldCoord.at<cv::Point3f>(1) = (cv::Point3f){0.2,0,0};
//        worldCoord.at<cv::Point3f>(2) = (cv::Point3f){0,0.3,0};
//        worldCoord.at<cv::Point3f>(3) = (cv::Point3f){0.2,0.3,0};


//        // Will store  markers found by camera (2D) ordered from bottom to top
//        cv::Mat imageCoord(numMarkers,1,cv::DataType<cv::Point2f>::type);

//        // copy H vector into a list for ordering
//        std::list<HoldPoint> HList;
//        int jj = 0;
//        for (std::vector<HoldPoint>::iterator it = H.begin(); it != H.end(); it++)
//        {
//            if (jj < numMarkers)
//            {
//                HList.push_back(*it);
//                imageCoord.at<cv::Point2f>(jj) = it->heldMatch;
//                jj++;
//            }
//        }

//        // sort holdpoints
//        std::list<HoldPoint> sorted = mergesort(HList);

//        // print ordered matches
////        std::cout << "Ordered Matches" << std::endl;
//        int j = numMarkers;
//        for (std::list<HoldPoint>::iterator it = sorted.begin(); it != sorted.end(); it++)
//        {
//            j--;
////            std::cout << it->heldMatch << std::endl;
//            if (j >= 0)
//            {
//                imageCoord.at<cv::Point2f>(j) = it->heldMatch;
//                if (j == 0) cv::circle(img, it->heldMatch, 3, cv::Scalar(0,255,0), -1);
//                if (j == 1) cv::circle(img, it->heldMatch, 3, cv::Scalar(255,255,0), -1);
//                if (j == 2) cv::circle(img, it->heldMatch, 3, cv::Scalar(0,0,0), -1);
//                if (j == 3) cv::circle(img, it->heldMatch, 3, cv::Scalar(255,0,0), -1);
////                j++;
//            }
//        }

        // if there are 4 valid markers try to estimate position
//        if (H.size() >= numMarkers)
//        {
            int flags = cv::ITERATIVE;
            bool useExtrinsicGuess = false;

            cv::solvePnP(worldCoord, imageCoord, cameraMatrix, distCoeffs, rvec, tvec, useExtrinsicGuess, flags);



//        }
//        } while ( std::next_permutation(perm,perm+numMarkers) );



    return img;
}



std::list<cv::Point> readMatches(Search s, std::list<cv::Point> matches, int matchIndex, bool horz)
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

//                // Color a point at each match
//                cv::circle(img, matches.front(), 3, cv::Scalar(0,255,0), -1);
        }
    }
    return matches;
}

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
        int radius = 40;

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
        int minClusterSize = 25;
        if (count > minClusterSize)
        {
            cv::Point avgMatch (xsum/count, ysum/count);
            avgMatches.push_front(avgMatch);
        }
    }
    return avgMatches;
}

cv::Mat drawTargets(cv::Mat img, std::vector<HoldPoint> H, cv::Scalar color)
{
    // Draw red taget over averaged matches
    for (std::vector<HoldPoint>::iterator it = H.begin(); it != H.end(); it++)
    {
        int l = 10; //radius of cross
        cv::Point center = it->heldMatch;

        cv::line(img, (cv::Point){center.x-l,center.y}, (cv::Point){center.x+l,center.y}, color, 2);
        cv::line(img, (cv::Point){center.x,center.y-l}, (cv::Point){center.x,center.y+l}, color, 2);
    }
    return img;
}

std::vector<HoldPoint> holdPoints(std::vector<HoldPoint> H, std::list<cv::Point> avgMatches)
{
    while (avgMatches.size() > 0)
    {
        bool matched = false;
        int radius = 30;
        // loops through all current matches
        for (std::vector<HoldPoint>::iterator it = H.begin(); it != H.end(); it++)
        {
            // update hold point if it is near a new match
            if (abs(avgMatches.front().x - it->prevPoint.x) < radius && abs(avgMatches.front().y - it->prevPoint.y) < radius)
            {
                it->update(avgMatches.front());
                matched = true;
                it->checked = true;
            }
        }

        // create new HoldPoint object if a avgMatch does not match any already existing
        if (!matched)
        {
            HoldPoint h;
            h.update(avgMatches.front());
            H.push_back(h);
        }
        avgMatches.pop_front();
    }

    // loop through all current matches again to delete matches that have timed out
    for (std::vector<HoldPoint>::iterator it = H.begin(); it != H.end(); it++)
    {
        if (it->heldMatch == (cv::Point)NULL)
        {
            H.erase(it);
            break; // because iteration length has now changed
        }

        // calls update on all holdPoints that didn't match any new matches
        if (!it->checked)
        {
            it->update((cv::Point)NULL);
        }
        else // make sure all holdPoints start the next loop with checked as false
        {
            it->checked = false;
        }
    }

    // check for duplicates and delete them
    for (std::vector<HoldPoint>::iterator it = H.begin(); it != H.end(); it++)
    {
        for (std::vector<HoldPoint>::iterator jt = it + 1; jt != H.end(); jt++)
        {
            int radius = 10;
            // delete one hold point if two are near to each other
            if (abs(it->prevPoint.x - jt->prevPoint.x) < radius && abs(it->prevPoint.y - jt->prevPoint.y) < radius)
            {
                H.erase(jt);
                goto getout; // because iteration length has changed
            }
        }
    }
    getout:

    return H;
}

cv::KalmanFilter createKalmanFilter(int x, int y)
{
    // Create kalman filter
    cv::KalmanFilter KF(4, 2, 0);
    KF.transitionMatrix = *(cv::Mat_<float>(4, 4) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1);
//    cv::Mat_<float> measurement(2,1); measurement.setTo(cv::Scalar(0));

    // Initialize kalman filter
    KF.statePre.at<float>(0) = x;
    KF.statePre.at<float>(1) = y;
    KF.statePre.at<float>(2) = 0;
    KF.statePre.at<float>(3) = 0;
    setIdentity(KF.measurementMatrix);
    setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-4));  // lower values mean more prediction
    setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-3));  // lower values tighten on found points
    setIdentity(KF.errorCovPost, cv::Scalar::all(.1));

    return KF;
}

cv::Point runKalmanFilter(cv::KalmanFilter KF, cv::Point statePt, std::list<cv::Point> avgMatches)
{
    // Run Kalman filter

    // numPredictions counts the number of frames since the last positive match
    static int numPredictions = 0;
    // maxNumPredictions is the number of frames the filter will guess a position since the last positive match
    static int maxNumPredictions = 5;

    // Get target location if a match was found
    cv::Point center;
    if (avgMatches.size() > 0)
    {
        int radius = 40;

        std::cout << "matches found" << std::endl;

        // loop through all average matches to see if any are close to previous match
        std::list< cv::Point > avgMatchesCopy = avgMatches;
        for (int i = 0; i < avgMatchesCopy.size(); i++)
        {
//                std::cout << "search" << std::endl;
//                std::cout << statePt << std::endl;
            // find if close to previous match
            if (abs(avgMatchesCopy.front().x - statePt.x) < radius && abs(avgMatchesCopy.front().y - statePt.y) < radius)
            {
                // it is close to one of the previous matches so use that one
                // push it on front of avgMatches so it will be used first
                // this means there is an extra duplicate match is avgMatches
                avgMatches.push_front(avgMatchesCopy.front());
//                        std::cout << "found" << std::endl;
                break;
            }

            // check all matches
            avgMatchesCopy.pop_front();
        }

        // current match is close to previous match
        if (abs(avgMatches.front().x - statePt.x) < radius && abs(avgMatches.front().y - statePt.y) < radius)
        {
            center = avgMatches.front();
            avgMatches.pop_front();
            numPredictions = 0;
            std::cout << "case 1" << std::endl;
        }
//            There was no match close to previous match, but the predition is close so use that
        else if (abs(KF.statePre.at<float>(0) - statePt.x) < radius && abs(KF.statePre.at<float>(1) - statePt.y) < radius && numPredictions < maxNumPredictions )// && KF.statePre.at<float>(0) != 0)
        {
            center = (cv::Point){(int)KF.statePre.at<float>(0),(int)KF.statePre.at<float>(1)};
            std::cout << statePt << std::endl;
            avgMatches.pop_front();
            numPredictions++;
            std::cout << "case 2" << std::endl;
        }
        // no current matches are close to previous match so create new filter
        else// (abs(avgMatches.front().x - statePt.x) > radius && abs(avgMatches.front().y - statePt.y) > radius && !(numPredictions < maxNumPredictions))
        {
            KF = createKalmanFilter(avgMatches.front().x, avgMatches.front().y);
            center = avgMatches.front();
            avgMatches.pop_front();
            numPredictions = 0;
            std::cout << "case 3" << std::endl;
            std::cout << "new filter***********" << std::endl;
        }

//        // If a match was found draw it
//        int l = 10; //radius of cross
//        cv::line(img, (cv::Point){center.x-l,center.y}, (cv::Point){center.x+l,center.y}, cv::Scalar(0,0,255), 2);
//        cv::line(img, (cv::Point){center.x,center.y-l}, (cv::Point){center.x,center.y+l}, cv::Scalar(0,0,255), 2);
    }
    // No match found
    // Predict position based on last prediction, don't do this more than maxNumPredictions times
    else if (numPredictions < maxNumPredictions)
    {
        center = (cv::Point){(int)KF.statePre.at<float>(0),(int)KF.statePre.at<float>(1)};
        numPredictions++;
    }
    // marker position not known
    else
    {
        statePt =  (cv::Point){-1,-1};
    }

    // Either a match was found or we still want to make a prediction based on the previous prediction up to maxNumPredictions times
    if (avgMatches.size() > 0 || numPredictions < maxNumPredictions)
    {
        // First predict, to update the internal statePre variable
        cv::Mat prediction = KF.predict();
        cv::Point predictPt(prediction.at<float>(0),prediction.at<float>(1));

        cv::Mat_<float> measurement(2,1); measurement.setTo(cv::Scalar(0));
        measurement(0) = center.x;
        measurement(1) = center.y;

        // The "correct" phase that is going to use the predicted value and our measurement
        cv::Mat estimated = KF.correct(measurement);
        statePt = (cv::Point){(int)estimated.at<float>(0),(int)estimated.at<float>(1)};
    }
    return statePt;
}
