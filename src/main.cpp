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

#include "search.h"
#include "holdpoint.h"
#include "marker.h"
#include "barcode.h"
#include "combinations.h"
#include "/opt/ros/groovy/include/opencv2/video/tracking.hpp"

// ROS includes
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>

// declare local methods
std::list<cv::Point> readMatches(Search s, std::list<cv::Point> matches, int matchIndex, bool horz);
std::list<cv::Point> averageMatches(std::list<cv::Point> matches);
cv::Mat drawTargets(cv::Mat img, std::vector<HoldPoint> H, cv::Scalar color);
void poseEstimation(cv::Mat imgBin, cv::Mat rvec, cv::Mat tvec, int w, int h, cv::Mat cameraMatrix, cv::Mat distCoeffs, Marker marker, Barcode barcode, Combinations comb);
void publishCameraTF(cv::Mat rMat, cv::Mat tvec);
void publishMarkerTF();
void comb(int N, int K);

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

    int newFrame = 0;
    cv::Mat img;
    cap >> img;

    int w = img.cols;
    int h = img.rows;

    Marker marker1;
    Marker marker2;

    Barcode barcode;
    barcode.setCameraParmeters(cameraMatrix, distCoeffs, w, h);

    Combinations comb;

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
        std::list< cv::Point > matches;
        matches = readMatches(s1, matches, matchIndex1, true);
        matches = readMatches(s2, matches, matchIndex2, false);

//        std::cout << "Number of matches: " << matches.size() <<std::endl;

        // Average clusters
        std::list<cv::Point> avgMatches = averageMatches(matches);

        H = hold.holdPoints(H, avgMatches);

        // put all holdpoints into an array
        int numMatches = H.size();
//        std::cout << "numMatches: " << numMatches << std::endl;
        HoldPoint holdPointsArray [numMatches];
        int k = 0;
//        std::cout << "targets" << std::endl;
        for (std::vector<HoldPoint>::iterator it = H.begin(); it != H.end(); it++)
        {
            holdPointsArray[k] = *it;
//            std::cout << holdPointsArray[k].heldMatch << std::endl;
            k++;
        }

        // find the distances from the first target to every other target
        int dist [numMatches]; // stores the distance from the first target to every other target
        cv::Point start = holdPointsArray[0].heldMatch;
//        std::cout << "distances" << std::endl;
        for (int i = 0; i < numMatches; i++)
        {
            cv::Point end =  holdPointsArray[i].heldMatch;
            dist[i] = sqrt(pow(end.x - start.x, 2) + pow(end.y - start.y, 2));
//            std::cout << dist[i] << std::endl;
        }

        // loop through and find the shortest distance 3 times
        std::vector<HoldPoint> H1;
        H1.push_back(holdPointsArray[0]);
//        std::cout << "shortest" << std::endl;
        for (int i = 0; i < 3; i++)
        {
            int min = INT_MAX;
            int indexOfMin;
            for (int j = 1; j < numMatches; j++) // exclude the distance from the first element to the first element because it is zero
            {
                if (dist[j] < min && dist[j] > 20) // > 20 to avoid duplicate points
                {
//                    std::cout << dist[j] <<std::endl;
                    min = dist[j];
                    indexOfMin = j;
                }
            }
//            std::cout << dist[indexOfMin] << std::endl;
            H1.push_back(holdPointsArray[indexOfMin]);
            dist[indexOfMin] = INT_MAX; // Make sure the same min value is not used again
        }
        dist[0] = INT_MAX;

        // find next target
        std::vector<HoldPoint> H2;
        if (numMatches >= 8)
        {
            // create new array with all elements with distance not set to INT_MAX
            HoldPoint holdPointsArray2 [numMatches-4];
            int dist2 [numMatches-4];
            int g = 0;
//            std::cout << "reused targets " << std::endl;
            for (int i = 0; i < numMatches; i++)
            {
                if (dist[i] < INT_MAX)
                {
                    holdPointsArray2[g] = holdPointsArray[i];
                    dist2[g] = dist[i];
//                    std::cout << holdPointsArray2[g].heldMatch << std::endl;
                    g++;
                }
            }

            // loop through and find the shortest distance 3 times

            H2.push_back(holdPointsArray2[0]);
    //        std::cout << "shortest" << std::endl;
            for (int i = 0; i < 3; i++)
            {
                int min = INT_MAX;
                int indexOfMin;
                for (int j = 1; j < numMatches; j++) // exclude the distance from the first element to the first element because it is zero
                {
                    if (dist2[j] < min && dist2[j] > 20) // > 20 to avoid duplicate points
                    {
    //                    std::cout << dist[j] <<std::endl;
                        min = dist2[j];
                        indexOfMin = j;
                    }
                }
    //            std::cout << dist[indexOfMin] << std::endl;
                H2.push_back(holdPointsArray2[indexOfMin]);
                dist2[indexOfMin] = INT_MAX; // Make sure the same min value is not used again
            }
            dist2[0] = INT_MAX;
        }
        // Draw targets over averaged matches
        img = drawTargets(img, H1, cv::Scalar(0,0,255));
        img = drawTargets(img, H2, cv::Scalar(0,255,0));

        std::vector<HoldPoint> H1sorted = marker1.sortPointsVertically(H1);
        std::vector<HoldPoint> H2sorted = marker2.sortPointsVertically(H2);

        marker1.foundMarkers = H1.size();
        marker1.setImageCoord(H1sorted);
        marker2.foundMarkers = H2.size();
        marker2.setImageCoord(H2sorted);

        marker1.setWorldCoord();
        marker2.setWorldCoord();

        // reset rvec and tvec
        rvec.at<double>(0) = 0;
        rvec.at<double>(1) = 0;
        rvec.at<double>(2) = 0;

        tvec.at<double>(0) = 0;
        tvec.at<double>(1) = 0;
        tvec.at<double>(2) = 0;

        cv::Mat marker1WorldTransform = cv::Mat(4,4,cv::DataType<double>::type);
        marker1WorldTransform.at<double>(0,0) = 1.0; marker1WorldTransform.at<double>(0,1) = 0.0; marker1WorldTransform.at<double>(0,2) = 0.0; marker1WorldTransform.at<double>(0,3) = 0.0;
        marker1WorldTransform.at<double>(1,0) = 0.0; marker1WorldTransform.at<double>(1,1) = 1.0; marker1WorldTransform.at<double>(1,2) = 0.0; marker1WorldTransform.at<double>(1,3) = 0.0;
        marker1WorldTransform.at<double>(2,0) = 0.0; marker1WorldTransform.at<double>(2,1) = 0.0; marker1WorldTransform.at<double>(2,2) = 1.0; marker1WorldTransform.at<double>(2,3) = 0.0;
        marker1WorldTransform.at<double>(3,0) = 0.0; marker1WorldTransform.at<double>(3,1) = 0.0; marker1WorldTransform.at<double>(3,2) = 0.0; marker1WorldTransform.at<double>(3,3) = 1.0;

        marker1.setWorldTransform(marker1WorldTransform);

        cv::Mat marker2WorldTransform = cv::Mat(4,4,cv::DataType<double>::type);
        double theta = PI/2;
        marker2WorldTransform.at<double>(0,0) = cos(theta); marker2WorldTransform.at<double>(0,1) = 0.0; marker2WorldTransform.at<double>(0,2) = -sin(theta); marker2WorldTransform.at<double>(0,3) = 0.10795;
        marker2WorldTransform.at<double>(1,0) = 0.0;        marker2WorldTransform.at<double>(1,1) = 1.0; marker2WorldTransform.at<double>(1,2) = 0.0;         marker2WorldTransform.at<double>(1,3) = 0.0;
        marker2WorldTransform.at<double>(2,0) = sin(theta); marker2WorldTransform.at<double>(2,1) = 0.0; marker2WorldTransform.at<double>(2,2) = cos(theta);  marker2WorldTransform.at<double>(2,3) = 0.3556;
        marker2WorldTransform.at<double>(3,0) = 0.0;        marker2WorldTransform.at<double>(3,1) = 0.0; marker2WorldTransform.at<double>(3,2) = 0.0;         marker2WorldTransform.at<double>(3,3) = 1.0;

        marker2.setWorldTransform(marker2WorldTransform);

        if (marker1.enoughMarkers)
        {
            // estimate pose
            marker1.poseEstimation(imgBin, w, h, barcode);

            if (!marker1.markerTransformationZero())
            {
//                marker1.averageVec(); // average later

                img = marker1.projectAxis(img, barcode);

//                img = marker1.projectTransformAxis(img, barcode, marker2WorldTransform);

                img = marker1.projectBarcodeGrid(img, barcode);
            }
        }

        if (marker2.enoughMarkers)
        {
            // estimate pose
            marker2.poseEstimation(imgBin, w, h, barcode);

            if (!marker2.markerTransformationZero())
            {
//                marker2.averageVec();

                img = marker2.projectAxis(img, barcode);

                img = marker2.projectBarcodeGrid(img, barcode);
            }
        }

        // if rvec and tvec != 0
        if (!marker1.markerTransformationZero())
        {
            cv::Mat totalImgCoord;
            cv::Mat totalWorldCoord;
            if (!marker2.markerTransformationZero())
            {
//                std::cout << "found both markers" << std::endl;

                // Define all image coordinates
                totalImgCoord = cv::Mat(8,1,cv::DataType<cv::Point2f>::type); // 2 markers
                cv::Mat imgCoord1 = cv::Mat(4,1,cv::DataType<cv::Point2f>::type);
                cv::Mat imgCoord2 = cv::Mat(4,1,cv::DataType<cv::Point2f>::type);
                imgCoord1 = marker1.getImageCoord(marker1.imgCoordOrientation);
                imgCoord2 = marker2.getImageCoord(marker2.imgCoordOrientation);
                totalImgCoord.at<cv::Point2f>(0) = imgCoord1.at<cv::Point2f>(0);
                totalImgCoord.at<cv::Point2f>(1) = imgCoord1.at<cv::Point2f>(1);
                totalImgCoord.at<cv::Point2f>(2) = imgCoord1.at<cv::Point2f>(2);
                totalImgCoord.at<cv::Point2f>(3) = imgCoord1.at<cv::Point2f>(3);
                totalImgCoord.at<cv::Point2f>(4) = imgCoord2.at<cv::Point2f>(0);
                totalImgCoord.at<cv::Point2f>(5) = imgCoord2.at<cv::Point2f>(1);
                totalImgCoord.at<cv::Point2f>(6) = imgCoord2.at<cv::Point2f>(2);
                totalImgCoord.at<cv::Point2f>(7) = imgCoord2.at<cv::Point2f>(3);

                // Define all world coordinates
                totalWorldCoord = cv::Mat(8,1,cv::DataType<cv::Point3f>::type); // 2 markers
                cv::Mat worldCoord1 = cv::Mat(4,1,cv::DataType<cv::Point2f>::type);
                cv::Mat worldCoord2 = cv::Mat(4,1,cv::DataType<cv::Point2f>::type);
                cv::Mat newWorldCoord2 = cv::Mat(4,1,cv::DataType<cv::Point2f>::type);
                worldCoord1 = marker1.getWorldCoord();
                worldCoord2 = marker2.getWorldCoord();

                // transform world coordinates for marker 2
                for (int i = 0; i < 4; i++)
                {
                    cv::Mat worldPoint = cv::Mat(4,1,cv::DataType<double>::type);
                    worldPoint.at<double>(0) = (double)worldCoord2.at<cv::Point3f>(i).x;
                    worldPoint.at<double>(1) = (double)worldCoord2.at<cv::Point3f>(i).y;
                    worldPoint.at<double>(2) = (double)worldCoord2.at<cv::Point3f>(i).z;
                    worldPoint.at<double>(3) = 1;

//                    std::cout << "WorldPoint: " << worldPoint << std::endl;
//                    std::cout << "transform: " << marker2.getWorldTransform() << std::endl;

                    worldPoint = marker2.getWorldTransform() * worldPoint;
//                    std::cout << "worldPoint: " << worldPoint << std::endl;

                    newWorldCoord2.at<cv::Point3f>(i).x = (float)worldPoint.at<double>(0);
                    newWorldCoord2.at<cv::Point3f>(i).y = (float)worldPoint.at<double>(1);
                    newWorldCoord2.at<cv::Point3f>(i).z = (float)worldPoint.at<double>(2);
                }

//                std::cout << "worldCoord2: " << newWorldCoord2 << std::endl;

                totalWorldCoord.at<cv::Point3f>(0) = worldCoord1.at<cv::Point3f>(0);
                totalWorldCoord.at<cv::Point3f>(1) = worldCoord1.at<cv::Point3f>(1);
                totalWorldCoord.at<cv::Point3f>(2) = worldCoord1.at<cv::Point3f>(2);
                totalWorldCoord.at<cv::Point3f>(3) = worldCoord1.at<cv::Point3f>(3);
                totalWorldCoord.at<cv::Point3f>(4) = newWorldCoord2.at<cv::Point3f>(0);
                totalWorldCoord.at<cv::Point3f>(5) = newWorldCoord2.at<cv::Point3f>(1);
                totalWorldCoord.at<cv::Point3f>(6) = newWorldCoord2.at<cv::Point3f>(2);
                totalWorldCoord.at<cv::Point3f>(7) = newWorldCoord2.at<cv::Point3f>(3);

//                // draw world coord for sanity check
//                cv::Mat temp = cv::Mat(8,1,cv::DataType<cv::Point2f>::type);
//                cv::projectPoints(totalWorldCoord, marker1.rvec, marker1.tvec, barcode.cameraMatrix, barcode. distCoeffs, temp);
//                for (int i = 0; i < 8; i++)
//                {
//                    cv::circle(img, temp.at<cv::Point2f>(i), 3, cv::Scalar(255,255,0), -1);
//                }
            }
            else
            {
                totalImgCoord = cv::Mat(4,1,cv::DataType<cv::Point2f>::type); // 1 marker
                cv::Mat imgCoord1 = cv::Mat(4,1,cv::DataType<cv::Point2f>::type);
                imgCoord1 = marker1.getImageCoord(marker1.imgCoordOrientation);
                totalImgCoord.at<cv::Point2f>(0) = imgCoord1.at<cv::Point2f>(0);
                totalImgCoord.at<cv::Point2f>(1) = imgCoord1.at<cv::Point2f>(1);
                totalImgCoord.at<cv::Point2f>(2) = imgCoord1.at<cv::Point2f>(2);
                totalImgCoord.at<cv::Point2f>(3) = imgCoord1.at<cv::Point2f>(3);

                totalWorldCoord = cv::Mat(4,1,cv::DataType<cv::Point3f>::type); // 1 marker
                cv::Mat worldCoord1 = cv::Mat(4,1,cv::DataType<cv::Point2f>::type);
                worldCoord1 = marker1.getWorldCoord();
                totalWorldCoord.at<cv::Point3f>(0) = worldCoord1.at<cv::Point3f>(0);
                totalWorldCoord.at<cv::Point3f>(1) = worldCoord1.at<cv::Point3f>(1);
                totalWorldCoord.at<cv::Point3f>(2) = worldCoord1.at<cv::Point3f>(2);
                totalWorldCoord.at<cv::Point3f>(3) = worldCoord1.at<cv::Point3f>(3);
            }

            int flags = cv::ITERATIVE;
            bool useExtrinsicGuess = false;
            std::cout << totalWorldCoord << std::endl;
//            rvec = marker1.rvec;
//            tvec = marker1.tvec;
            cv::solvePnP(totalWorldCoord, totalImgCoord, barcode.cameraMatrix, barcode.distCoeffs, rvec, tvec, useExtrinsicGuess, flags);


            // project axis
            cv::Mat axis = cv::Mat(4,1,cv::DataType<cv::Point3f>::type); // 1 marker
            std::vector<cv::Point2f> projectedAxis;
            axis.at<cv::Point3f>(0) = (cv::Point3f){0,0,0};
            axis.at<cv::Point3f>(1) = (cv::Point3f){0.1,0,0};
            axis.at<cv::Point3f>(2) = (cv::Point3f){0,0.1,0};
            axis.at<cv::Point3f>(3) = (cv::Point3f){0,0,0.1};

            cv::projectPoints(axis, rvec, tvec, barcode.cameraMatrix, barcode.distCoeffs, projectedAxis);

            cv::line(img, projectedAxis[0], projectedAxis[1], cv::Scalar(0,0,255), 2);
            cv::line(img, projectedAxis[0], projectedAxis[2], cv::Scalar(0,255,0), 2);
            cv::line(img, projectedAxis[0], projectedAxis[3], cv::Scalar(255,0,0), 2);


            // inverse pose estimation to get camera position
            cv::Mat rMat(3,3,cv::DataType<double>::type);
            cv::Mat rMatTrans(3,3,cv::DataType<double>::type);
            cv::Mat tvecCam(3,1,cv::DataType<double>::type);

            cv::Rodrigues(rvec, rMat);
            cv::transpose(rMat, rMatTrans);
            tvecCam = -rMatTrans * tvec;

            // publish tf
            publishMarkerTF();
            publishCameraTF(rMatTrans, tvecCam);

        }

        // Display images
        cv::imshow("Binary Image", imgBin);

        cv::imshow("Original Image", img);

        // keep window open until any key is pressed
//        if(cv::waitKey(150) >= 0) break; // for recorded video
        if(cv::waitKey(1) >= 0) break; // from USB cam
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

// TODO move to search
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
        int minClusterSize = 20;
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


