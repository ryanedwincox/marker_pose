#include "kalman.h"

// ******************************
//
// Currently not used
//
// *****************************

Kalman::Kalman()
{
}

cv::KalmanFilter Kalman::createKalmanFilter(int x, int y)
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

cv::Point Kalman::runKalmanFilter(cv::KalmanFilter KF, cv::Point statePt, std::list<cv::Point> avgMatches)
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
