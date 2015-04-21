#include "holdpoint.h"

HoldPoint::HoldPoint()
{
    count = 0;
    timeout = 10;
    checked = false;
    radius = 50;
}

void HoldPoint::update(cv::Point avgMatch)
{
    heldMatch = (cv::Point)NULL;
    if (avgMatch != (cv::Point)NULL)
    {
        prevPoint = avgMatch;
        heldMatch = avgMatch;
        count = 0;
    }
    else if (count < timeout)
    {
        count++;
        heldMatch = prevPoint;
    }
}

std::vector<HoldPoint> HoldPoint::holdPoints(std::vector<HoldPoint> H, std::list<cv::Point> avgMatches)
{
    while (avgMatches.size() > 0)
    {
        bool matched = false;
//        int radius = 30;
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
//            int radius = 10;
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


HoldPoint::~HoldPoint()
{
}

