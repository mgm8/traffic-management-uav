/*
 * Traffic Management UAV.
 * 
 * Copyright (c) 2016 Gabriel Mariano Marcelino <gabriel.mm8@gmail.com>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>
 * 
 */

#include <cmath>

#include "bgtracking.h"

using namespace std;
using namespace cv;

BGTracking::BGTracking(vector<Point2f> kp, Config &p)
{
    prev_bg_keypoints = kp;
    config = &p;
    counter = 0;
}

BGTracking::~BGTracking()
{
    
}

void BGTracking::setKeyPoints(vector<Point2f> points)
{
    prev_bg_keypoints = points;
}

vector<Point2f> BGTracking::getKeyPoints()
{
    return prev_bg_keypoints;
}

double BGTracking::getModulus()
{
    return modulus;
}

double BGTracking::getAngle()
{
    return angle;
}

bool BGTracking::Update(Mat &prev_frame, Mat &frame, Mat &f, bool draw_of_vectors)
{
    vector<float> err;
    vector<uchar> status;
    vector<Point2f> new_bg_keypoints = prev_bg_keypoints;
    calcOpticalFlowPyrLK(prev_frame, frame, prev_bg_keypoints,
                         new_bg_keypoints, status, err,
                         config->of_winSize, 3, config->of_termcrit, 0,
                         0.001);
    
    modulus = 0;
    angle = 0;
    int counter = 0;
    for(unsigned int i=0;i<status.size();i++)
    {
        if (status[i])                                                  // Check if the optical flow between the keypoints was found
        {
            if (draw_of_vectors)
                arrowedLine(f, prev_bg_keypoints[i], new_bg_keypoints[i], Scalar(0,0,255), 2);
    
            modulus += sqrt(pow(prev_bg_keypoints[i].x - new_bg_keypoints[i].x, 2) +
                            pow(prev_bg_keypoints[i].y - new_bg_keypoints[i].y, 2));
            double ang = atan2(prev_bg_keypoints[i].y - new_bg_keypoints[i].y,
                               prev_bg_keypoints[i].x - new_bg_keypoints[i].x);
            if (ang < 0)
                ang += 2*M_PI;
            angle += ang;
            counter++;
        }
    }
    
    if (counter != 0)
    {
        modulus /= counter;
        angle /= counter;
    }
    
    prev_bg_keypoints = new_bg_keypoints;
    
    if (counter < 50)
    {
        counter = 0;
        return false;
    }
    else
    {
        counter++;
        return true;
    }
}
