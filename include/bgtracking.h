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

#ifndef BG_TRACKING_H_
#define BG_TRACKING_H_

#include <vector>
#include <opencv2/opencv.hpp>

#include "config.h"

class BGTracking
{
    private:
        std::vector<cv::Point2f> prev_bg_keypoints;
        double modulus;
        double angle;
        Config *config;
        int counter;
    public:
        BGTracking(std::vector<cv::Point2f>, Config&);
        ~BGTracking();
        void setKeyPoints(std::vector<cv::Point2f>);
        std::vector<cv::Point2f> getKeyPoints();
        double getModulus();
        double getAngle();
        bool Update(cv::Mat&, cv::Mat&, cv::Mat&, bool);
};

#endif
