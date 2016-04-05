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

#ifndef CAR_H_
#define CAR_H_

#include <vector>
#include <opencv2/opencv.hpp>

#include "config.h"
#include "sensors_data.h"
#include "camera.h"
#include "bgtracking.h"

// Earth radius
#define R 6371000

class Car
{
    private:
        Config *config;
        SensorsData *sensors_data;
        cv::Rect bounding_box;
        cv::Point2f prev_img_pos;
        cv::Point2f img_pos;
        std::vector<cv::Point2f> prev_car_keypoints;
        std::vector<cv::Point2f> rect_points;
        cv::Ptr<cv::ml::SVM> svm;
        double modulus;
        double angle;
        double speed;
        double latitude;
        double longitude;
        double heading;
        double vx;
        double vy;
        double vz;
        unsigned int frame_counter;
    public:
        Car(cv::Rect, Config&, SensorsData&, cv::Ptr<cv::ml::SVM>);
        ~Car();
        void setKeyPoints(std::vector<cv::Point2f>);
        cv::Rect getRect();
        cv::Point2f getPosition();
        std::vector<cv::Point2f> getKeyPoints();
        std::vector<cv::Point2f> getRectPoints();
        double getSpeed();
        double getLatitude();
        double getLongitude();
        bool Update(cv::Mat&, cv::Mat&, BGTracking&, cv::Mat&, bool);
        void CalcRealPos(cv::Mat&, SensorsData&, Camera&);
        bool UpdateDetection(cv::Mat&);
        double getModulus();
        double getAngle();
};

#endif
