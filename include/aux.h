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

#ifndef AUX_H_
#define AUX_H_

#include <vector>
#include <opencv2/opencv.hpp>

#include "car.h"
#include "bgtracking.h"
#include "config.h"
#include "sensors_data.h"

void Setup(Config&, cv::Mat);

void ReadSensors(SensorsData&);

cv::Mat DrawResults(cv::Mat, std::vector<Car>&, BGTracking&, SensorsData&, Config&);

cv::Mat DrawKeyPoints(cv::Mat, std::vector<cv::Point2f>, cv::Scalar);

cv::Mat WriteFPS(cv::Mat, double);

cv::Mat DrawROIs(cv::Mat, std::vector<cv::Rect>);

cv::Mat DrawCarsKeyPoints(cv::Mat, std::vector<Car>, cv::Scalar);

std::vector<cv::Point2f> KeyPoint2Point2f(std::vector<cv::KeyPoint>);

#endif
