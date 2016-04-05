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

#ifndef DETECT_H_
#define DETECT_H_

#include <vector>
#include <opencv2/opencv.hpp>

#include "config.h"
#include "sensors_data.h"
#include "car.h"
#include "bgtracking.h"

#define HORIZONTAL_SEARCH 0
#define VERTICAL_SEARCH 1

double CalcVariance(cv::Mat&);

cv::Mat getHOGDescriptors(cv::Mat&, Config*);

int max(int, int);

int min(int, int);

struct IdxCompare
{
    const std::vector<int>& target;
    IdxCompare(const std::vector<int>& target): target(target) {}
    bool operator()(int a, int b) const { return target[a] < target[b]; }
};

std::vector<int> argsort(std::vector<int>);

std::vector<cv::Rect> NonMaximaSupression(std::vector<cv::Rect>,
										  float overlapThresh=0.4);

void DetectCars(cv::Mat&, cv::Ptr<cv::ml::SVM>, Config&, SensorsData&,
                std::vector<Car>&, std::vector<cv::Rect>&,
                int mode=HORIZONTAL_SEARCH);

std::vector<cv::Point2f> DetectKeyPoints(cv::Mat&, Config&);

void ClassifingKeyPoints(std::vector<cv::Point2f>&, std::vector<Car>&);

#endif
