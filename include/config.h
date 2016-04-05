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

#ifndef CONFIG_H_
#define CONFIG_H_

#include <opencv2/opencv.hpp>

struct Config
{
    // Detection step
    int sliding_window_width;
    int sliding_window_height;
    int sliding_window_horizontal_step;
    int sliding_window_vertical_step;
    float pyramid_initial_scale;
    float pyramid_final_scale;
    float pyramid_scale_step;
    float svm_min_hyperplane_distance;
    int variance_threshold;
    // Keypoints extraction
    int fast_threshold;
    int frames_to_update;
    // Optical flow
    cv::Size of_subPixWinSize;
    cv::Size of_winSize;
    cv::TermCriteria of_termcrit;
    // Draw step
    cv::Scalar cars_rect_color;
    cv::Scalar cars_speed_color;
    int cars_speed_font;
    double cars_speed_font_size;
    cv::Scalar cars_speed_font_color;
    cv::Point2f cam_speed_position;
    int cam_speed_font;
    double cam_speed_font_size;
    cv::Scalar cam_speed_font_color;
    cv::Point2f cam_arrow_center;
    cv::Scalar cam_arrow_color;
    int cam_arrow_size;
    // Direction detector
    float still_flight_threshold;
    float equal_speed_threshold;
};

#endif
