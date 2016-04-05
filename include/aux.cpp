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
#include <string>
#include <sstream>

#include "aux.h"

using namespace std;
using namespace cv;

void Setup(Config &config, Mat frame)
{
    config.sliding_window_width = 50;
    config.sliding_window_height = 25;
    config.sliding_window_horizontal_step = 6;
    config.sliding_window_vertical_step = 6;
    config.pyramid_initial_scale = 1;
    config.pyramid_final_scale = 1;
    config.pyramid_scale_step = 0.5;
    config.svm_min_hyperplane_distance = -0.75;
    config.variance_threshold = 1000;
    
    config.fast_threshold = 40;
    config.frames_to_update = 25;
    
    config.cars_rect_color = Scalar(0,255,0);
    config.cars_speed_color = Scalar(255,0,0);
    config.cars_speed_font = FONT_HERSHEY_COMPLEX;
    config.cars_speed_font_size = 0.4;
    config.cars_speed_font_color = Scalar(255,255,255);
    config.cam_speed_position = Point2f(frame.cols-200, frame.rows-25);
    config.cam_speed_font = FONT_HERSHEY_COMPLEX;
    config.cam_speed_font_size = 0.85;
    config.cam_speed_font_color = Scalar(255,255,255);
    config.cam_arrow_center = Point2f(frame.cols-40, frame.rows-35);
    config.cam_arrow_color = Scalar(255,255,255);
    config.cam_arrow_size = 30;
    
    config.of_subPixWinSize = Size(10,10);
    config.of_winSize = Size(31,31);
    config.of_termcrit = TermCriteria(TermCriteria::COUNT|TermCriteria::EPS, 20, 0.03);
    
    config.still_flight_threshold = 0.1;
    config.equal_speed_threshold = 0.75;
}

void ReadSensors(SensorsData &sensors_data)
{
    // BMP180
	sensors_data.temperature = 25;
	sensors_data.pressure = 101000;
	sensors_data.altitude = 100;
	// HMC5883
	sensors_data.mag_x = 1;
	sensors_data.mag_y = 1;
	sensors_data.mag_z = 1;
	sensors_data.heading = 0;
	// MPU6050
	sensors_data.ax = 1;
	sensors_data.ay = 1;
	sensors_data.az = 1;
	sensors_data.gx = 1;
	sensors_data.gy = 1;
	sensors_data.gz = 1;
	sensors_data.qw = 1;
	sensors_data.qx = 1;
	sensors_data.qy = 1;
	sensors_data.qz = 1;
	sensors_data.euler_yaw = 1;
	sensors_data.euler_pitch = 1;
	sensors_data.euler_roll = 1;
	sensors_data.yaw = 0;
	sensors_data.pitch = 0;
	sensors_data.roll = 0;
	// NEO-6M
	sensors_data.latitude = 0;
	sensors_data.longitude = 0;
	sensors_data.gps_altitude = 100;
	sensors_data.course = 0;
	sensors_data.speed = 15;
	sensors_data.centisecond = 0;
	sensors_data.second = 0;
	sensors_data.minute = 0;
	sensors_data.hour = 0;
	sensors_data.day = 0;
	sensors_data.month = 0;
	sensors_data.year = 0;
	sensors_data.satellites = 0;
	sensors_data.hdop = 9999;
}

Mat DrawResults(Mat image, vector<Car> &cars, BGTracking &nav, SensorsData &sensors_data, Config &config)
{
    // Draw cars bounding boxes and speed
    for(unsigned int i=0;i<cars.size();i++)
    {
        ostringstream buffer;
        if (cars[i].getSpeed() >= 100)
            buffer << trunc(cars[i].getSpeed());
        else if (cars[i].getSpeed() >= 10)
            buffer << trunc(cars[i].getSpeed()*10)/10;
        else
            buffer << trunc(cars[i].getSpeed()*100)/100;
        rectangle(image, cars[i].getRect(), config.cars_rect_color, 2);
        rectangle(image, Rect(cars[i].getRect().x, cars[i].getRect().y-15, 35, 15), Scalar(255,0,0), CV_FILLED);
        putText(image, buffer.str(),
                Point2f(cars[i].getRect().x+2, cars[i].getRect().y-4), config.cars_speed_font,
                config.cars_speed_font_size,
                config.cars_speed_font_color);
    }
    
    // Draw camera speed
    image(Rect(image.cols-210, image.rows-75, 210, 75)) = 0;
    ostringstream buffer2;
    buffer2 << sensors_data.speed;
    putText(image, buffer2.str() + " m/s", config.cam_speed_position,
            config.cam_speed_font, config.cam_speed_font_size,
            config.cam_speed_font_color);
    
    // Draw camera direction
    Point2f src = Point2f(config.cam_arrow_center.x - config.cam_arrow_size*cos(nav.getAngle()),
                          config.cam_arrow_center.y + config.cam_arrow_size*sin(nav.getAngle()));
    Point2f dst = Point2f(config.cam_arrow_center.x + config.cam_arrow_size*cos(nav.getAngle()),
                          config.cam_arrow_center.y - config.cam_arrow_size*sin(nav.getAngle()));
    arrowedLine(image, src, dst, config.cam_arrow_color);
    
    return image;
}

Mat DrawKeyPoints(Mat image, vector<Point2f> points, Scalar color)
{
    for(unsigned int i=0;i<points.size();i++)
        circle(image, points[i], 3, color, -1, 5);
    return image;
}

Mat WriteFPS(Mat image, double fps)
{
    image(Rect(0, 0, 110, 30)) = 0;
    ostringstream buffer;
    if (fps >= 10)
        buffer << trunc(fps*10)/10;
    else
        buffer << trunc(fps*100)/100;
    putText(image, buffer.str() + " fps", Point2f(5, 20), FONT_HERSHEY_COMPLEX_SMALL,
            1, Scalar(255,255,255));
    
    return image;
}

Mat DrawROIs(Mat image, vector<Rect> rois)
{
    for(unsigned int i=0;i<rois.size();i++)
        rectangle(image, rois[i], Scalar(0,0,255), 2);
    
    return image;
}

Mat DrawCarsKeyPoints(Mat image, vector<Car> cars, Scalar color)
{
    for(unsigned int i=0;i<cars.size();i++)
        for(unsigned int j=0;j<cars[i].getKeyPoints().size();j++)
        circle(image, cars[i].getKeyPoints()[j], 3, color, -1, 5);
    
    return image;
}

vector<Point2f> KeyPoint2Point2f(vector<KeyPoint> input)
{
    vector<Point2f> output;
    for(unsigned int i=0;i<input.size();i++)
        output.push_back(input[i].pt);
    
    return output;
}
