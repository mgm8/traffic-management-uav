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

#include "car.h"
#include "detect.h"

using namespace std;
using namespace cv;
using namespace cv::ml;

Car::Car(Rect r, Config &p, SensorsData &sd, Ptr<SVM> s)
{
    bounding_box = r;
    rect_points.push_back(Point2f(r.x,r.y));
    rect_points.push_back(Point2f(r.x+r.width,r.y));
    rect_points.push_back(Point2f(r.x+r.width,r.y+r.height));
    rect_points.push_back(Point2f(r.x,r.y+r.height));
    rect_points.push_back(Point2f(r.x+r.width/2,r.y));
    rect_points.push_back(Point2f(r.x+r.width,r.y+r.height));
    rect_points.push_back(Point2f(r.x+r.width/2,r.y+r.height));
    rect_points.push_back(Point2f(r.x,r.y+r.height/2));
    config = &p;
    sensors_data = &sd;
    svm = s;
    img_pos = Point2f(r.x, r.y);
    speed = 0;
    latitude = 0;
    longitude = 0;
    heading = 0;
    prev_car_keypoints.push_back(Point2f(r.x+r.width/2,r.y+r.height/2));         // Rect center
    prev_car_keypoints.push_back(Point2f(prev_car_keypoints[0].x - 5, prev_car_keypoints[0].y - 5));
    prev_car_keypoints.push_back(Point2f(prev_car_keypoints[0].x + 5, prev_car_keypoints[0].y - 5));
    prev_car_keypoints.push_back(Point2f(prev_car_keypoints[0].x - 5, prev_car_keypoints[0].y + 5));
    prev_car_keypoints.push_back(Point2f(prev_car_keypoints[0].x + 5, prev_car_keypoints[0].y + 5));
    frame_counter = 0;
}

Car::~Car()
{
    
}

void Car::setKeyPoints(vector<Point2f> kp)
{
    prev_car_keypoints = kp;
}

Rect Car::getRect()
{
    return bounding_box;
}

Point2f Car::getPosition()
{
    return img_pos;
}

vector<Point2f> Car::getKeyPoints()
{
    return prev_car_keypoints;
}

vector<Point2f> Car::getRectPoints()
{
    return rect_points;
}

double Car::getSpeed()
{
    return speed;
}

double Car::getLatitude()
{
    return latitude;
}

double Car::getLongitude()
{
    return longitude;
}

bool Car::Update(Mat &prev_frame, Mat &frame, BGTracking &bg_tracking, Mat &f, bool draw_of_vectors)
{
    vector<float> err;
    vector<uchar> status;
    vector<Point2f> new_car_keypoints = prev_car_keypoints;
    calcOpticalFlowPyrLK(prev_frame, frame, prev_car_keypoints,
                         new_car_keypoints, status, err,
                         config->of_winSize, 3, config->of_termcrit, 0,
                         0.001);
                         
    img_pos = Point2f(new_car_keypoints[0].x-13, new_car_keypoints[0].y-25);
    bounding_box = Rect(img_pos.x, img_pos.y, bounding_box.width, bounding_box.height);
    vector<Point2f> r_points;
    r_points.push_back(Point2f(bounding_box.x, bounding_box.y));
    r_points.push_back(Point2f(bounding_box.x + bounding_box.width, bounding_box.y));
    r_points.push_back(Point2f(bounding_box.x + bounding_box.width, bounding_box.y + bounding_box.height));
    r_points.push_back(Point2f(bounding_box.x, bounding_box.y + bounding_box.height));
    r_points.push_back(Point2f(bounding_box.x + bounding_box.width/2, bounding_box.y));
    r_points.push_back(Point2f(bounding_box.x + bounding_box.width, bounding_box.y + bounding_box.height));
    r_points.push_back(Point2f(bounding_box.x + bounding_box.width/2, bounding_box.y + bounding_box.height));
    r_points.push_back(Point2f(bounding_box.x, bounding_box.y + bounding_box.height/2));
    rect_points = r_points;
    
    modulus = 0;
    angle = 0;
    int counter = 0;
    for(unsigned int j=0;j<status.size();j++)
    {
        if (status[j])
        {
            if (draw_of_vectors)
                arrowedLine(f, prev_car_keypoints[j], new_car_keypoints[j], Scalar(255,0,255), 2);
            modulus += sqrt(pow(prev_car_keypoints[j].x - new_car_keypoints[j].x, 2) +
                            pow(prev_car_keypoints[j].y - new_car_keypoints[j].y, 2));
            double ang = atan2(prev_car_keypoints[j].y - new_car_keypoints[j].y,
                               prev_car_keypoints[j].x - new_car_keypoints[j].x);
            if (ang < 0)
                ang += 2*M_PI;
            angle += ang;
            counter++;
        }
    }
    if (counter > 2)
    {
        modulus /= counter;
        angle /= counter;
    }
    else
        return false;
/*
	if (frame_counter == 5)
	{
		frame_counter = 0;
		if (!this->UpdateDetection(frame))
			return false;
	}
	else
		frame_counter++;
*/
	// A still car
    if ((modulus <= bg_tracking.getModulus()*1.15) and (modulus >= bg_tracking.getModulus()*0.85))
        speed = 0;
    // A car moving at the same speed
    else if (modulus <= config->equal_speed_threshold)
		speed = sensors_data->speed;
    else
    {
        if ((bg_tracking.getAngle()*1.2 >= angle) and (bg_tracking.getAngle()*0.8 <=  angle))
        {
			speed = (sensors_data->speed*modulus/bg_tracking.getModulus()) - sensors_data->speed;
        }
        else if (((bg_tracking.getAngle()-M_PI)*1.2 >= angle) and ((bg_tracking.getAngle()-M_PI)*0.8 <=  angle))
			speed = (sensors_data->speed*modulus/bg_tracking.getModulus()) + sensors_data->speed;
        else
			speed = (sensors_data->speed*modulus/bg_tracking.getModulus());
    }
    
    prev_car_keypoints = new_car_keypoints;
    
    return true;
}

void Car::CalcRealPos(Mat &frame, SensorsData &sensors_data, Camera &cam)
{
    int i = img_pos.x - frame.cols/2;
    int j = img_pos.y - frame.rows/2;
    double h = sensors_data.altitude;
    double f = cam.focal_lenght;
    double theta = sensors_data.roll;
    double phi = sensors_data.pitch;
    double ps = cam.pixel_size;
    double lat1 = sensors_data.latitude*M_PI/180;
	double lon1 = sensors_data.longitude*M_PI/180;
    double heading = sensors_data.heading*M_PI/180;
    
    double x_c = ps*j*h/(j*sin(theta) + cos(theta)*(f*cos(phi) + i*sin(phi)));
    double y_c = ps*i*h/(j*sin(theta) + cos(theta)*(f*cos(phi) + i*sin(phi)));
    double z_c = f*h/(j*sin(theta) + cos(theta)*(f*cos(phi) + i*sin(phi)));
    
    // Distance
	double d = sqrt(pow(x_c,2) + pow(y_c,2));
	
	// Angular distance
	double delta = d/R;
	
	double bearing = heading - (270*M_PI/180) + atan2(x_c, y_c);
	
	double lat2 = asin(sin(lat1)*cos(delta) + cos(lat1)*sin(delta)*cos(bearing));
    double lon2 = lon1 + atan2(sin(bearing)*sin(delta)*cos(lat1), cos(delta) - sin(lat1)*sin(lat2));
    
    latitude = lat2*180/M_PI;
    longitude = lon2*180/M_PI;
}

bool Car::UpdateDetection(Mat &image)
{
	Mat car_image = image(this->getRect()).clone();
	
	int mode;
	if (bounding_box.width > bounding_box.height)
		mode = HORIZONTAL_SEARCH;
	else
		mode = VERTICAL_SEARCH;
	
	if (mode == VERTICAL_SEARCH)
	{
		transpose(car_image, car_image);  
		flip(car_image, car_image, 1);
	}
	
    HOGDescriptor hog(Size(50,25), Size(10,10), Size(5,5), Size(5,5), 18);
    vector<float> ders;
    vector<Point> locs;
    
    hog.compute(car_image, ders, Size(5,5), Size(0,0), locs);
    
    Mat hogfeat(1, ders.size(), CV_32FC1);

    for(unsigned int i=0;i<ders.size();i++)
        hogfeat.at<float>(0, i) = ders.at(i);
	
	if (svm->predict(hogfeat) == 1)
		return true;
	else
		return false;
}

double Car::getModulus()
{
	return modulus;
}

double Car::getAngle()
{
	return angle;
}
