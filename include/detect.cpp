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
#include <algorithm>

#include "detect.h"
#include "aux.h"

using namespace std;
using namespace cv;
using namespace cv::ml;

double CalcVariance(Mat &image)
{
    vector<double> mean;
    vector<double> std_dev;
    meanStdDev(image, mean, std_dev);
    
    return pow(std_dev[0], 2);
}

Mat getHOGDescriptors(Mat image, Config &config, int mode)
{
	if (mode == VERTICAL_SEARCH)
	{
		transpose(image, image);  
		flip(image, image, 1);
	}
	
    HOGDescriptor hog(Size(50,25), Size(10,10), Size(5,5), Size(5,5), 18);
    vector<float> ders;
    vector<Point> locs;
    
    hog.compute(image, ders, Size(5,5), Size(0,0), locs);
    
    Mat hogfeat(1, ders.size(), CV_32FC1);

    for(unsigned int i=0;i<ders.size();i++)
        hogfeat.at<float>(0, i) = ders.at(i);
    
    return hogfeat;
}

int max(int x, int y)
{
	return (x > y)? x:y;
}

int min(int x, int y)
{
	return (x < y)? x:y;
}

vector<int> argsort(vector<int> input)
{
    vector<int> idx;

    for(int i=0; i<input.size(); ++i)
        idx.push_back(i);

    sort(idx.begin(), idx.end(), IdxCompare(input));

    return idx;
}

vector<Rect> NonMaximaSupression(vector<Rect> boxes, float overlapThresh)
{
    // If there is no boxes, you don't need this function
    if (boxes.empty())
        return boxes;
    
    vector<Rect> output_boxes;
    vector<int> pick;
    
    // Get rectangles coordinates
    vector<int> x1;
    vector<int> y1;
    vector<int> x2;
    vector<int> y2;
    for(unsigned int i=0;i<boxes.size();i++)
    {
        x1.push_back(boxes[i].x);
        y1.push_back(boxes[i].y);
        x2.push_back(boxes[i].x + boxes[i].width);
        y2.push_back(boxes[i].y + boxes[i].height);
    }
    
    // Calc areas
    vector<int> area;
    for(unsigned int i=0;i<boxes.size();i++)
        area.push_back((x2[i] - x1[i] + 1)*(y2[i] - y1[i] + 1));
    
    // Sort y2 index
    vector<int> ids = argsort(y2);
    
    while(ids.size() > 0)
    {
        int last = ids.size() - 1;
        int i = ids[last];
        pick.push_back(i);
        vector<int> suppress;
        suppress.push_back(last);
        
        for(unsigned int pos=0;pos<last;pos++)
        {
            int j = ids[pos];
            
            int xx1 = max(x1[i], x1[j]);
            int yy1 = max(y1[i], y1[j]);
            int xx2 = min(x2[i], x2[j]);
            int yy2 = min(y2[i], y2[j]);
            
            int w = max(0, xx2 - xx1 + 1);
            int h = max(0, yy2 - yy1 + 1);
            
            float overlap = float(w*h)/area[j];
            
            if (overlap > overlapThresh)
                suppress.push_back(pos);
        }
        
        sort(suppress.begin(), suppress.end());
        for(int k=suppress.size()-1;k>=0;k--)
            ids.erase(ids.begin() + suppress[k]);
    }
    
    // Return filtered boxes
    for(unsigned int i=0;i<boxes.size();i++)
        for(unsigned int j=0;j<pick.size();j++)
            if (i == pick[j])
                output_boxes.push_back(boxes[i]);
    
    return output_boxes;
}

void DetectCars(Mat &image, Ptr<SVM> svm, Config &config,
                SensorsData &sensors_data, vector<Car> &cars,
                vector<Rect> &rois, int mode)
{
    vector<Rect> cars_rect;
	
	int sliding_window_width;
	int sliding_window_height;
	if (mode == HORIZONTAL_SEARCH)
	{
		sliding_window_width = config.sliding_window_width;
		sliding_window_height = config.sliding_window_height;
	}
	else if (mode == VERTICAL_SEARCH)
	{
		sliding_window_width = config.sliding_window_height;
		sliding_window_height = config.sliding_window_width;
	}
	
    for(int r=0;r<rois.size();r++)
    {
        int min_i = rois[r].y;
        int max_i = rois[r].y + rois[r].height;
        int min_j = rois[r].x;
        int max_j = rois[r].x + rois[r].width;
        
        for(float s = config.pyramid_initial_scale;                     // Pyramid search method
            s <= config.pyramid_final_scale;
            s += config.pyramid_scale_step)
        {
            Mat resized_gray_image;
            resize(image, resized_gray_image, Size(), s, s);
            for(int i = min_i*s;
                i < (max_i*s - sliding_window_height);
                i += config.sliding_window_vertical_step)
            {
                for(int j = min_j*s;
                    j < (max_j*s - sliding_window_width);
                    j += config.sliding_window_horizontal_step)
                {
                    Rect sliding_rect(j, i, sliding_window_width,
                                      sliding_window_height);
                    vector<Point2f> rect_corners;
                    rect_corners.push_back(Point2f(sliding_rect.x, sliding_rect.y));
                    rect_corners.push_back(Point2f(sliding_rect.x + sliding_rect.width, sliding_rect.y));
                    rect_corners.push_back(Point2f(sliding_rect.x + sliding_rect.width, sliding_rect.y + sliding_rect.height));
                    rect_corners.push_back(Point2f(sliding_rect.x, sliding_rect.y + sliding_rect.height));
                    bool is_inside = false;
                    for(int c=0;c<cars.size();c++)
                    {
                        if (cars[c].getRect().x == j)
                            if (cars[c].getRect().y == i)
                            {
                                is_inside = true;
                                break;
                            }
                        for(int k=0;k<cars[c].getRectPoints().size();k++)
                        {
                            if (pointPolygonTest(rect_corners, cars[c].getRectPoints()[k], false) >= 0)
                            {
                                is_inside = true;
                                break;
                            }
                        }
                    }
                    if (is_inside)
                        continue;
                    Mat sliding_mat = resized_gray_image(sliding_rect).clone();
                    // Variance filter
                    if (CalcVariance(sliding_mat) > config.variance_threshold)
                    {
                        // HOG
                        vector<float> hyperplane_distance;
                        svm->predict(getHOGDescriptors(sliding_mat, config, mode),
                                     hyperplane_distance, 1);
                        if (hyperplane_distance[0] < config.svm_min_hyperplane_distance)
                        {
                            // Image mirror filter
                            Mat car_flip;
                            if (mode == HORIZONTAL_SEARCH)
                                flip(sliding_mat, car_flip, 0);
                            else
                                flip(sliding_mat, car_flip, 1);
                            vector<float> hdf;
                            svm->predict(getHOGDescriptors(car_flip, config, mode),
                                         hdf, 1);
                            if (hdf[0] < config.svm_min_hyperplane_distance)
                            {
                                cars_rect.push_back(Rect(j/s, i/s, sliding_window_width,
                                                     sliding_window_height));
                            }
                        }
                    }
                }
            }
        }
    }
    
    // Non-Maxima Supression filter
    cars_rect = NonMaximaSupression(cars_rect);
        
    // Add cars
    for(unsigned int i=0;i<cars_rect.size();i++)
        cars.push_back(Car(cars_rect[i], config, sensors_data, svm));
}

vector<Point2f> DetectKeyPoints(Mat &image, Config &config)
{
    vector<KeyPoint> keypoints;
	FAST(image, keypoints, config.fast_threshold, true);
    //detector->detect(image, keypoints);
    return KeyPoint2Point2f(keypoints);
}

void ClassifingKeyPoints(vector<Point2f> &kp, vector<Car> &cars)
{
    for(unsigned int i=0;i<cars.size();i++)
    {
        for(unsigned int j=0;j<kp.size();j++)
        {
            vector<Point2f> rect_corners;
            rect_corners.push_back(Point2f(cars[i].getRect().x, cars[i].getRect().y));
            rect_corners.push_back(Point2f(cars[i].getRect().x+cars[i].getRect().width, cars[i].getRect().y));
            rect_corners.push_back(Point2f(cars[i].getRect().x+cars[i].getRect().width, cars[i].getRect().y+cars[i].getRect().height));
            rect_corners.push_back(Point2f(cars[i].getRect().x, cars[i].getRect().y+cars[i].getRect().height));
            if (pointPolygonTest(rect_corners, kp[j], false) >= 0)
            {
                kp.erase(kp.begin()+j);
                j--;
            }
        }
    }
}
