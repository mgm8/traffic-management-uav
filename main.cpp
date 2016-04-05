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

#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <opencv2/opencv.hpp>

#include "include/car.h"
#include "include/detect.h"
#include "include/aux.h"
#include "include/bgtracking.h"
#include "include/config.h"
#include "include/sensors_data.h"
#include "include/camera.h"

//#define VIDEO_OUTPUT 1

using namespace std;
using namespace cv;
using namespace cv::ml;
using namespace std::chrono;

int main(int argc, char **argv)
{
    // Opening the file capture
    VideoCapture cap(argv[1]);
    if (!cap.isOpened())
    {
        cout << "Error opening the capture" << endl;
        return -1;
    }
    
    // Load trained SVM data
    Ptr<SVM> svm = SVM::load<SVM>("hog-svm-cars.xml");
    
    Mat frame;
    Mat gray_frame;
    Mat prev_frame;
    Mat prev_gray_frame;
    
    // Get the first frame for a initial setup
    cap >> prev_frame;
    cvtColor(prev_frame, prev_gray_frame, CV_BGR2GRAY);
    
    // Set camera specs
    Camera cam_specs(Size(1920, 1080), 0.004, 2.8e-6);
    
    // Set config parameters
    Config config;
    Setup(config, prev_frame);
    
    // Initial sensors read
    SensorsData sensors_data;
    ReadSensors(sensors_data);

    // Set ROIs
    vector<Rect> initial_roi;
    initial_roi.push_back(Rect(0, 0, prev_frame.cols, prev_frame.rows));

    vector<Rect> rois;
    rois.push_back(Rect(200, 0, prev_frame.cols-400, 100));
    rois.push_back(Rect(200, prev_frame.rows-100, prev_frame.cols-400, 100));

    vector<Rect> rois1;
    rois1.push_back(rois[0]);

    vector<Rect> rois2;
    rois2.push_back(rois[1]);

    // Detect the cars in first frame
    vector<Car> cars;
    DetectCars(prev_gray_frame, svm, config, sensors_data, cars,
               initial_roi, VERTICAL_SEARCH);

    // Finding keypoints in first frame
    vector<Point2f> keypoints = DetectKeyPoints(prev_gray_frame, config);
    
    // Classify keypoints
    ClassifingKeyPoints(keypoints, cars);
        
    BGTracking bg_tracking(keypoints, config);
    
    // Output record
    #ifdef VIDEO_OUTPUT
    VideoWriter capOutput("result.avi", CV_FOURCC('H','2','6','4'), cap.get(CV_CAP_PROP_FPS),
                          prev_frame.size(), true);
    #endif
    
    unsigned int frame_counter = 0;
    while(true)
    {
        auto t0 = high_resolution_clock::now();
        cap >> frame;
        if (frame.empty())
            break;
        
        cvtColor(frame, gray_frame, CV_BGR2GRAY);
        
        // Update background tracking
        if (!bg_tracking.Update(prev_gray_frame, gray_frame, frame, false) or (frame_counter == config.frames_to_update))
        {
            vector<Point2f> keypoints = DetectKeyPoints(gray_frame, config);
            ClassifingKeyPoints(keypoints, cars);
            bg_tracking.setKeyPoints(keypoints);
            frame_counter = 0;
        }

        // Search for new cars
        vector<Car> cars2 = cars;
        int cars_size = cars.size();
        thread detect_cars_thread1(DetectCars, ref(gray_frame), svm, ref(config),
											   ref(sensors_data), ref(cars), ref(rois1),
                                               VERTICAL_SEARCH);
        thread detect_cars_thread2(DetectCars, ref(gray_frame), svm, ref(config),
                                               ref(sensors_data), ref(cars2), ref(rois2),
                                               VERTICAL_SEARCH);
        detect_cars_thread1.join();
        detect_cars_thread2.join();
        
        for(unsigned int i=cars_size;i<cars2.size();i++)
            cars.push_back(cars2[i]);
        
        // Update cars tracking
        for(unsigned int i=0;i<cars.size();i++)
            if (!cars[i].Update(prev_gray_frame, gray_frame, bg_tracking, frame, false))
            {
                cars.erase(cars.begin()+i);
                i--;
            }

        // Draw results
        Mat result = DrawResults(frame, cars, bg_tracking, sensors_data, config);

        //result = DrawKeyPoints(frame, bg_tracking.getKeyPoints(), Scalar(0,0,255));
        //result = DrawCarsKeyPoints(frame, cars, Scalar(0,255,255));
        //result = DrawROIs(result, rois);

        auto t1 = high_resolution_clock::now();

        #ifndef VIDEO_OUTPUT
        result = WriteFPS(result, 1000/double(duration_cast<milliseconds>(t1-t0).count()));
        
        imshow("Traffic management UAV", result);
        char d = waitKey(1);
        if (d == 'q')
            break;
        if (d == 's')
            imwrite("print.png", result);
        #else
        capOutput << result;
        #endif
        
        swap(prev_gray_frame, gray_frame);
        frame_counter++;
    }

    cap.release();
    
    #ifdef VIDEO_OUTPUT
    capOutput.release();
    #endif
    
    return 0;
}
