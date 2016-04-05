/*
 * HOG-SVM Trainer
 * A program to train a SVM classifier with HOG descriptors.
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
#include <string>
#include <sstream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
using namespace cv::ml;

string IntToStr(int x)
{
    ostringstream buffer;
	buffer << x;
    string output = buffer.str();
    return output;
}

Mat CalcHOGDes(Mat image)
{
    // Size input_size(50,25);
    // Size block_size(10,10);  // 2*cell_size
    // Size block_stride(5,5);  // Step
    // Size cell_size(5,5);
    // int angles_number = 18;
    HOGDescriptor hog(Size(50,25), Size(10,10), Size(5,5), Size(5,5), 18);
    vector<float> ders;
    vector<Point> locs;
    
    // stride and paddng (must be (0,0))
    hog.compute(image, ders, Size(5,5), Size(0,0), locs);
    
    Mat hogfeat(1, ders.size(), CV_32FC1);

    for(unsigned int i=0;i<ders.size();i++)
        hogfeat.at<float>(0, i) = ders.at(i);
    
    return hogfeat;
}

int main(int argc, char **argv)
{
    // INPUTS
    const string pos_path = argv[1];                                    // Positive images folder
    const string neg_path = argv[2];                                    // Negative images folder
    
    const string XML_FILE = "hog-svm.xml";
    
    const string IMAGES_FORMAT = ".jpg";                                // If your images have a different format, change it here
    
    const int HNM_STEPS = 2;                                            // Number of hard negative mining iterations
    
    // Load images and compute their HOG descriptors
    cout << "Loading images... ";
    Mat pos_descriptors;
    Mat neg_descriptors;
    for(unsigned int f=1;;f++)
    {
        Mat pos_image = imread(pos_path + IntToStr(f) + IMAGES_FORMAT,
                               CV_LOAD_IMAGE_GRAYSCALE);
        Mat neg_image = imread(neg_path + IntToStr(f) + IMAGES_FORMAT,
                               CV_LOAD_IMAGE_GRAYSCALE);
        
        if (!pos_image.empty())
            pos_descriptors.push_back(CalcHOGDes(pos_image));
        
        if (!neg_image.empty())
            neg_descriptors.push_back(CalcHOGDes(neg_image));
        
        if (pos_image.empty() and neg_image.empty())
            break;
    }
    cout << "DONE!" << endl;
    
    cout << "Positive images: " << pos_descriptors.rows << endl;
    cout << "Negative images: " << neg_descriptors.rows << endl;
    
    // SVM trainning
    cout << "Creating SVM matrix... ";
    Mat samples;
    Mat labels;
    unsigned int samples_size = pos_descriptors.rows + neg_descriptors.rows;
    for(unsigned int i=0;i<samples_size;i++)
        if (i < pos_descriptors.rows)
            labels.push_back(1);
        else
            labels.push_back(0);
    
    vconcat(pos_descriptors, neg_descriptors, samples);
    cout << "DONE!" << endl;
    
    cout << "Training with SVM... ";
    Ptr<SVM> svm = SVM::create();
    svm->setType(SVM::C_SVC);
    svm->setKernel(SVM::LINEAR);
    svm->setGamma(3);
    svm->setDegree(3);

    svm->train(samples, ROW_SAMPLE , labels);
    cout << "DONE!" << endl;

    // Hard negative mining
    cout << "Hard negative mining... ";
    Mat false_pos_descriptors;
    for(unsigned int f=0;f<HNM_STEPS;f++)
    {
        Mat neg_image = imread(neg_path + IntToStr(f) + IMAGES_FORMAT,
                               CV_LOAD_IMAGE_GRAYSCALE);
        
        if (!neg_image.empty())
        {
            for(float s=1.0;s<=2.0;s+=0.5)                              // Multiple scales
            {
                Mat rez_neg_image;
                resize(neg_image, rez_neg_image, Size(), s, s);
                for(unsigned int i=0;i<(rez_neg_image.cols-neg_image.cols);i+=10)
                    for(unsigned int j=0;j<(rez_neg_image.rows-neg_image.rows);j+=10)
                    {
                        Rect rect(i, j, neg_image.cols, neg_image.rows);
                        Mat search_rect = rez_neg_image(rect).clone();
                        Mat search_rect_descr = CalcHOGDes(search_rect);
                        if (svm->predict(search_rect_descr) == 1)
                            false_pos_descriptors.push_back(search_rect_descr);
                    }
            }
        }
        else
            break;
    }
    cout << "DONE!" << endl;
    
    // SVM retraining (With hard negative mining)
    cout << "Retraining SVM with hard negative mining... ";
    if (false_pos_descriptors.rows > 1)
    {
        vconcat(samples, false_pos_descriptors, samples);
        for(unsigned int i=0;i<false_pos_descriptors.rows;i++)
            labels.push_back(0);

        svm->train(samples, ROW_SAMPLE , labels);
        cout << "DONE!" << endl;
    }
    else
        cout << "No false positives found!" << endl;
    
    cout << "Saving the SVM training... ";
    svm->save(XML_FILE);
    cout << "DONE!" << endl;
    
    return 0;
}
