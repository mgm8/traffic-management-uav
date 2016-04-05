/*
 * Traffic Management UAV Data Capture.
 * A program to capture images and sensors data with Raspberry Pi.
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

#include <unistd.h>
#include <stdint.h>
#include <sstream>
#include <cmath>
#include <thread>
#include <mutex>
#include <vector>
#include <stdexcept>
#include <chrono>
#include <opencv2/opencv.hpp>

#include "include/bmp180.h"
#include "include/hmc5883.h"
#include "include/MPU6050_6Axis_MotionApps20.h"							// The MPU6050.h file is also included
#include "include/neo-6m.h"
#include "include/log.h"
#include "include/sensors-data.h"
#include "include/aux.h"

//#define VIDEO_OUTPUT

#define MAG_DECLINATION -0.3349

using namespace std;
using namespace std::chrono;
using namespace cv;

struct Frame {
	Mat frame;
	unsigned int n;
};

void CaptureImages(VideoCapture*, vector<Frame>*, unsigned int*, unsigned int*);
void RecordImages(vector<Frame>*);
void RecordVideo(VideoWriter*, vector<Frame>*);
void ReadSensors(BMP180*, HMC5883*, MPU6050*, NEO_6M*, Log*,
				 uint16_t, vector<Frame>*, unsigned int*, unsigned int*);
void ReadDMP(MPU6050*, uint16_t, SensorsData*);

auto datalog_start = high_resolution_clock::now();						// Reference time
mutex mt;

int main()
{
	BMP180 bar;
	HMC5883 mag(MAG_DECLINATION);
	MPU6050 imu;
	NEO_6M *gps;
	VideoCapture camCapture;
	#ifdef VIDEO_OUTPUT
	VideoWriter camOutput;
	#endif
	Log log;
	uint16_t packetSize;												// expected DMP packet size (default is 42 bytes)
	vector<Frame> frames;
	unsigned int frame_counter = 0;
	unsigned int frame_capture_time = 0;
	
	// MPU6050 and its DMP initialization
	imu.initialize();
	if (imu.dmpInitialize() == 0)										// return status after each device operation (0 = success, !0 = error)
	{
		imu.setDMPEnabled(true);										// turn on the DMP, now that it's ready
		packetSize = imu.dmpGetFIFOPacketSize();						// get expected DMP packet size for later comparison
    }
    else
		throw runtime_error("It is not possible to initialize the MPU6050 DMP!");

	// NEO-6M initialization
	gps = new NEO_6M();
	
	// Camera initialization
	camCapture.open(0);
	if (!camCapture.isOpened())
		throw runtime_error("It is not possible to open the camera!");
	
	#ifdef VIDEO_OUTPUT
	// Video writer initialization
	camOutput.open((currentDateTime() +".avi").c_str(), CV_FOURCC('P','I','M','1'), 25,
				   Size(camCapture.get(CAP_PROP_FRAME_WIDTH),
				   camCapture.get(CAP_PROP_FRAME_HEIGHT)), true);
	#endif
	
	thread image_capture_thread(CaptureImages, &camCapture, &frames, 
											   &frame_counter,
											   &frame_capture_time);
	thread image_record_thread(RecordImages, &frames);
	
	#ifdef VIDEO_OUTPUT
	thread video_record_thread(RecordVideo, &camOutput, &frames);
	#endif
	
	thread read_sensors_thread(ReadSensors, &bar, &mag, &imu, gps,
											&log, packetSize, &frames,
											&frame_counter,
											&frame_capture_time);

	image_capture_thread.join();
	image_record_thread.join();
	#ifdef VIDEO_OUTPUT
	video_record_thread.join();
	#endif
	read_sensors_thread.join();

	delete gps;
	camCapture.release();
	#ifdef VIDEO_OUTPUT
	camOutput.release();
	#endif

	return 0;
}

void CaptureImages(VideoCapture *camCapture, vector<Frame> *cap,
				   unsigned int *frame_counter, unsigned int *frame_capture_time)
{
	unsigned int empty_frames = 0;
	
	Frame fr;
	
	fr.n = 0;
	
	while(true)
	{
		camCapture->read(fr.frame);
		auto frame_capture_end = high_resolution_clock::now();
		
		if (!fr.frame.empty())
		{
			mt.lock();	
			cap->push_back(fr);
			mt.lock();
			
			*frame_counter = *frame_counter + 1;
			fr.n++;
			*frame_capture_time = duration_cast<milliseconds>(frame_capture_end - datalog_start).count();
			
			if (empty_frames > 0)
				empty_frames = 0;
		}
		else
		{
			empty_frames++;
			
			if (empty_frames == 100)
			{
				throw runtime_error("There is a problem with the image capture!");
				break;
			}
		}
	}
}

void RecordImages(vector<Frame> *cap)
{	
	string folder = "captures";
	
	if (!verifyDirectory(folder))
		createDirectory(folder);
		
	folder += "/";
	folder += currentDateTime();
	createDirectory(folder);
	folder += "/";
	
	string file_name;
	
	while(true)
	{
		while(!cap->empty())
		{
			ostringstream buffer;
			
			unsigned int frame_number = cap->front().n;
	
			buffer << cap->front().n;
			file_name = folder;
			
			if (frame_number < 10)
				file_name += "00000";
			else if (frame_number < 100)
				file_name += "0000";
			else if (frame_number < 1000)
				file_name += "000";
			else if (frame_number < 10000)
				file_name += "00";
			else if (frame_number < 100000)
				file_name += "0";
			
			file_name += buffer.str();
			file_name += ".jpg";
			
			imwrite(file_name, cap->front().frame);
			
			mt.lock();
			cap->erase(cap->begin());
			mt.unlock();
		}
	}
}

void RecordVideo(VideoWriter* camOutput, vector<Frame> *cap)
{
	while(true)
	{
		while(!cap->empty())
		{
			camOutput->write(cap->front().frame);
			
			cap->erase(cap->begin());
		}
	}
}

void ReadSensors(BMP180 *bar, HMC5883 *mag, MPU6050 *imu, NEO_6M *gps,
				 Log *log, uint16_t packetSize, vector<Frame> *cap,
				 unsigned int *frame_counter, unsigned int *frame_capture_time)
{
	SensorsData sData;
	
	int one_sec_counter = 0;
	int hundred_ms_counter = 0;
	
	while(true)
	{	
		// Get acc/gyro data
		ReadDMP(imu, packetSize, &sData);
		
		// Get barometer pressure and altitude
		bar->Read();
		sData.pressure = bar->getPressure();
		sData.altitude = bar->getAltitude();
		
		if (hundred_ms_counter == 2)									// 100 ms timer
		{
			// Get magnetometer data
			mag->Read();
			sData.mag_x = mag->getXMagData();
			sData.mag_y = mag->getYMagData();
			sData.mag_z = mag->getZMagData();
			sData.heading = mag->getHeading();
			
			hundred_ms_counter = 0;
		}
		
		if (one_sec_counter == 20)										// 1 second timer (Sample rate = 1 Hz)
		{
			// Get barometer temperature
			sData.temperature = bar->getTemperature();
			
			// Get GPS data
			gps->Read();
			sData.latitude = gps->getData().location.lat();
			sData.longitude = gps->getData().location.lng();
			sData.gps_altitude = gps->getData().altitude.meters();
			sData.course = gps->getData().course.deg();
			sData.speed = gps->getData().speed.kmph();
			sData.centisecond = gps->getData().time.centisecond();
			sData.second = gps->getData().time.second();
			sData.minute = gps->getData().time.minute();
			sData.hour = gps->getData().time.hour();
			sData.day = gps->getData().date.day();
			sData.month = gps->getData().date.month();
			sData.year = gps->getData().date.year();
			sData.satellites = gps->getData().satellites.value();
			sData.hdop = gps->getData().hdop.value();
			
			one_sec_counter = 0;
		}
		
		log->Write(*frame_counter, *frame_capture_time, &sData);
		
		usleep(50000);
		
		one_sec_counter++;
		hundred_ms_counter++;
	}
}

// Read MPU6050 DMP data
void ReadDMP(MPU6050 *imu, uint16_t packetSize, SensorsData *sData)
{
	uint8_t fifoBuffer[64];												// FIFO storage buffer
	
	Quaternion q;														// [w, x, y, z] quaternion container
	VectorFloat gravity;												// [x, y, z] gravity vector
	float ypr[3];														// [yaw, pitch, roll] yaw/pitch/roll container and gravity vector
	float euler[3];

	int16_t accel[3];
	int16_t gyro[3];

	uint16_t fifoCount = imu->getFIFOCount();							// count of all bytes currently in FIFO
	if (fifoCount == 1024)
	{
		imu->resetFIFO();												// reset so we can continue cleanly
		sData->ax = sData->ay = sData->az = 0;
		sData->gx = sData->gy = sData->gz = 0;
		sData->yaw = sData->pitch = sData->roll = 0;
	}
	else if (fifoCount >= 42)											// otherwise, check for DMP data ready interrupt (this should happen frequently)
	{
        imu->getFIFOBytes(fifoBuffer, packetSize);						// read a packet from FIFO
        imu->dmpGetAccel(accel, fifoBuffer);
        imu->dmpGetGyro(gyro, fifoBuffer);
		imu->dmpGetQuaternion(&q, fifoBuffer);
		imu->dmpGetGravity(&gravity, &q);
		imu->dmpGetEuler(euler, &q);
		imu->dmpGetYawPitchRoll(ypr, &q, &gravity);
		
		sData->ax = accel[0];
		sData->ay = accel[1];
		sData->az = accel[2];
		sData->gx = gyro[0];
		sData->gy = gyro[1];
		sData->gz = gyro[2];
		sData->qw = q.w;
		sData->qx = q.x;
		sData->qy = q.y;
		sData->qz = q.z;
		sData->euler_yaw = euler[0];
		sData->euler_pitch = euler[1];
		sData->euler_roll = euler[2];
		sData->yaw = ypr[0] * 180/M_PI;									// Conversion from radians to degrees
		sData->pitch = ypr[1] * 180/M_PI;
		sData->roll = ypr[2] * 180/M_PI;
	}
	else
	{
		sData->ax = sData->ay = sData->az = 0;
		sData->gx = sData->gy = sData->gz = 0;
		sData->yaw = sData->pitch = sData->roll = 0;
	}
}
