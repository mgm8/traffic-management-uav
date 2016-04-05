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

#include <stdexcept>

#include "log.h"
#include "aux.h"

Log::Log()
{
	if (!verifyDirectory("logs"))
		createDirectory("logs");
	
	this->openFile(("logs/" + currentDateTime() + ".txt").c_str());
}

Log::Log(std::string log_file)
{	
	this->openFile(log_file.c_str());
}

Log::Log(const char *file)
{	
	this->openFile(file);
}

void Log::openFile(const char *file)
{
	fout.open(file);
    
	if (!fout)
		throw std::runtime_error("Error opening the log file!");
	
	this->WriteHeaderMessage();
		
	log_counter = 0;
}

Log::~Log()
{
	fout.close();
}

void Log::WriteHeaderMessage()
{
	fout << "# Flight log created on: " << currentDateTime() << std::endl;
	fout << "#" << std::endl;
	fout << "# Log code:" << std::endl;
	fout << "#\tn = Log number" << std::endl;
	fout << "#\tI = Frame number and elapsed time until the capture of this frame" << std::endl;
	fout << "#\tB = Barometer data" << std::endl;
	fout << "#\tM = Magnetometer data " << std::endl;
	fout << "#\tA = IMU data" << std::endl;
	fout << "#\tG = GPS data" << std::endl;
	fout << "#" << std::endl;
	fout << "# Example:" << std::endl;
	fout << "# ~" << std::endl;
	fout << "# n:111" << std::endl;
	fout << "# I:25,10545" << std::endl;
	fout << "# B:29.8,101281,0.249868" << std::endl;
	fout << "# M:-1.90909,-9.36364,5.20408,276.654" << std::endl;
	fout << "# A:-27,-462,7526,-2,0,0,0.996887,-0.0501709,0.0124512,0.0588379,-0.118879,-0.0189221,0.101705,-6.81128,-1.76105,-5.65696" << std::endl;
	fout << "# G:-28.8068,-49.2238,21.1,0.16668,3,39,46,0,9,2,2016,9,86" << std::endl;
	fout << "# ;" << std::endl;
	fout << "#" << std::endl;
}

void Log::Write(unsigned int frame_number, unsigned int elapsed_time, SensorsData *sData)
{
	fout << "~" << std::endl;
	fout << "n:" << log_counter << std::endl;
	fout << "I:" << frame_number << "," << elapsed_time << std::endl;
	fout << "B:" << sData->temperature << "," << sData->pressure << "," << sData->altitude << std::endl;
	fout << "M:" << sData->mag_x << "," << sData->mag_y << "," << sData->mag_z << "," << sData->heading << std::endl;
	fout << "A:" << sData->ax << "," << sData->ay << "," << sData->az << ",";
	fout << sData->gx << "," << sData->gy << "," << sData->gz << ",";
	fout << sData->qw << "," << sData->qx << "," << sData->qy << "," << sData->qz << ",";
	fout << sData->euler_yaw << "," << sData->euler_pitch << "," << sData->euler_roll << ",";
	fout << sData->yaw << "," << sData->pitch << "," << sData->roll << std::endl;
	fout << "G:";
	fout << sData->latitude << "," << sData->longitude << "," << sData->gps_altitude << ",";
	fout << sData->course << ",";
	fout << sData->speed << ",";
	fout << int(sData->hour) << "," << int(sData->minute) << "," << int(sData->second) << "," << int(sData->centisecond) << ",";
	fout << int(sData->day) << "," << int(sData->month) << "," << int(sData->year) << ",";
	fout << int(sData->satellites) << ",";
	fout << int(sData->hdop) << std::endl;
	fout << ";" << std::endl;
	
	log_counter++;
}
