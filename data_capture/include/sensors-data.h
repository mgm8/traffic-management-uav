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

#ifndef SENSORS_DATA_H_
#define SENSORS_DATA_H_

#include <stdint.h>

struct SensorsData {
	// BMP180
	double temperature;
	double pressure;
	double altitude;
	// HMC5883
	double mag_x;
	double mag_y;
	double mag_z;
	double heading;
	// MPU6050
	int16_t ax;
	int16_t ay;
	int16_t az;
	int16_t gx;
	int16_t gy;
	int16_t gz;
	float qw;
	float qx;
	float qy;
	float qz;
	float euler_yaw;
	float euler_pitch;
	float euler_roll;
	float yaw;
	float pitch;
	float roll;
	// NEO-6M
	double latitude;
	double longitude;
	double gps_altitude;
	double course;
	double speed;
	uint8_t centisecond;
	uint8_t second;
	uint8_t minute;
	uint8_t hour;
	uint8_t day;
	uint8_t month;
	uint16_t year;
	int8_t satellites;
	double hdop;
};

#endif
