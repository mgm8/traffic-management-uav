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

#ifndef NEO_6M_H_
#define NEO_6M_H_

#include "TinyGPS++.h"

#define RPI_SERIAL_ADDRESS "/dev/ttyAMA0"
#define NEO_6M_BAUD_RATE 9600

class NEO_6M {
		int fd;
		TinyGPSPlus *gps;
	public:
		NEO_6M(const char* adr=RPI_SERIAL_ADDRESS);
		~NEO_6M();
		void Read();
		TinyGPSPlus getData();
};

#endif
