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

#include "neo-6m.h"
#include "serial.h"

NEO_6M::NEO_6M(const char* adr)
{
	if ((fd = Serial_Open(adr, NEO_6M_BAUD_RATE)) < 0)
		throw std::runtime_error("Error openning the serial device!");
	else
		gps = new TinyGPSPlus();
}

NEO_6M::~NEO_6M()
{
	delete gps;
	
	Serial_Close(fd);
}

void NEO_6M::Read()
{
	while(Serial_DataAvail(fd))
		gps->encode(Serial_Getchar(fd));
}

TinyGPSPlus NEO_6M::getData()
{
	return *gps;
}
