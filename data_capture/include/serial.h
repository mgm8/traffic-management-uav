/*
 * Traffic Management UAV Data Capture.
 * A program to capture images and sensors data with Raspberry Pi.
 * 
 * Copyright (c) 2016 Gabriel Mariano Marcelino <gabriel.mm8@gmail.com>
 * 
 * Based on wiringPi library Serial module (wiringPiSerial)
 * <https://projects.drogon.net/raspberry-pi/wiringpi/>
 * Copyright (c) 2013 Gordon Henderson
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

#ifndef SERIAL_H_
#define SERIAL_H_

int Serial_Open(const char*, const int);
void Serial_Close (const int);
int Serial_DataAvail (const int);
int Serial_Getchar (const int);

#endif
