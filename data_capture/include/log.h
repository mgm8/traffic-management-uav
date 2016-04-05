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

#ifndef LOG_H_
#define LOG_H_

#include <fstream>
#include <string>

#include "sensors-data.h"

class Log {
		std::ofstream fout;
		unsigned int log_counter;
	public:
		Log();
		Log(std::string);
		Log(const char*);
		void openFile(const char*);
		~Log();
		void WriteHeaderMessage();
		void Write(unsigned int, unsigned int, SensorsData*);
};

#endif
