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

#include <sys/types.h>
#include <sys/stat.h>
#include <ctime>
#include <stdexcept>

#include "aux.h"

bool verifyDirectory(std::string folder)
{
	return verifyDirectory(folder.c_str());
}

bool verifyDirectory(const char *folder)
{
	struct stat info;
	
	if ((stat(folder, &info) != 0) && !(info.st_mode & S_IFDIR))
		return false;
	else
		return true;
}

bool verifyFile(std::string file)
{
	return verifyFile(file.c_str());
}

bool verifyFile(const char *file)
{
	struct stat info;
	
	if ((stat(file, &info) != 0) && !(info.st_mode & S_IFREG))
		return false;
	else
		return true;
}

void createDirectory(std::string folder)
{
	return createDirectory(folder.c_str());
}

void createDirectory(const char* folder)
{
	const int dir_err = mkdir(folder, ACCESSPERMS);
	if (dir_err == -1)
		throw std::runtime_error("Error creating a new folder!");
}

std::string currentDateTime()
{
	time_t now = time(0);
	struct tm tstruct;
	char buf[80];
	tstruct = *localtime(&now);
	strftime(buf, sizeof(buf), "%d-%m-%Y_%H-%M-%S", &tstruct);
	return buf;
}
