/*
 * BMP180 Raspberry Pi library:
 * A C++ library to access Bosch BMP180 sensor in Raspberry Pi.
 *
 * Copyright (C) 2016 Gabriel Mariano Marcelino <gabriel.mm8@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program; if not, see <http://www.gnu.org/licenses/>
 * 
 */

#include <stdexcept>
#include <cmath>														// pow()
#include <unistd.h>														// usleep() and close()

#include "bmp180.h"
#include "i2c.h"												        // I2C access functions

BMP180::BMP180(unsigned int o, int rpi_version)
{
	if ((o < BMP180_ULTRALOWPOWER) or (o > BMP180_ULTRAHIGHRES))
		throw std::runtime_error("Invalid mode!");
	else
		oss = o;
	
	if ((fd = I2C_Setup(BMP180_ADR, rpi_version)) < 0)
        throw std::runtime_error("Failed to open I2C bus!");
    else
    {   
        this->LoadCalibration();
		this->setBasePressure();
	}
}

BMP180::~BMP180()
{
	if (fd >= 0)
		close(fd);
}

// Calibration data reading
void BMP180::LoadCalibration()
{
	cal_reg.ac1 = (I2C_ReadReg8(fd, AC1_MSB) << 8) + I2C_ReadReg8(fd, AC1_LSB);
	cal_reg.ac2 = (I2C_ReadReg8(fd, AC2_MSB) << 8) + I2C_ReadReg8(fd, AC2_LSB);
	cal_reg.ac3 = (I2C_ReadReg8(fd, AC3_MSB) << 8) + I2C_ReadReg8(fd, AC3_LSB);
	cal_reg.ac4 = (I2C_ReadReg8(fd, AC4_MSB) << 8) + I2C_ReadReg8(fd, AC4_LSB);
	cal_reg.ac5 = (I2C_ReadReg8(fd, AC5_MSB) << 8) + I2C_ReadReg8(fd, AC5_LSB);
	cal_reg.ac6 = (I2C_ReadReg8(fd, AC6_MSB) << 8) + I2C_ReadReg8(fd, AC6_LSB);
	cal_reg.b1 = (I2C_ReadReg8(fd, B1_MSB) << 8) + I2C_ReadReg8(fd, B1_LSB);
	cal_reg.b2 = (I2C_ReadReg8(fd, B2_MSB) << 8) + I2C_ReadReg8(fd, B2_LSB);
	cal_reg.mb = (I2C_ReadReg8(fd, MB_MSB) << 8) + I2C_ReadReg8(fd, MB_LSB);
	cal_reg.mc = (I2C_ReadReg8(fd, MC_MSB) << 8) + I2C_ReadReg8(fd, MC_LSB);
	cal_reg.md = (I2C_ReadReg8(fd, MD_MSB) << 8) + I2C_ReadReg8(fd, MD_LSB);
}

// Read uncompensated temperature value
int32_t BMP180::getUT()
{
	if (I2C_WriteReg8(fd, BMP180_REG_CONTROL, BMP180_COMMAND_TEMPERATURE) < 0)
	{
		throw std::runtime_error("Error writing into the device!");
		
		return -1;
	}
	else
	{
		usleep(5000);
		
		int32_t UT = (I2C_ReadReg8(fd, BMP180_REG_RESULT) << 8) + I2C_ReadReg8(fd, 0xF7);
		
		return UT;
	}
}

// Read uncompensated pressure value
int32_t BMP180::getUP()
{
	if (I2C_WriteReg8(fd, BMP180_REG_CONTROL, BMP180_COMMAND_PRESSURE + (oss << 6)) < 0)
	{
		throw std::runtime_error("Error writing into the device!");
		
		return -1;
	}
	else
	{
		switch(oss)
		{
			case BMP180_ULTRALOWPOWER:
				usleep(5000);
				break;
			case BMP180_STANDARD:
				usleep(8000);
				break;
			case BMP180_HIGHRES:
				usleep(14000);
				break;
			case BMP180_ULTRAHIGHRES:
				usleep(26000);
				break;
			default:
				usleep(8000);
				break;
		}

		int32_t UP = ((I2C_ReadReg8(fd, BMP180_REG_RESULT) << 16) + (I2C_ReadReg8(fd, 0xF7) << 8) + I2C_ReadReg8(fd, 0xF8)) >> (8 - oss);

		return UP;
	}
}

void BMP180::Read()
{
	UT = this->getUT();
	UP = this->getUP();
}

// Calculate true temperature
double BMP180::getTemperature()
{
	//int32_t UT = this->getUT();
	int32_t x1 = ((UT - cal_reg.ac6)*cal_reg.ac5) >> 15;
	int32_t x2 = (cal_reg.mc << 11)/(x1 + cal_reg.md);
	int32_t b5 = x1 + x2;
	int32_t T = (b5 + 8) >> 4;
	
	return T/10.0;
}

// Calculate true pressure
int32_t BMP180::getPressure()
{
	//int32_t UT = this->getUT();
	//int32_t UP = this->getUP();
	
	int32_t x1 = ((UT - cal_reg.ac6)*cal_reg.ac5) >> 15;
	int32_t x2 = (cal_reg.mc << 11)/(x1 + cal_reg.md);
	int32_t b5 = x1 + x2;
	
	int32_t b6 = b5 - 4000;
	x1 = (cal_reg.b2*(b6*b6) >> 12) >> 11;
	x2 = (cal_reg.ac2*b6) >> 11;
	int32_t x3 = x1 + x2;
	int32_t b3 = (((cal_reg.ac1*4 + x3) << oss) + 2)/4;
	x1 = (cal_reg.ac3*b6) >> 13;
	x2 = (cal_reg.b1*(b6*b6) >> 12) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	uint32_t b4 = (cal_reg.ac4*(x3 + 32768)) >> 15;
	uint32_t b7 = (UP - b3)*(50000 >> oss);
	
	int32_t P;
	if (b7 < 0x80000000)
		P = (b7*2)/b4;
	else
		P = (b7/b4)*2;
		
	x1 = (P >> 8)*(P >> 8);
	x1 = (x1*3038) >> 16;
	x2 = (-7357*P) >> 16;
	P = P + ((x1 + x2 + 3791) >> 4);
	
	return P;															// Pressure in Pa
}

// Calculate altitude
double BMP180::getAltitude()
{
	return 44330.0*(1 - pow(double(this->getPressure())/double(this->getBasePressure()), 1/5.255));		// Altitude in meters
}

double BMP180::getSeaLevelRelativeAltitude()
{
	return 44330.0*(1 - pow(double(this->getPressure())/double(SEALEVEL_PRESSURE), 1/5.255));		// Altitude in meters
}

double BMP180::getSeaLevelPressure(double real_altitude)
{
	return double(this->getPressure())/pow(1 - (real_altitude/44330.0), 5.255);
}

void BMP180::setBasePressure()
{
	this->Read();
	P0 = this->getPressure();
}

int32_t BMP180::getBasePressure()
{
	return P0;
}

// Datasheet values for calibration
void BMP180::LoadDatasheetCalibration()
{
	cal_reg.ac1 = 408;
	cal_reg.ac2 = -72;
	cal_reg.ac3 = -14383;
	cal_reg.ac4 = 32741;
	cal_reg.ac5 = 32757;
	cal_reg.ac6 = 23153;
	cal_reg.b1 = 6190;
	cal_reg.b2 = 4;
	cal_reg.mb = -32768;
	cal_reg.mc = -8711;
	cal_reg.md = 2868;
}
