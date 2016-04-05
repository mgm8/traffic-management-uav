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
 
#ifndef BMP180_H_
#define BMP180_H_

#include <stdint.h>

#define BMP180_ADR 0x77

#define	BMP180_REG_CONTROL 0xF4
#define	BMP180_REG_RESULT 0xF6

// Commands
#define BMP180_COMMAND_TEMPERATURE 0x2E
#define BMP180_COMMAND_PRESSURE 0x34

// Modes
#define BMP180_ULTRALOWPOWER 0
#define BMP180_STANDARD 1
#define BMP180_HIGHRES 2
#define BMP180_ULTRAHIGHRES 3

// Calibration registers addresses
#define AC1_MSB 0xAA 
#define AC2_MSB 0xAC
#define AC3_MSB 0xAE
#define AC4_MSB 0xB0
#define AC5_MSB 0xB2
#define AC6_MSB 0xB4
#define B1_MSB 0xB6
#define B2_MSB 0xB8
#define MB_MSB 0xBA
#define MC_MSB 0xBC
#define MD_MSB 0xBE
#define AC1_LSB 0xAB 
#define AC2_LSB 0xAD
#define AC3_LSB 0xAF
#define AC4_LSB 0xB1
#define AC5_LSB 0xB3
#define AC6_LSB 0xB5
#define B1_LSB 0xB7
#define B2_LSB 0xB9
#define MB_LSB 0xBB
#define MC_LSB 0xBD
#define MD_LSB 0xBF

// Sealevel pressure
#define  SEALEVEL_PRESSURE 101325

struct CalibReg {
	int16_t ac1;
	int16_t ac2;
	int16_t ac3;
	uint16_t ac4;
	uint16_t ac5;
	uint16_t ac6;
	int16_t b1;
	int16_t b2;
	int16_t mb;
	int16_t mc;
	int16_t md;
};

class BMP180 {
		int fd;
		unsigned int oss;
		CalibReg cal_reg;
		int32_t P0;
		int32_t UT;
		int32_t UP;
		void LoadCalibration();
		void LoadDatasheetCalibration();
	public:
		BMP180(unsigned int o=BMP180_STANDARD, int rpi_version=2);
		~BMP180();
		int32_t getUT();
		int32_t getUP();
		void Read();
		double getTemperature();
		int32_t getPressure();
		double getAltitude();
		double getSeaLevelRelativeAltitude();
		double getSeaLevelPressure(double);
		void setBasePressure();
		int32_t getBasePressure();
};

#endif
