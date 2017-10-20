/************************************************************************************
* ADXL345.h - Header for ADXL345 accelerometer               	                   	*
* Copyright (c) 2014-2016 Antoine van de Cruyssen. All rights reserved             	*
*************************************************************************************
* Rev 1.0 - September 2016															*
* - Initial release																	*
*************************************************************************************
* This library is free software; you can redistribute it and/or						*
* modify it under the terms of the GNU Lesser General Public						*
* License as published by the Free Software Foundation; either						*
* version 2.1 of the License, or (at your option) any later version.				*
*																					*
* This library is distributed in the hope that it will be useful,					*
* but WITHOUT ANY WARRANTY; without even the implied warranty of					*
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU					*
* Lesser General Public License for more details.									*
* 																					*
* You should have received a copy of the GNU Lesser General Public					*
* License along with this library; if not, write to the Free Software				*
* Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA		*
************************************************************************************/
#ifndef ADXL345_h
#define ADXL345_h
//#define ADXL345_DEBUGMEAS
#include <Arduino.h> 
#include <I2C_Device.h>

#define X	0		// Variable X axis
#define Y	1		// Variable Y axis
#define D	2		// Variable degrees
#define I	3		// Variable info

#define HORIZONTAL	0
#define VERTICAL	1

struct AccelerometerDataSet {
	float XAxis = 0;					// X axis
	float YAxis = 0;					// Y axis
	float ZAxis = 0;					// Z axis
	float Degr = 0; 					// Caculated angle towards earth
};

class ADXL345 : public i2c_device
{
    public:
        ADXL345();
        void begin(uint8_t Address, boolean TWI_Begin);
		void getSensorData(struct AccelerometerDataSet *amds, uint8_t mode=HORIZONTAL);
};

#endif
