/************************************************************************************
* TMP112.cpp - Library for TMP112 temperature sensor                              	*
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

#include "TMP112.h"

// Constructor
TMP112::TMP112()
{

}

void TMP112::begin(uint8_t Address, boolean TWI_Begin) {
    i2c_device::begin(Address, TWI_Begin);
}

float TMP112::getSensorData() {
	byte Temperature[2];
    float CalcTemp;
	i2c_device::read_many(0x00, 2, Temperature);
	uint16_t TC=((((uint16_t)Temperature[0]<<8)|(uint16_t)Temperature[1])>>4);
	if (TC <= 0x07FF) {								// Value >= 0 °C
		CalcTemp=(float)TC*(float)0.0625;
	}
	else {											// Value < 0 °C
		CalcTemp=(float)(0x1000-TC)*(float)-0.0625;
	}
	return CalcTemp;
}