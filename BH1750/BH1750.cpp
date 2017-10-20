/************************************************************************************
* BH1750.cpp - Library for BH1750 light sensor		                              	*
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

#include "BH1750.h"

// Constructor
BH1750::BH1750()
{

}

void BH1750::begin(uint8_t Address, boolean TWI_Begin) {
    i2c_device::begin(Address, TWI_Begin);
	delay(200);
	i2c_device::write_byte(0x10);		// Continuously H-Resolution Mode 
	delay(200);

}

uint16_t BH1750::getSensorData() {
	byte RawSensorData[2];
	
    i2c_device::read_woreg(2,RawSensorData);

#ifdef BH1750_DEBUGMEAS
	for(int x=0; x<2; x++) {
		if (RawSensorData[x]<16) Serial.print("0");
		Serial.print(RawSensorData[x],HEX);
		Serial.print(" ");
	}
	Serial.print("\n");
#endif	
	
//	i2c_device::read_many(0xF7, 8, RawSensorData);
//	long RawPressure=((((long)(RawSensorData[0])<<16)|((long)(RawSensorData[1])<<8)|(long)(RawSensorData[2]))>>4);
//	long RawTemperature=((((long)RawSensorData[3]<<16)|((long)RawSensorData[4]<<8)|(long)RawSensorData[5])>>4);
	return (((uint16_t)RawSensorData[0]<<8)|(uint16_t)RawSensorData[1]);

}


