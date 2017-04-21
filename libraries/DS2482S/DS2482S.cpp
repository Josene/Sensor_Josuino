/************************************************************************************
* DS2482S.cpp - Library for DS2482S 1wire bridge                                   	*
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

#include "DS2482S.h"

// Constructor
DS2482S::DS2482S()
{

}

void DS2482S::begin(uint8_t Address, boolean TWI_Begin) {
    i2c_device::begin(Address, TWI_Begin);

	i2c_device::write_byte(0xF0);				// Device init

	i2c_device::write(0xD2,0xE1);				// Write config

	i2c_device::write_byte(0xB4);				// 1-wire reset

	}

uint8_t DS2482S::readByte(void) {
	byte result[2]={ };
	i2c_device::write(0xE1,0xE1);				// Set read pointer
	i2c_device::read_woreg(1,result);
	return result[0];
}

void DS2482S::write1Wire(uint8_t data) {
	i2c_device::write(0xA5,data);				// 1-wire write byte
}

void DS2482S::read1Wire(void) {
	i2c_device::write_byte(0x96);				// 1-wire generate read timeslot
}

void DS2482S::reset1Wire(void) {
	i2c_device::write_byte(0xB4);				// 1-wire reset
}

void DS2482S::requestDS18B20(void) {
	i2c_device::write_byte(0xB4);				// 1-wire reset
    delay(2);
	i2c_device::write(0xA5,0xCC);				// 1-wire write byte
    delay(1);
	i2c_device::write(0xA5,0x44);				// 1-wire write byte
    delay(1);
//	Serial.print("Req ");
}

float DS2482S::getSensorData(void) {
	uint8_t Datalsb = 0;
	uint8_t Datamsb = 0;
	uint8_t Checkup = 0;
	uint16_t Result = 0;
	byte result[2]={ };
	uint8_t ds18b20[8]={ };
	
//	Serial.print("Get ");

	i2c_device::write_byte(0xB4);				// 1-wire reset
    delay(2);
	i2c_device::write(0xA5,0xCC);				// 1-wire write byte
    delay(1);
	i2c_device::write(0xA5,0xBE);				// 1-wire write byte
    delay(1);

	for(uint8_t x=0; x<8; x++) {
		i2c_device::write_byte(0x96);				// 1-wire generate read timeslot
		delay(1);
  		i2c_device::write(0xE1,0xE1);				// Set read pointer
		delay(1);
		result[0]=0;
		i2c_device::read_woreg(1,result);
		ds18b20[x]=result[0];
		//Serial.print(ds18b20[x],HEX);
		//Serial.print(" ");
		delay(1);
	}

//	Serial.println("done.");

	if (ds18b20[7] != 0x10) return 0xFFFF;			// If byte 7 != 0x10, then error
	float CalcTemp;
	uint16_t TC=(((uint16_t)ds18b20[1]<<8)|(uint16_t)ds18b20[0]);
	if (TC <= 0x07FF) {								// Value >= 0 °C
		CalcTemp=(float)TC*(float)0.0625;
	}
	else {											// Value < 0 °C
		CalcTemp=(float)(0x1000-TC)*(float)-0.0625;
	}
	return CalcTemp;
	

}



