/************************************************************************************
* AA025E64.cpp - Library for AA025E64         				                      	*
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

#include "AA025E64.h"

// Constructor
AA025E64::AA025E64()
{

}

void AA025E64::begin(uint8_t Address, boolean TWI_Begin) {
    i2c_device::begin(Address, TWI_Begin);
}

void AA025E64::getID(uint32_t *ID) {
	uint8_t data[8]= { };
	i2c_device::read_many(0xF8, 8, data);
	ID[1] = ((uint32_t)(data[0])<<24)|((uint32_t)(data[1])<<16)|((uint32_t)(data[2])<<8)|((uint32_t)(data[3]));
	ID[0] = ((uint32_t)(data[4])<<24)|((uint32_t)(data[5])<<16)|((uint32_t)(data[6])<<8)|((uint32_t)(data[7]));
}

void AA025E64::dump() {
	uint8_t data[16]= { };
	for(uint16_t i=0;i<256; i+=16){
		i2c_device::read_many(i, 16, data);
		for(uint8_t j=0;j<=15;j++) {
			if(data[j]<16) Serial.print("0");
			Serial.print(data[j],HEX);
			Serial.print(" ");
		}
		Serial.println();
	}
}
