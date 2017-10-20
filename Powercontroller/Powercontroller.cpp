/************************************************************************************
* Powercontroller.cpp - Library for Powercontroller                               	*
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

#include "Powercontroller.h"

// Constructor
Powercontroller::Powercontroller()
{

}

void Powercontroller::begin(uint8_t Address, boolean TWI_Begin) {
    i2c_device::begin(Address, TWI_Begin);
	//Serial.println(F("Powercontroller init OK"));
}

void Powercontroller::setOutput(uint8_t Register, uint8_t Data) {
	i2c_device::write(Register,Data);
}

void Powercontroller::writeISP(uint32_t ISPdest, uint8_t ISPlen, uint8_t *d) {
	Serial.print("\nISP for "); Serial.print(ISPdest,HEX);
	Serial.print(" with len:"); Serial.println(ISPlen);
	Powercontroller::setMode(1);
	delay(20);
	for(uint8_t i=0;i<ISPlen;i++) {
		Serial.print(d[i],HEX);
		Serial.print(" ");
	}
	i2c_device::write_many(0x00, ISPlen, d);
	delay(60);
	uint8_t stat[1];
	Serial.print("\nWait for answer:");

	for(uint8_t i=0; i<10;i++){
		i2c_device::read_woreg(1,stat);
		delay(50);
		Serial.print(" ");
		Serial.print(stat[0],HEX);
		if(stat[0]==0x49) break;
	}
	
	if(stat[0]==0x49) {
		Serial.println("\nGot 0x49, now reading");
		uint8_t data[20];
		i2c_device::read_woreg(16,data);

		Serial.print("Got 16 bytes:");
		for(uint8_t i=0; i<16;i++){
			Serial.print(data[i],HEX);
			Serial.print(" ");
		}
		Serial.println();
	}
	
	
	Powercontroller::setMode(0);
}

uint8_t Powercontroller::getMode() {
	return i2c_device::read(0xF1);
}

void Powercontroller::setMode(uint8_t mode) {
	if(mode==1) Serial.println("Set to ISP mode");
	if(mode==0) Serial.println("Set to normal mode");
	
	i2c_device::write(0xF1,mode);
}

void Powercontroller::getData(struct PSUDataSet *psu, boolean dbg) {
	uint8_t d[14]= { };
	i2c_device::read_many(0x2E, 14, d);
	
	if(dbg){
		for(uint8_t i=0; i<14; i++){
			Serial.print("R:");
			Serial.print((i+0x2E),HEX);
			Serial.print("=0x");
			if(d[i]<16) Serial.print("0");
			Serial.print(d[i],HEX);
			Serial.print(" ");
		}
		Serial.println();
	}

	psu->FunctionMode=d[1];
	psu->BatteryVoltage=((uint16_t)(d[2])<<8)|d[3];
	psu->PSUVoltage=((uint16_t)(d[4])<<8)|d[5];
	psu->BatteryCurrent=((uint16_t)(d[6])<<8)|d[7];
	psu->PSUCurrent=((uint16_t)(d[8])<<8)|d[9];
	psu->PSUTemperature=((uint16_t)(d[10])<<8)|d[11];
	psu->BatteryCoulombs=((uint16_t)(d[12])<<8)|d[13];
}

void Powercontroller::getTest() {
	uint8_t d[9]= { };
	i2c_device::read_many(0x2F, 13, d);
	for(uint8_t i=0; i<13; i++){
		Serial.print("R:");
		Serial.print((i+0x2F),HEX);
		Serial.print("=0x");
		if(d[i]<16) Serial.print("0");
		Serial.print(d[i],HEX);
		Serial.print(" ");
	}
	Serial.println();
}



