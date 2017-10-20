/************************************************************************************
* T6713.cpp - Library for T6713 CO2 sensor			                              	*
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

#include "T6713.h"

// Constructor
T6713::T6713()
{

}

void T6713::begin(uint8_t UARTI2CAddress, uint8_t UARTChannel) {
	SC16IS752::begin(UARTI2CAddress,false);
    SC16IS752::setup(UARTChannel,BAUD_19200,UART_8E1);
    SC16IS752::setGPIODir(0xFF);
//    i2c_device::begin(Address, TWI_Begin);
}

void T6713::askSensor() {
	uint8_t tx[8]={0x15,0x04,0x13,0x8B,0x00,0x01,0x46,0x70};
	SC16IS752::writeUART_Array(CO2,sizeof(tx),tx);
}

void T6713::askStatus() {
	uint8_t tx[8]={0x15,0x04,0x13,0x8A,0x00,0x01,0x17,0xB0};
	SC16IS752::writeUART_Array(CO2,sizeof(tx),tx);
}

void T6713::askCalibration() {
	uint8_t tx[8]={0x15,0x05,0x03,0xEC,0xFF,0x00,0x4E,0x9F};
	SC16IS752::writeUART_Array(CO2,sizeof(tx),tx);
}

boolean T6713::checkPower() {
	if((SC16IS752::readGPIO() & 0x20) != 0) return true; else return false;
}

void T6713::powerOn() {
	SC16IS752::writeGPIO(SC16IS752::readGPIO() | 0x20);
}

void T6713::powerOff() {
	SC16IS752::writeGPIO(SC16IS752::readGPIO() & 0xDF);
}

uint16_t T6713::getData(boolean dbg){
	uint8_t rx[8]={ };
	uint8_t x = SC16IS752::readUART_Array(CO2,rx);
	if(dbg==true){
		for(uint8_t y = 0; y<x; y++) {
			Serial.print (rx[y],HEX);
			Serial.print (",");
		}
	Serial.println("");
	}
	return ((uint16_t)rx[3] << 8) | rx[4];
}
	
