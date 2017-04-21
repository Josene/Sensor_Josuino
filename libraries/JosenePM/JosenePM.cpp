/************************************************************************************
* JosenePM.cpp - Library for JosenePM PM sensor				                           	*
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

#include "JosenePM.h"
uint16_t pm[2]={ };

// Constructor
JosenePM::JosenePM()
{

}

void JosenePM::begin(uint8_t UARTI2CAddress, uint8_t UARTChannel) {
	SC16IS752::begin(UARTI2CAddress,false);
	SC16IS752::setup(UARTChannel,BAUD_9600,UART_8N1);
    SC16IS752::setGPIODir(0xFF);
}

boolean JosenePM::checkPower() {
	if((SC16IS752::readGPIO() & 0x10) != 0) return true; else return false;
}

void JosenePM::powerOn() {
	SC16IS752::writeGPIO(SC16IS752::readGPIO() | 0x10);
}

void JosenePM::powerOff() {
	SC16IS752::writeGPIO(SC16IS752::readGPIO() & 0xEF);
}

uint16_t * JosenePM::getData(boolean dbg){
	uint8_t rx[63]={ };
	uint8_t x = SC16IS752::readUART_Array(PM,rx);
	if(dbg==true){
		for(uint8_t y = 0; y<x; y++) {
			Serial.print (rx[y],HEX);
			Serial.print (",");
		}
	}
	if(x==10 && rx[0]==0xAA && rx[1]==0xC0 && rx[9]==0xAB) {
		pm[0]=((uint16_t)rx[5] << 8) | rx[4];
		pm[1]=((uint16_t)rx[3] << 8) | rx[2];
		//pm[1];
		if(dbg==true) {
			Serial.print("PM10:"); Serial.print(pm[0]); Serial.print(" ");
			Serial.print("PM2.5:"); Serial.print(pm[1]); Serial.print(" ");
			Serial.println("");
		} 
	} else {
		if(dbg==true) Serial.println(" ?");
	}
	return pm;
}
	
