/************************************************************************************
* PCB150011.cpp - Library for PCB150011 soundpressuremodule                      	*
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

#include "PCB150011.h"

Utils PCB150011Utils;

int8_t Corr[31];
//uint8_t OCT3[124];		// All calculated audio values
/*
#define CUR 	0 		// Offset cur value in array = 0,  or 0 ~ 30
#define MAX 	31  	// Offset max value in array = 31, or 31 ~ 62
#define MIN 	62  	// Offset min value in array = 62, or 62 ~ 93
#define AVG 	93  	// Offset avg value in array = 93, or 93 ~ 124
*/

// Constructor
PCB150011::PCB150011()
{

}

void PCB150011::begin(uint8_t Address, boolean TWI_Begin) {
    i2c_device::begin(Address, TWI_Begin);
}

void PCB150011::setAmpFactor(uint16_t AmpFactor) {
	uint8_t af[2]={0,0};
	af[1] = ((AmpFactor & 0xFF00) >> 8);		// MSB
	af[0] = (AmpFactor & 0x00FF);				// LSB
	i2c_device::write_many(0x07, 2, af);		// Write Ampfactor to 0x07 LSB and 0x08 MSB
}

void PCB150011::enableLED(void) {
	i2c_device::write(0x09,0x0F);
}

void PCB150011::disableLED(void) {
	i2c_device::write(0x09,0x00);
}

void PCB150011::clearThirdsSpectrum(uint8_t *OCT3) {
	for(uint8_t x=0;x<31;x++) {
		Corr[x]=PCB150011Utils.readEEPROM(0x70+x,1);
		OCT3[x+CUR]=0x00;
		OCT3[x+MAX]=0x00;
		OCT3[x+MIN]=0xFF;
		OCT3[x+AVG]=0x00;
	}
	OCT3[PKT]=0;	
}


void PCB150011::getThirdsSpectrum(uint8_t *OCT3) {
	i2c_device::read_many(0x4D, 31, OCT3);		// Read current data in current part of array (0~30)
	OCT3[0]=0;
	OCT3[PKT]=0;
	for(uint8_t x=0;x<31;x++) {
		if(OCT3[PKT]<OCT3[x]) OCT3[PKT]=OCT3[x];
		OCT3[x]+=Corr[x];
		if(OCT3[x]>128)OCT3[x]=0;
		if(OCT3[x]>OCT3[x+MAX]) OCT3[x+MAX]=OCT3[x];
		if(OCT3[x]<OCT3[x+MIN]) OCT3[x+MIN]=OCT3[x];
		OCT3[x+AVG]=((OCT3[x+AVG]+OCT3[x])/2);
	}
	
	
	
/*
	Serial.print("Cur ");
	for(uint8_t x=0;x<31;x++) {
		if((x%10)==0){
			Serial.print("[");
			Serial.print(x);
			Serial.print("] ");
		}
		Serial.print(OCT3[x]);
		Serial.print(" ");
	}
	Serial.println();

	Serial.print("Max ");
	for(uint8_t x=0;x<31;x++) {
		if((x%10)==0){
			Serial.print("[");
			Serial.print(x);
			Serial.print("] ");
		}
		Serial.print(OCT3[x+MAX]);
		Serial.print(" ");
	}
	Serial.println();

	Serial.print("Min ");
	for(uint8_t x=0;x<31;x++) {
		if((x%10)==0){
			Serial.print("[");
			Serial.print(x);
			Serial.print("] ");
		}
		Serial.print(OCT3[x+MIN]);
		Serial.print(" ");
	}
	Serial.println();

	Serial.print("Avg ");
	for(uint8_t x=0;x<31;x++) {
		if((x%10)==0){
			Serial.print("[");
			Serial.print(x);
			Serial.print("] ");
		}
		Serial.print(OCT3[x+AVG]);
		Serial.print(" ");
	}
	Serial.println();

//	return OCT3;
*/
}