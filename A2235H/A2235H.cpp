/************************************************************************************
* A2235H.cpp - Library for A2235H CO2 sensor			                              	*
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

#include "A2235H.h"
float GPSData[5];

// Constructor
A2235H::A2235H()
{

}

void A2235H::begin(uint8_t UARTI2CAddress, uint8_t UARTChannel) {
	SC16IS752::begin(UARTI2CAddress,false);
    SC16IS752::setup(UARTChannel,BAUD_9600,UART_8N1);
	SC16IS752::setGPIODir(0xFE);
    SC16IS752::writeGPIO(0);
	delay(50);
	SC16IS752::writeGPIO(SC16IS752::readGPIO() | 0x04);		// GPS reset high
	delay(50);
	A2235H::powerOn();
	delay(50);
	A2235H::disableNMEA();
}

void A2235H::disableNMEA() {
	for(uint8_t i=3;i<8;i++) {
		A2235H::queryGPS(i);
		delay(50);
	}
}

void A2235H::queryGPS(uint8_t msg) {
	char buffer[25];
	strcpy_P(buffer,(char*)pgm_read_word(&(GStr[msg]))); 
	SC16IS752::writeUART_charArray(GPSREC,sizeof(buffer),buffer);
}

boolean A2235H::checkPower() {
	if((SC16IS752::readGPIO() & 0x01) != 0) return true; else return false;
}

void A2235H::powerOn() {
	if(A2235H::checkPower()==false) {						// Power == Off, so switch on
		SC16IS752::writeGPIO(SC16IS752::readGPIO() | 0x02); // GPS enable pulse on
		delay(50);
		SC16IS752::writeGPIO(SC16IS752::readGPIO() & 0xFD); // GPS enable pulse off
		delay(50);
	}
}

void A2235H::powerOff() {
	if(A2235H::checkPower()==true) {						// Power == Off, so switch on
		SC16IS752::writeGPIO(SC16IS752::readGPIO() | 0x02); // GPS enable pulse on
		delay(100);
		SC16IS752::writeGPIO(SC16IS752::readGPIO() & 0xFD); // GPS enable pulse off
		delay(100);
	}
}

uint8_t A2235H::arrToNibble(uint8_t d[], uint8_t s) {
	uint8_t nibble = d[s];
	if(nibble>=0x30 && nibble<=0x39) {
		return (nibble-0x30);
	} else if(nibble>=0x41 && nibble<=0x46) {
		return (nibble-0x37);
	} else {
		return 0xFF;
	}
	
}

void A2235H::getData(struct GPSDataSet *gps, boolean dbg) {
	if(A2235H::checkPower()==false){
		gps->Fix=0;
		return;
	}

	uint8_t rx[64]={ };
	uint8_t x = SC16IS752::readUART_Array(GPSREC,rx);
	if(dbg==true){
		for(uint8_t y = 0; y<x; y++) {
			Serial.print ((char)rx[y]);
		}
	}

	uint8_t i = 0;
	uint8_t field[22]={ };
	uint8_t fieldcnt = 0;
	for (uint8_t cnt = 0; cnt < 21; cnt++) field[cnt]=0;

	for (i=0;i<x;i++){
		if (rx[i]==',' || rx[i]=='.') {
			field[fieldcnt] = i;
			fieldcnt++;
		}
		/*
		if (rx[i]=='*') {
			Serial.print(i);
			Serial.print(",");
			Serial.println(x);
		}
		*/
	}
/*	
	while (rx[i]!= '*' || i<64) {					// If "*" then finished, or end of buffer is reached
		if (rx[i]==',' || rx[i]=='.'){
			field[fieldcnt] = i;
			fieldcnt++;
		}
	i++;
	}
	Serial.println(i);
*/
	fieldcnt = 0;
	gps->DateTimeUpdate = 0;

	/************************************************************
	** $GPGSA,A,3,13,28,15,05,30,24,18,17,21,,,,1.4,0.9,1.1*3E **
	************************************************************/
	if(rx[3]=='G' && rx[4]=='S' && rx[5]=='A') {
		for (uint8_t cnt = 2; cnt < 13; cnt++) if ((field[cnt+1] - field[cnt]) > 1) fieldcnt++;
		gps->Siv=fieldcnt;
		gps->Fix=A2235H::arrToNibble(rx,field[1]+1);
		//GPSData[G_FIX] = A2235H::arrToNibble(rx,field[1]+1);
		if (field[15] - field[14] == 2) {
			gps->Dop = A2235H::arrToNibble(rx,field[14]+1)+ 
			   (float)(A2235H::arrToNibble(rx,field[15]+1)*.1);
			gps->DopDecimal = A2235H::arrToNibble(rx,field[14]+1);
			gps->DopFrac =    A2235H::arrToNibble(rx,field[15]+1);
	    }
		if (field[15] - field[14] == 3) {
			gps->Dop = A2235H::arrToNibble(rx,field[14]+1)*10+ 
					   A2235H::arrToNibble(rx,field[14]+2)+ 
			   (float)(A2235H::arrToNibble(rx,field[15]+1)*.1);
			gps->DopDecimal = A2235H::arrToNibble(rx,field[14]+1)*10+ 
					          A2235H::arrToNibble(rx,field[14]+2); 
			gps->DopFrac =    A2235H::arrToNibble(rx,field[15]+1);
	    }
	}
	
	/******************************************************
	** $GPGLL,5128.3541,N,00540.2729,E,093234.000,A,A*54 **
	******************************************************/
	uint32_t GPSLatFrac=0;
	uint32_t GPSLonFrac=0;
	if(rx[3]=='G' && rx[4]=='L' && rx[5]=='L') {
		
		if ((field[1] - field[0]) > 1) {
			gps->Latitude = (A2235H::arrToNibble(rx,field[0]+1)*10L)+
							(A2235H::arrToNibble(rx,field[0]+2));
			GPSLatFrac =	(A2235H::arrToNibble(rx,field[0]+3)*100000L)+
							(A2235H::arrToNibble(rx,field[0]+4)*10000L)+
							(A2235H::arrToNibble(rx,field[1]+1)*1000L)+
							(A2235H::arrToNibble(rx,field[1]+2)*100L)+
							(A2235H::arrToNibble(rx,field[1]+3)*10L)+
							(A2235H::arrToNibble(rx,field[1]+4));
			gps->LatitudeDegr = (int16_t)(gps->Latitude);
			gps->LatitudeFrac = ((float)GPSLatFrac / (float)600000L) * 1000000L;
			gps->Latitude+=((float)GPSLatFrac / (float)600000L);
			if (rx[field[2] + 1] == 'S') {
				gps->Latitude*=-1;
				gps->LatitudeDegr*=-1;
			}
		}
		if ((field[4] - field[3]) > 1) {
			gps->Longitude =(A2235H::arrToNibble(rx,field[3]+1)*100L)+
							(A2235H::arrToNibble(rx,field[3]+2)*10L)+
							(A2235H::arrToNibble(rx,field[3]+3));
			GPSLonFrac =	(A2235H::arrToNibble(rx,field[3]+4)*100000L)+
							(A2235H::arrToNibble(rx,field[3]+5)*10000L)+
							(A2235H::arrToNibble(rx,field[4]+1)*1000L)+
							(A2235H::arrToNibble(rx,field[4]+2)*100L)+
							(A2235H::arrToNibble(rx,field[4]+3)*10L)+
							(A2235H::arrToNibble(rx,field[4]+4));
			gps->LongitudeDegr = (int16_t)(gps->Longitude);
			gps->LongitudeFrac = ((float)GPSLonFrac / (float)600000L) * 1000000L;
			gps->Longitude+=((float)GPSLonFrac / (float)600000L);
			if (rx[field[5] + 1] == 'W') {
				gps->Longitude*=-1;
				gps->LongitudeDegr*=-1;

			}
		}

	}
	
	/**************************************
	** $GPZDA,095208.000,15,08,2017,,*58 **
	**************************************/
	if(rx[3]=='Z' && rx[4]=='D' && rx[5]=='A') {
		gps->DateTime[HOURS] =   (A2235H::arrToNibble(rx,field[0]+1)*10)+
							     (A2235H::arrToNibble(rx,field[0]+2));
		gps->DateTime[MINUTES] = (A2235H::arrToNibble(rx,field[0]+3)*10)+
							     (A2235H::arrToNibble(rx,field[0]+4));
		gps->DateTime[SECONDS] = (A2235H::arrToNibble(rx,field[0]+5)*10)+
							     (A2235H::arrToNibble(rx,field[0]+6));
		gps->DateTime[DATE] =    (A2235H::arrToNibble(rx,field[2]+1)*10)+
								 (A2235H::arrToNibble(rx,field[2]+2));
		gps->DateTime[MONTH] =	 (A2235H::arrToNibble(rx,field[3]+1)*10)+
								 (A2235H::arrToNibble(rx,field[3]+2));
		gps->DateTime[YEAR] =	 (A2235H::arrToNibble(rx,field[4]+3)*10)+
								 (A2235H::arrToNibble(rx,field[4]+4));
		gps->DateTimeUpdate = 1;
		gps->DateTime[DAY] = 0;
	}
}

