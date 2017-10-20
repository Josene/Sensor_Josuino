/************************************************************************************
* A2235H.h - Header for A2235H GPS sensor			      	                    	*
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
#ifndef A2235H_h
#define A2235H_h
#include <Arduino.h> 
#include <SC16IS752.h>

const PROGMEM char GPGSA_QUERY[] = "$PSRF103,2,1,0,1*27\r\n";
const PROGMEM char GPGLL_QUERY[] = "$PSRF103,1,1,0,1*24\r\n";
const PROGMEM char GPZDA_QUERY[] = "$PSRF103,8,1,0,1*2D\r\n";
const PROGMEM char GPGGA_OFF[] = "$PSRF103,0,0,0,1*24\r\n";
const PROGMEM char GPGLL_OFF[] = "$PSRF103,1,0,0,1*25\r\n";
const PROGMEM char GPGSA_OFF[] = "$PSRF103,2,0,0,1*26\r\n";
const PROGMEM char GPGSV_OFF[] = "$PSRF103,3,0,0,1*27\r\n";
const PROGMEM char GPRMC_OFF[] = "$PSRF103,4,0,0,1*20\r\n";

const char* const GStr[] PROGMEM = {GPGSA_QUERY,GPGLL_QUERY,GPZDA_QUERY,GPGGA_OFF,GPGLL_OFF,GPGSA_OFF,GPGSV_OFF,GPRMC_OFF,};

struct GPSDataSet {
	float Longitude = 0;				// Received longitude
	float Latitude = 0;					// Received latitude
	int16_t LongitudeDegr = 0;			// Received longitude degrees
	uint32_t LongitudeFrac = 0;			// Received longitude fraction
	int16_t LatitudeDegr = 0;			// Received latitude degrees
	uint32_t LatitudeFrac = 0;			// Received latitude fraction
	uint8_t Fix = 0;					// Fix mode
	uint8_t Siv = 0;					// Sats in view
	float Dop = 0;						// Dillution of precision
	uint8_t DopDecimal = 0;
	uint8_t DopFrac = 0;
	uint8_t Mode = 0;					// Current mode of GPS receiver
	uint8_t DateTime[7] = {0,0,0,0,0,0,0};
	uint8_t DateTimeUpdate = 0;
	//uint32_t Time = 0;
	//uint32_t Date = 0;
};

#define G_GPGSA_QUERY 	0
#define G_GPGLL_QUERY 	1
#define G_GPZDA_QUERY 	2
#define G_GPGGA_OFF 	3
#define G_GPGLL_OFF 	4
#define G_GPGSA_OFF 	5
#define G_GPGSV_OFF 	6
#define G_GPRMC_OFF 	7

#define G_LAT			0
#define G_LON			1
#define G_FIX			2
#define G_SIV			3
#define G_DOP			4

#define SECONDS 	0x00
#define MINUTES  	0x01
#define HOURS  		0x02
#define DAY  		0x03
#define DATE  		0x04
#define MONTH  		0x05
#define YEAR	  	0x06


class A2235H : public SC16IS752
{
    public:
        A2235H();
        void begin(uint8_t UARTI2CAddress, uint8_t UARTChannel);
		void disableNMEA();
		void queryGPS(uint8_t msg);
		uint8_t arrToNibble(uint8_t d[], uint8_t s);
		void getData(struct GPSDataSet *gps, boolean dbg=false);
		boolean checkPower();
		void powerOn();
		void powerOff();
};

#endif
