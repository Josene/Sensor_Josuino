/************************************************************************************
* PCB150011.h - Header for PCB150011 real time clock          	                    	*
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
#ifndef PCB150011_h
#define PCB150011_h
#include <Arduino.h> 
#include <I2C_Device.h>
#include <Josuino.h>

#define CUR 	0 		// Offset cur value in array = 0,  or 0 ~ 30
#define MAX 	31  	// Offset max value in array = 31, or 31 ~ 62
#define MIN 	62  	// Offset min value in array = 62, or 62 ~ 93
#define AVG 	93  	// Offset avg value in array = 93, or 93 ~ 124

class PCB150011 : public i2c_device
{
    public:
        PCB150011();
        void begin(uint8_t Address, boolean TWI_Begin);
		void setAmpFactor(uint16_t AmpFactor);
		void enableLED(void);
		void disableLED(void);
		void getThirdsSpectrum(uint8_t *OCT3);
		void clearThirdsSpectrum(uint8_t *OCT3);
		//static uint8_t decToBcd(uint8_t val);
		//static uint8_t bcdToDec(uint8_t val);
		//uint8_t *getData();
		//void setData(uint8_t *d);
};

#endif
