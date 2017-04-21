/************************************************************************************
* M41T00S.h - Header for M41T00S real time clock          	                    	*
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
#ifndef M41T00S_h
#define M41T00S_h
#include <Arduino.h> 
#include <I2C_Device.h>

class M41T00S : public i2c_device
{
    public:
        M41T00S();
        void begin(uint8_t Address, boolean TWI_Begin);
		static uint8_t decToBcd(uint8_t val);
		static uint8_t bcdToDec(uint8_t val);
		uint8_t *getData();
		void setData(uint8_t *d);
};

#endif
