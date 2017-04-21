/************************************************************************************
* BH1750.h - Header for BH1750 light sensor               	                    	*
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
#ifndef BH1750_h
#define BH1750_h
//#define BH1750_DEBUGMEAS
#include <Arduino.h> 
#include <I2C_Device.h>

static uint16_t BH1750value;

class BH1750 : public i2c_device
{
    public:
        BH1750();
        void begin(uint8_t Address, boolean TWI_Begin);
		uint16_t getSensorData();
};

#endif
