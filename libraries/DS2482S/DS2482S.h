/************************************************************************************
* DS2482S.h - Header for DS2482S 1wire bridge	          	                    	*
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
#ifndef DS2482S_h
#define DS2482S_h
#include <Arduino.h> 
#include <I2C_Device.h>

class DS2482S : public i2c_device
{
    public:
        DS2482S();
        void begin(uint8_t Address, boolean TWI_Begin);
		uint8_t readByte(void);
		void write1Wire(uint8_t data);
		void read1Wire(void);
		void reset1Wire(void);
		void requestDS18B20(void);
		float getSensorData(void);
};
		
#endif
