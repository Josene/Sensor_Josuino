/************************************************************************************
* IconRing.h - Header for IconRing 				          	                    	*
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
#ifndef IconRing_h
#define IconRing_h
#include <Arduino.h> 
#include <I2C_Device.h>
#include <PCA9635.h>

#define TOGGLE		0xF8
#define UPDATE		0x01
#define MEMORY		0x00 

class IconRing : public i2c_device
{
	public:
		IconRing();
		static void begin();
		static void getTest();
		static void updateNow();
		static void singleIcon(int8_t num, uint8_t a, uint8_t r=0xFF, uint8_t g=0xAF, uint8_t b=0xAF, uint8_t mode=UPDATE);
		static void allOff();
		static void startUpRing();
	private:
		static void updateIcons(byte *arr);
};

#endif
