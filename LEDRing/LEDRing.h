/************************************************************************************
* LEDRing.h - Header for LEDRing 				          	                    	*
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
#ifndef LEDRing_h
#define LEDRing_h
#include <Arduino.h> 
#include <I2C_Device.h>
#include <PCA9635.h>

#define TOGGLE		0xF8
#define UPDATE		0x01
#define MEMORY		0x00 

#define DUALCENTER				0x7A
#define DUALLEFT				0x7B
#define DUALRIGHT				0x7C
#define CENTER					0x7D
#define LEFT					0x7E
#define RIGHT			 		0x7F
#define ARROWLEFT				0x44
#define ARROWRIGHT				0x45
#define ARROWUP					0x46
#define ARROWDOWN				0x47

#define BARGRAPH1				0x3D
#define BARGRAPH2       		0x3E
#define BARGRAPH3   			0x3F
#define BARGRAPH4				0x40
#define BARGRAPH5				0x41
#define BARGRAPH6				0x42
#define BARGRAPH7				0x43

#define ARROWDOWN1				0x44
#define ARROWDOWN2       		0x45
#define ARROWDOWN3   			0x46
#define ARROWDOWN4				0x47
#define ARROWDOWN5				0x48
#define ARROWDOWN6				0x49
#define ARROWDOWN7				0x4A

#define ARROWEAST1				0x4B
#define ARROWEAST2       		0x4C
#define ARROWEAST3   			0x4D
#define ARROWEAST4				0x4E
#define ARROWEAST5				0x4F
#define ARROWEAST6				0x50
#define ARROWEAST7				0x51

#define ARROWWEST1				0x52
#define ARROWWEST2       		0x53
#define ARROWWEST3   			0x54
#define ARROWWEST4				0x55
#define ARROWWEST5				0x56
#define ARROWWEST6				0x57
#define ARROWWEST7				0x58

#define ARROWUP1				0x59
#define ARROWUP2       			0x5A
#define ARROWUP3   				0x5B
#define ARROWUP4				0x5C
#define ARROWUP5				0x5D
#define ARROWUP6				0x5E
#define ARROWUP7				0x5F

#define FULLON					0x60

#define HORIZONTALBAR1			0x61
#define HORIZONTALBAR2       	0x62
#define HORIZONTALBAR3   		0x63
#define HORIZONTALBAR4			0x64
#define HORIZONTALBAR5			0x65
#define HORIZONTALBAR6			0x66
#define HORIZONTALBAR7			0x67

#define FRONT					0x41
#define BACK					0x42
#define BOTH					0x43

#define DEMOMAX					7

#define BLACK					0x000000
#define DARKRED 				0x4F0000
#define DARKBLUE				0x00004F
#define BLUE					0x0000FF
#define LIGHTBLUE				0x003FFF
#define CYAN					0x007FFF
#define GREEN					0x00FF00
#define LIGHTGREEN				0x1FDF1F
#define GREENYELLOW				0x7FFF00
#define YELLOW					0xFFFF00
#define ORANGE					0xFF3F00
#define RED						0xFF0000
#define MAGENTA					0xFF006F
#define WHITE					0xFFAFAF



class LEDRing : public i2c_device
{
    public:
        LEDRing();
		static void begin();
		static void reset(uint16_t HardwareType);
		static void startUpRing();
		static void updateNow();
		static void getTest(uint8_t i);
		static void singleLED(int8_t num, uint8_t a, uint8_t r, uint8_t g, uint8_t b, uint8_t mode=MEMORY);
		static void rangeLED(int8_t start, int8_t no, uint8_t a, uint8_t r, uint8_t g, uint8_t b, uint8_t mode=MEMORY);
		static void easyDemoTM(int8_t start, int8_t length, uint8_t br, uint32_t fclr, uint32_t bclr, uint8_t mode=UPDATE);
		static void showSymbol(uint8_t symbol, uint8_t step, uint8_t side, uint8_t br, uint32_t fclr, uint32_t bclr);
        //void begin(uint8_t Address, boolean TWI_Begin);
		//float getSensorData();
	private:
		static void updateRing(byte *arr);

};

#endif
