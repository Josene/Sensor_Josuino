/************************************************************************************
* IconRing.cpp - Library for IconRing 				                            	*
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

#include "IconRing.h"
#include "Josuino.h"

pca9635 Icons;
Utils IconRingUtil;
byte IconLED[16];

/************************************************************************************
*************************************************************************************
* IconRing class																	*
*************************************************************************************
************************************************************************************/
IconRing::IconRing(void) {
	
}

void IconRing::getTest() {
	Icons.getTest();
}

void IconRing::begin() {
    Icons.begin(0x1A, false);                 	// Container for icons ring on top
	Icons.set_driver_mode(0);
	Icons.setMode(0x80,0x05);					// From now on, don't respond to all call address
}

/***************************************************
* IconRing, set single icon, defaults update,      *
* in case of icon 5 and 6 RGB values can be given, *
* if ommitted, these LEDs become white 			   *	
***************************************************/
void IconRing::singleIcon(int8_t num, uint8_t a, uint8_t r, uint8_t g, uint8_t b, uint8_t mode) {
	if(a>0x0F) return;

	if(num>=0x00 && num<=0x0B) {
		uint8_t A=0;
		if(a!=0){if(a%2==0) A=(1<<(a/2)+1)>>1; else A=(3<<(a/2))>>1;}	

		if(num>=0x00 && num<=0x04) {
			IconLED[num] = A;
		}
		if(num==0x05) {
			IconLED[num+0] = (((r*(A+1))&0xFF00)>>8);
			IconLED[num+1] = (((g*(A+1))&0xFF00)>>8);
			IconLED[num+2] = (((b*(A+1))&0xFF00)>>8);
		}
		if(num==0x06) {
			IconLED[num+2] = (((r*(A+1))&0xFF00)>>8);
			IconLED[num+3] = (((g*(A+1))&0xFF00)>>8);
			IconLED[num+4] = (((b*(A+1))&0xFF00)>>8);
		}
		if(num>=0x07 && num<=0x0B) {
			IconLED[num+4] = A;
		}
		
		if (mode==UPDATE) IconRing::updateIcons(IconLED);
	}
}

/*********************
* Turn Off all icons *
*********************/
void IconRing::allOff() {
	for(uint8_t i=0;i<16;i++){
		IconLED[i]=0;
	}
	IconRing::updateIcons(IconLED);
}

/********************
* Startup animation *
********************/
void IconRing::startUpRing() {
	for (int x=0;x<=5;x++) {
		int z=(x*-1)+11;
        for (int y=0;y<=15;y++) {
            IconRing::singleIcon(x, y, 0xFF, 0xAF, 0xAF, MEMORY);
            IconRing::singleIcon(z, y, 0xFF, 0xAF, 0xAF, UPDATE);
			delay(10);
        }
	}
	for (int x=0;x<=5;x++) {
		int z=(x*-1)+11;
        for (int y=15;y>-1;y--) {
            IconRing::singleIcon(x, y, 0xFF, 0xAF, 0xAF, MEMORY);
            IconRing::singleIcon(z, y, 0xFF, 0xAF, 0xAF, UPDATE);
			delay(10);
        }
	}
}

/*****************************************************************************************
* External command to update Iconring, this is default done by mode=UPDATE in SingleIcon *
*****************************************************************************************/
void IconRing::updateNow() {
	IconRing::updateIcons(IconLED);	
}

/**************************************
* Private function to update hardware *
**************************************/
void IconRing::updateIcons(byte *arr) {
	for (byte ledno = 0; ledno < 16; ledno++) {
		IconRingUtil.ctrlDebugLEDs(LEDI2C, ON);
		Icons.set_led_pwm(ledno, arr[ledno]);
		IconRingUtil.ctrlDebugLEDs(LEDI2C, OFF);
	}
}