/************************************************************************************
* LEDRing.cpp - Library for LEDRing 				                            	*
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

#include "LEDRing.h"
#include "Josuino.h"

pca9635 RGB[6];
Utils LEDRingUtil;
byte RGBLED[96];

// Constructor
LEDRing::LEDRing()
{

}

void LEDRing::begin() {
	RGB[0].begin(0x12, false);					// Container for segment 1 of LED ring
    RGB[1].begin(0x13, false);					// Container for segment 2 of LED ring
    RGB[2].begin(0x14, false);					// Container for segment 3 of LED ring
    RGB[3].begin(0x15, false);					// Container for segment 4 of LED ring
    RGB[4].begin(0x16, false);					// Container for segment 5 of LED ring
    RGB[5].begin(0x17, false);					// Container for segment 6 of LED ring
}

void LEDRing::reset(uint16_t HardwareType) {
	if(HardwareType==0x1711) {
		for(uint8_t i=0;i<6;i++){
			RGB[i].setMode(0x81,0x15);
		}
	}
}

/********************
* Startup animation *
********************/
void LEDRing::startUpRing() {
    for (int x=-16;x<=64;x++) {
        for (int y=0;y<=15;y++) {
            LEDRing::singleLED(x+y, y, 0x7F, 0x00, 0x7F);
            LEDRing::singleLED((x-16)+y, y, 0x7F, 0x7F, 0x00);
            LEDRing::singleLED((x-32)+y, y, 0x00, 0x7F, 0x7F);
        }
        LEDRing::updateNow();
        delay(25);
    }	
}

/*********************************************
* LEDRing, set single led, default no update *
*********************************************/
void LEDRing::singleLED(int8_t num, uint8_t a, uint8_t r, uint8_t g, uint8_t b, uint8_t mode) {
	if(a>0x0F) return;

	if(num>=0x00 && num<=0x1F) {
		uint8_t A=0;
		if(a!=0){if(a%2==0) A=(1<<(a/2)+1)>>1; else A=(3<<(a/2))>>1;}	
		RGBLED[(num*3)+0] = (((r*(A+1))&0xFF00)>>8);
		RGBLED[(num*3)+1] = (((g*(A+1))&0xFF00)>>8);
		RGBLED[(num*3)+2] = (((b*(A+1))&0xFF00)>>8);
		if (mode==UPDATE) LEDRing::updateRing(RGBLED);
	}
}

/*********************************************
* LEDRing, set single led, default no update *
*********************************************/
void LEDRing::rangeLED(int8_t start, int8_t no, uint8_t a, uint8_t r, uint8_t g, uint8_t b, uint8_t mode) {
	if(a>0x0F) return;
	if(start<0x00 || start>0x1F) return;
	if(no<0x01) return;
	no--;
	int8_t end=start+no;
	if(end>0x1F) return;
	
	for(int16_t num=start; num<=end; num++) {
		uint8_t A=0;
		if(a!=0){if(a%2==0) A=(1<<(a/2)+1)>>1; else A=(3<<(a/2))>>1;}	
		RGBLED[(num*3)+0] = (((r*(A+1))&0xFF00)>>8);
		RGBLED[(num*3)+1] = (((g*(A+1))&0xFF00)>>8);
		RGBLED[(num*3)+2] = (((b*(A+1))&0xFF00)>>8);
	}
	if (mode==UPDATE) LEDRing::updateRing(RGBLED);
}

/*********************************************************************************************
* External command to update LEDRing, can also be done by mode=UPDATE in single LED function *
*********************************************************************************************/
void LEDRing::updateNow() {
	extern uint32_t HardwareType;
	LEDRing::updateRing(RGBLED);	
	LEDRing::reset(HardwareType);
}

void LEDRing::getTest(uint8_t i) {
	RGB[i].getTest();
}

/**************************************
* Private function to update hardware *
**************************************/
void LEDRing::updateRing(byte *arr) {
    for (byte rgbno = 0; rgbno < 6; rgbno++) {
        for (byte ledno = 0; ledno < 16; ledno++) {
            byte cnt = (rgbno*16)+(ledno);
			LEDRingUtil.ctrlDebugLEDs(LEDI2C, ON);
			RGB[rgbno].set_led_pwm(ledno, arr[cnt]);
			LEDRingUtil.ctrlDebugLEDs(LEDI2C, OFF);
        }
    }
	extern uint32_t HardwareType;
	LEDRing::reset(HardwareType);

}

void LEDRing::showSymbol(uint8_t symbol, uint8_t step, uint8_t side, uint8_t br, uint32_t fclr, uint32_t bclr) {
	uint8_t offset = (side-FRONT) * 0x10;
	switch(symbol){
		case ARROWLEFT :
			LEDRing::rangeLED(0x00+offset,0x10,br,(uint8_t)((bclr&0xFF0000)>>16),(uint8_t)((bclr&0xFF00)>>8),(uint8_t)(bclr&0xFF),MEMORY); 								// Background color
			switch(step) {
				case 7:
					LEDRing::rangeLED(0x06 + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY);
					break;
				case 6:
					LEDRing::rangeLED(0x03 + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					LEDRing::rangeLED(0x06 + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					LEDRing::rangeLED(0x0A + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					break;
				case 5:
					LEDRing::rangeLED(0x01 + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					LEDRing::rangeLED(0x03 + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					LEDRing::rangeLED(0x06 + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					LEDRing::rangeLED(0x0A + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					LEDRing::rangeLED(0x0D + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					break;
				case 4:
					LEDRing::rangeLED(0x00 + offset,0x02,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					LEDRing::rangeLED(0x03 + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					LEDRing::rangeLED(0x06 + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					LEDRing::rangeLED(0x0A + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					LEDRing::rangeLED(0x0D + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					LEDRing::rangeLED(0x0F + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					break;
				case 3:
					LEDRing::rangeLED(0x02 + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					LEDRing::rangeLED(0x04 + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					LEDRing::rangeLED(0x07 + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					LEDRing::rangeLED(0x0B + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					LEDRing::rangeLED(0x0E + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					break;
				case 2:
					LEDRing::rangeLED(0x05 + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					LEDRing::rangeLED(0x08 + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					LEDRing::rangeLED(0x0C + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					break;
				case 1:
					LEDRing::rangeLED(0x09 + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					break;
			}
			break;
		case ARROWRIGHT :
			LEDRing::rangeLED(0x00+offset,0x10,br,(uint8_t)((bclr&0xFF0000)>>16),(uint8_t)((bclr&0xFF00)>>8),(uint8_t)(bclr&0xFF),MEMORY); 								// Background color
			switch(step) {
				case 7:
					LEDRing::rangeLED(0x09 + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					break;
				case 6:
					LEDRing::rangeLED(0x05 + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					LEDRing::rangeLED(0x09 + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					LEDRing::rangeLED(0x0C + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					break;
				case 5:
					LEDRing::rangeLED(0x02 + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					LEDRing::rangeLED(0x05 + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					LEDRing::rangeLED(0x09 + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					LEDRing::rangeLED(0x0C + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					LEDRing::rangeLED(0x0E + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					break;
				case 4:
					LEDRing::rangeLED(0x00 + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					LEDRing::rangeLED(0x02 + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					LEDRing::rangeLED(0x05 + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					LEDRing::rangeLED(0x09 + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					LEDRing::rangeLED(0x0C + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					LEDRing::rangeLED(0x0E + offset,0x02,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					break;
				case 3:
					LEDRing::rangeLED(0x01 + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					LEDRing::rangeLED(0x04 + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					LEDRing::rangeLED(0x08 + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					LEDRing::rangeLED(0x0B + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					LEDRing::rangeLED(0x0D + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					break;
				case 2:
					LEDRing::rangeLED(0x03 + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					LEDRing::rangeLED(0x07 + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					LEDRing::rangeLED(0x0A + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					break;
				case 1:
					LEDRing::rangeLED(0x06 + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					break;
			}
			break;
		case ARROWUP :
			LEDRing::rangeLED(0x00+offset,0x10,br,(uint8_t)((bclr&0xFF0000)>>16),(uint8_t)((bclr&0xFF00)>>8),(uint8_t)(bclr&0xFF),MEMORY); 								// Background color
			switch(step) {
				case 7:
					LEDRing::rangeLED(0x00 + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					break;
				case 6:
					LEDRing::rangeLED(0x00 + offset,0x03,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					break;
				case 5:
					LEDRing::rangeLED(0x00 + offset,0x04,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					LEDRing::rangeLED(0x05 + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					break;
				case 4:
					LEDRing::rangeLED(0x00 + offset,0x04,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					LEDRing::rangeLED(0x05 + offset,0x02,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					LEDRing::rangeLED(0x09 + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					break;
				case 3:
					LEDRing::rangeLED(0x04 + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					LEDRing::rangeLED(0x07 + offset,0x02,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					LEDRing::rangeLED(0x0A + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					LEDRing::rangeLED(0x0C + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					break;
				case 2:
					LEDRing::rangeLED(0x0B + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					LEDRing::rangeLED(0x0D + offset,0x02,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					break;
				case 1:
					LEDRing::rangeLED(0x0F + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					break;
			}
			break;
		case ARROWDOWN :
			LEDRing::rangeLED(0x00+offset,0x10,br,(uint8_t)((bclr&0xFF0000)>>16),(uint8_t)((bclr&0xFF00)>>8),(uint8_t)(bclr&0xFF),MEMORY); 								// Background color
			switch(step) {
				case 7:
					LEDRing::rangeLED(0x0F + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					break;
				case 6:
					LEDRing::rangeLED(0x0D + offset,0x03,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					break;
				case 5:
					LEDRing::rangeLED(0x0A + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					LEDRing::rangeLED(0x0C + offset,0x04,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					break;
				case 4:
					LEDRing::rangeLED(0x06 + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					LEDRing::rangeLED(0x09 + offset,0x02,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					LEDRing::rangeLED(0x0C + offset,0x04,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					break;
				case 3:
					LEDRing::rangeLED(0x03 + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					LEDRing::rangeLED(0x05 + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					LEDRing::rangeLED(0x07 + offset,0x02,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					LEDRing::rangeLED(0x0B + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					break;
				case 2:
					LEDRing::rangeLED(0x01 + offset,0x02,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					LEDRing::rangeLED(0x04 + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					break;
				case 1:
					LEDRing::rangeLED(0x00 + offset,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
					break;
			}
			break;
	}
}

/************************************************************************************************************************
* Easy demo code, provides, start en length, and fixed colors. Custom values in format 0x00RRGGBB are also possible. 	*
*************************************************************************************************************************
* Start value can be 0x00, 0x1F, or one of the values in the switch case in the routine                              	*
* Default colors for demo are now:																				    	*
* #define BLACK			0x000000																						*
* #define DARKRED 		0x4F0000																						*
* #define DARKBLUE		0x00004F																						*
* #define BLUE			0x0000FF																						*
* #define CYAN			0x007FFF																						*
* #define GREEN			0x00FF00																						*
* #define GREENYELLOW	0x7FFF00																						*
* #define YELLOW		0xFFFF00																						*
* #define ORANGE		0xFF3F00																						*
* #define RED			0xFF0000																						*
* #define MAGENTA		0xFF006F																						*
* #define WHITE			0xFFAFAF																						*
************************************************************************************************************************/
void LEDRing::easyDemoTM(int8_t start, int8_t length, uint8_t br, uint32_t fclr, uint32_t bclr, uint8_t mode) {
	switch (start) {
		case DUALRIGHT :
			LEDRing::rangeLED(0x00,0x20,br,(uint8_t)((bclr&0xFF0000)>>16),(uint8_t)((bclr&0xFF00)>>8),(uint8_t)(bclr&0xFF),MEMORY); 					// Background color
			LEDRing::rangeLED(0x00,(length*2),br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),UPDATE); 				// Foreground color
			break;
		case DUALCENTER :
			LEDRing::rangeLED(0x00,0x20,br,(uint8_t)((bclr&0xFF0000)>>16),(uint8_t)((bclr&0xFF00)>>8),(uint8_t)(bclr&0xFF),MEMORY); 					// Background color
			LEDRing::rangeLED(((length*-1)+16),(length*2),br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),UPDATE); 	// Foreground color
			break;
		case DUALLEFT :
			LEDRing::rangeLED(0x00,0x20,br,(uint8_t)((bclr&0xFF0000)>>16),(uint8_t)((bclr&0xFF00)>>8),(uint8_t)(bclr&0xFF),MEMORY); 					// Background color
			LEDRing::rangeLED(((length*-2)+32),(length*2),br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),UPDATE); 	// Foreground color
			break;
		case RIGHT :
			LEDRing::rangeLED(0x00,0x20,br,(uint8_t)((bclr&0xFF0000)>>16),(uint8_t)((bclr&0xFF00)>>8),(uint8_t)(bclr&0xFF),MEMORY); 					// Background color
			LEDRing::rangeLED(0x00,length,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),UPDATE); 					// Foreground color
			break;
		case LEFT :
			LEDRing::rangeLED(0x00,0x20,br,(uint8_t)((bclr&0xFF0000)>>16),(uint8_t)((bclr&0xFF00)>>8),(uint8_t)(bclr&0xFF),MEMORY); 					// Background color
			LEDRing::rangeLED(((length*-1)+32),length,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),UPDATE); 		// Foreground color
			break;
		case HORIZONTALBAR7 :
			if(length==FRONT) {
				LEDRing::rangeLED(0x00,0x10,br,(uint8_t)((bclr&0xFF0000)>>16),(uint8_t)((bclr&0xFF00)>>8),(uint8_t)(bclr&0xFF),MEMORY); 					// Background color
				LEDRing::rangeLED(0x00,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),UPDATE); 					// Foreground color
			} else if(length==BACK) {
				LEDRing::rangeLED(0x10,0x10,br,(uint8_t)((bclr&0xFF0000)>>16),(uint8_t)((bclr&0xFF00)>>8),(uint8_t)(bclr&0xFF),MEMORY); 					// Background color
				LEDRing::rangeLED(0x10,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),UPDATE); 					// Foreground color
			} else if(length==BOTH) {
				LEDRing::rangeLED(0x00,0x20,br,(uint8_t)((bclr&0xFF0000)>>16),(uint8_t)((bclr&0xFF00)>>8),(uint8_t)(bclr&0xFF),MEMORY); 					// Background color
				LEDRing::rangeLED(0x00,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
				LEDRing::rangeLED(0x10,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),UPDATE); 					// Foreground color
			}
			break;
		case HORIZONTALBAR6 :
			if(length==FRONT) {
				LEDRing::rangeLED(0x00,0x10,br,(uint8_t)((bclr&0xFF0000)>>16),(uint8_t)((bclr&0xFF00)>>8),(uint8_t)(bclr&0xFF),MEMORY); 					// Background color
				LEDRing::rangeLED(0x01,0x02,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),UPDATE); 					// Foreground color
			} else if(length==BACK) {
				LEDRing::rangeLED(0x10,0x10,br,(uint8_t)((bclr&0xFF0000)>>16),(uint8_t)((bclr&0xFF00)>>8),(uint8_t)(bclr&0xFF),MEMORY); 					// Background color
				LEDRing::rangeLED(0x11,0x02,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),UPDATE); 					// Foreground color
			} else if(length==BOTH) {
				LEDRing::rangeLED(0x00,0x20,br,(uint8_t)((bclr&0xFF0000)>>16),(uint8_t)((bclr&0xFF00)>>8),(uint8_t)(bclr&0xFF),MEMORY); 					// Background color
				LEDRing::rangeLED(0x01,0x02,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
				LEDRing::rangeLED(0x11,0x02,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),UPDATE); 					// Foreground color
			}
			break;
		case HORIZONTALBAR5 :
			if(length==FRONT) {
				LEDRing::rangeLED(0x00,0x10,br,(uint8_t)((bclr&0xFF0000)>>16),(uint8_t)((bclr&0xFF00)>>8),(uint8_t)(bclr&0xFF),MEMORY); 					// Background color
				LEDRing::rangeLED(0x03,0x03,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),UPDATE); 					// Foreground color
			} else if(length==BACK) {
				LEDRing::rangeLED(0x10,0x10,br,(uint8_t)((bclr&0xFF0000)>>16),(uint8_t)((bclr&0xFF00)>>8),(uint8_t)(bclr&0xFF),MEMORY); 					// Background color
				LEDRing::rangeLED(0x13,0x03,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),UPDATE); 					// Foreground color
			} else if(length==BOTH) {
				LEDRing::rangeLED(0x00,0x20,br,(uint8_t)((bclr&0xFF0000)>>16),(uint8_t)((bclr&0xFF00)>>8),(uint8_t)(bclr&0xFF),MEMORY); 					// Background color
				LEDRing::rangeLED(0x03,0x03,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
				LEDRing::rangeLED(0x13,0x03,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),UPDATE); 					// Foreground color
			}
			break;
		case HORIZONTALBAR4 :
			if(length==FRONT) {
				LEDRing::rangeLED(0x00,0x10,br,(uint8_t)((bclr&0xFF0000)>>16),(uint8_t)((bclr&0xFF00)>>8),(uint8_t)(bclr&0xFF),MEMORY); 					// Background color
				LEDRing::rangeLED(0x06,0x04,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),UPDATE); 					// Foreground color
			} else if(length==BACK) {
				LEDRing::rangeLED(0x10,0x10,br,(uint8_t)((bclr&0xFF0000)>>16),(uint8_t)((bclr&0xFF00)>>8),(uint8_t)(bclr&0xFF),MEMORY); 					// Background color
				LEDRing::rangeLED(0x16,0x04,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),UPDATE); 					// Foreground color
			} else if(length==BOTH) {
				LEDRing::rangeLED(0x00,0x20,br,(uint8_t)((bclr&0xFF0000)>>16),(uint8_t)((bclr&0xFF00)>>8),(uint8_t)(bclr&0xFF),MEMORY); 					// Background color
				LEDRing::rangeLED(0x06,0x04,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
				LEDRing::rangeLED(0x16,0x04,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),UPDATE); 					// Foreground color
			}
			break;
		case HORIZONTALBAR3 :
			if(length==FRONT) {
				LEDRing::rangeLED(0x00,0x10,br,(uint8_t)((bclr&0xFF0000)>>16),(uint8_t)((bclr&0xFF00)>>8),(uint8_t)(bclr&0xFF),MEMORY); 					// Background color
				LEDRing::rangeLED(0x0A,0x03,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),UPDATE); 					// Foreground color
			} else if(length==BACK) {
				LEDRing::rangeLED(0x10,0x10,br,(uint8_t)((bclr&0xFF0000)>>16),(uint8_t)((bclr&0xFF00)>>8),(uint8_t)(bclr&0xFF),MEMORY); 					// Background color
				LEDRing::rangeLED(0x1A,0x03,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),UPDATE); 					// Foreground color
			} else if(length==BOTH) {
				LEDRing::rangeLED(0x00,0x20,br,(uint8_t)((bclr&0xFF0000)>>16),(uint8_t)((bclr&0xFF00)>>8),(uint8_t)(bclr&0xFF),MEMORY); 					// Background color
				LEDRing::rangeLED(0x0A,0x03,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
				LEDRing::rangeLED(0x1A,0x03,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),UPDATE); 					// Foreground color
			}
			break;
		case HORIZONTALBAR2 :
			if(length==FRONT) {
				LEDRing::rangeLED(0x00,0x10,br,(uint8_t)((bclr&0xFF0000)>>16),(uint8_t)((bclr&0xFF00)>>8),(uint8_t)(bclr&0xFF),MEMORY); 					// Background color
				LEDRing::rangeLED(0x0D,0x02,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),UPDATE); 					// Foreground color
			} else if(length==BACK) {
				LEDRing::rangeLED(0x10,0x10,br,(uint8_t)((bclr&0xFF0000)>>16),(uint8_t)((bclr&0xFF00)>>8),(uint8_t)(bclr&0xFF),MEMORY); 					// Background color
				LEDRing::rangeLED(0x1D,0x02,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),UPDATE); 					// Foreground color
			} else if(length==BOTH) {
				LEDRing::rangeLED(0x00,0x20,br,(uint8_t)((bclr&0xFF0000)>>16),(uint8_t)((bclr&0xFF00)>>8),(uint8_t)(bclr&0xFF),MEMORY); 					// Background color
				LEDRing::rangeLED(0x0D,0x02,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
				LEDRing::rangeLED(0x1D,0x02,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),UPDATE); 					// Foreground color
			}
			break;
		case HORIZONTALBAR1 :
			if(length==FRONT) {
				LEDRing::rangeLED(0x00,0x10,br,(uint8_t)((bclr&0xFF0000)>>16),(uint8_t)((bclr&0xFF00)>>8),(uint8_t)(bclr&0xFF),MEMORY); 					// Background color
				LEDRing::rangeLED(0x0F,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),UPDATE); 					// Foreground color
			} else if(length==BACK) {
				LEDRing::rangeLED(0x10,0x10,br,(uint8_t)((bclr&0xFF0000)>>16),(uint8_t)((bclr&0xFF00)>>8),(uint8_t)(bclr&0xFF),MEMORY); 					// Background color
				LEDRing::rangeLED(0x1F,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),UPDATE); 					// Foreground color
			} else if(length==BOTH) {
				LEDRing::rangeLED(0x00,0x20,br,(uint8_t)((bclr&0xFF0000)>>16),(uint8_t)((bclr&0xFF00)>>8),(uint8_t)(bclr&0xFF),MEMORY); 					// Background color
				LEDRing::rangeLED(0x0F,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
				LEDRing::rangeLED(0x1F,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),UPDATE); 					// Foreground color
			}
			break;
		case BARGRAPH7 :
			if(length==FRONT || length==BOTH) {
				LEDRing::rangeLED(0x00,0x10,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
			} 
			if(length==BACK || length==BOTH) {
				LEDRing::rangeLED(0x10,0x10,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
			}
			if(mode==UPDATE) LEDRing::updateRing(RGBLED);
			break;
		case BARGRAPH6 :
			if(length==FRONT || length==BOTH) {
				LEDRing::rangeLED(0x00,0x10,br,(uint8_t)((bclr&0xFF0000)>>16),(uint8_t)((bclr&0xFF00)>>8),(uint8_t)(bclr&0xFF),MEMORY); 					// Background color
				LEDRing::rangeLED(0x01,0x0F,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
			} 
			if(length==BACK || length==BOTH) {
				LEDRing::rangeLED(0x10,0x10,br,(uint8_t)((bclr&0xFF0000)>>16),(uint8_t)((bclr&0xFF00)>>8),(uint8_t)(bclr&0xFF),MEMORY); 					// Background color
				LEDRing::rangeLED(0x11,0x0F,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
			}
			if(mode==UPDATE) LEDRing::updateRing(RGBLED);
			break;
		case BARGRAPH5 :
			if(length==FRONT || length==BOTH) {
				LEDRing::rangeLED(0x00,0x10,br,(uint8_t)((bclr&0xFF0000)>>16),(uint8_t)((bclr&0xFF00)>>8),(uint8_t)(bclr&0xFF),MEMORY); 					// Background color
				LEDRing::rangeLED(0x03,0x0D,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
			} 
			if(length==BACK || length==BOTH) {
				LEDRing::rangeLED(0x10,0x10,br,(uint8_t)((bclr&0xFF0000)>>16),(uint8_t)((bclr&0xFF00)>>8),(uint8_t)(bclr&0xFF),MEMORY); 					// Background color
				LEDRing::rangeLED(0x13,0x0D,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
			}
			if(mode==UPDATE) LEDRing::updateRing(RGBLED);
			break;
		case BARGRAPH4 :
			if(length==FRONT || length==BOTH) {
				LEDRing::rangeLED(0x00,0x10,br,(uint8_t)((bclr&0xFF0000)>>16),(uint8_t)((bclr&0xFF00)>>8),(uint8_t)(bclr&0xFF),MEMORY); 					// Background color
				LEDRing::rangeLED(0x06,0x0A,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
			} 
			if(length==BACK || length==BOTH) {
				LEDRing::rangeLED(0x10,0x10,br,(uint8_t)((bclr&0xFF0000)>>16),(uint8_t)((bclr&0xFF00)>>8),(uint8_t)(bclr&0xFF),MEMORY); 					// Background color
				LEDRing::rangeLED(0x16,0x0A,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
			}
			if(mode==UPDATE) LEDRing::updateRing(RGBLED);
			break;
		case BARGRAPH3 :
			if(length==FRONT || length==BOTH) {
				LEDRing::rangeLED(0x00,0x10,br,(uint8_t)((bclr&0xFF0000)>>16),(uint8_t)((bclr&0xFF00)>>8),(uint8_t)(bclr&0xFF),MEMORY); 					// Background color
				LEDRing::rangeLED(0x0A,0x06,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
			} 
			if(length==BACK || length==BOTH) {
				LEDRing::rangeLED(0x10,0x10,br,(uint8_t)((bclr&0xFF0000)>>16),(uint8_t)((bclr&0xFF00)>>8),(uint8_t)(bclr&0xFF),MEMORY); 					// Background color
				LEDRing::rangeLED(0x1A,0x06,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
			}
			if(mode==UPDATE) LEDRing::updateRing(RGBLED);
			break;
		case BARGRAPH2 :
			if(length==FRONT || length==BOTH) {
				LEDRing::rangeLED(0x00,0x10,br,(uint8_t)((bclr&0xFF0000)>>16),(uint8_t)((bclr&0xFF00)>>8),(uint8_t)(bclr&0xFF),MEMORY); 					// Background color
				LEDRing::rangeLED(0x0D,0x03,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
			} 
			if(length==BACK || length==BOTH) {
				LEDRing::rangeLED(0x10,0x10,br,(uint8_t)((bclr&0xFF0000)>>16),(uint8_t)((bclr&0xFF00)>>8),(uint8_t)(bclr&0xFF),MEMORY); 					// Background color
				LEDRing::rangeLED(0x1D,0x03,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
			}
			if(mode==UPDATE) LEDRing::updateRing(RGBLED);
			break;
		case BARGRAPH1 :
			if(length==FRONT || length==BOTH) {
				LEDRing::rangeLED(0x00,0x10,br,(uint8_t)((bclr&0xFF0000)>>16),(uint8_t)((bclr&0xFF00)>>8),(uint8_t)(bclr&0xFF),MEMORY); 					// Background color
				LEDRing::rangeLED(0x0F,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
			} 
			if(length==BACK || length==BOTH) {
				LEDRing::rangeLED(0x10,0x10,br,(uint8_t)((bclr&0xFF0000)>>16),(uint8_t)((bclr&0xFF00)>>8),(uint8_t)(bclr&0xFF),MEMORY); 					// Background color
				LEDRing::rangeLED(0x1F,0x01,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY); 					// Foreground color
			}
			if(mode==UPDATE) LEDRing::updateRing(RGBLED);
			break;
			

		case ARROWDOWN7 :
			if(length==FRONT || length==BOTH) LEDRing::showSymbol(ARROWDOWN,7,FRONT,br,fclr,bclr);
			if(length==BACK  || length==BOTH) LEDRing::showSymbol(ARROWDOWN,7,BACK,br,fclr,bclr);
			if(mode==UPDATE) LEDRing::updateRing(RGBLED);
			break;
		case ARROWDOWN6 :
			if(length==FRONT || length==BOTH) LEDRing::showSymbol(ARROWDOWN,6,FRONT,br,fclr,bclr);
			if(length==BACK  || length==BOTH) LEDRing::showSymbol(ARROWDOWN,6,BACK,br,fclr,bclr);
			if(mode==UPDATE) if(mode==UPDATE) LEDRing::updateRing(RGBLED);
			break;
		case ARROWDOWN5 :
			if(length==FRONT || length==BOTH) LEDRing::showSymbol(ARROWDOWN,5,FRONT,br,fclr,bclr);
			if(length==BACK  || length==BOTH) LEDRing::showSymbol(ARROWDOWN,5,BACK,br,fclr,bclr);
			if(mode==UPDATE) LEDRing::updateRing(RGBLED);
			break;
		case ARROWDOWN4 :
			if(length==FRONT || length==BOTH) LEDRing::showSymbol(ARROWDOWN,4,FRONT,br,fclr,bclr);
			if(length==BACK  || length==BOTH) LEDRing::showSymbol(ARROWDOWN,4,BACK,br,fclr,bclr);
			if(mode==UPDATE) LEDRing::updateRing(RGBLED);
			break;
		case ARROWDOWN3 :
			if(length==FRONT || length==BOTH) LEDRing::showSymbol(ARROWDOWN,3,FRONT,br,fclr,bclr);
			if(length==BACK  || length==BOTH) LEDRing::showSymbol(ARROWDOWN,3,BACK,br,fclr,bclr);
			if(mode==UPDATE) LEDRing::updateRing(RGBLED);
			break;
		case ARROWDOWN2 :
			if(length==FRONT || length==BOTH) LEDRing::showSymbol(ARROWDOWN,2,FRONT,br,fclr,bclr);
			if(length==BACK  || length==BOTH) LEDRing::showSymbol(ARROWDOWN,2,BACK,br,fclr,bclr);
			if(mode==UPDATE) LEDRing::updateRing(RGBLED);
			break;
		case ARROWDOWN1 :
			if(length==FRONT || length==BOTH) LEDRing::showSymbol(ARROWDOWN,1,FRONT,br,fclr,bclr);
			if(length==BACK  || length==BOTH) LEDRing::showSymbol(ARROWDOWN,1,BACK,br,fclr,bclr);
			if(mode==UPDATE) LEDRing::updateRing(RGBLED);
			break;

		case ARROWUP7 :
			if(length==FRONT || length==BOTH) LEDRing::showSymbol(ARROWUP,7,FRONT,br,fclr,bclr);
			if(length==BACK  || length==BOTH) LEDRing::showSymbol(ARROWUP,7,BACK,br,fclr,bclr);
			if(mode==UPDATE) LEDRing::updateRing(RGBLED);
			break;
		case ARROWUP6 :
			if(length==FRONT || length==BOTH) LEDRing::showSymbol(ARROWUP,6,FRONT,br,fclr,bclr);
			if(length==BACK  || length==BOTH) LEDRing::showSymbol(ARROWUP,6,BACK,br,fclr,bclr);
			if(mode==UPDATE) LEDRing::updateRing(RGBLED);
			break;
		case ARROWUP5 :
			if(length==FRONT || length==BOTH) LEDRing::showSymbol(ARROWUP,5,FRONT,br,fclr,bclr);
			if(length==BACK  || length==BOTH) LEDRing::showSymbol(ARROWUP,5,BACK,br,fclr,bclr);
			if(mode==UPDATE) LEDRing::updateRing(RGBLED);
			break;
		case ARROWUP4 :
			if(length==FRONT || length==BOTH) LEDRing::showSymbol(ARROWUP,4,FRONT,br,fclr,bclr);
			if(length==BACK  || length==BOTH) LEDRing::showSymbol(ARROWUP,4,BACK,br,fclr,bclr);
			if(mode==UPDATE) LEDRing::updateRing(RGBLED);
			break;
		case ARROWUP3 :
			if(length==FRONT || length==BOTH) LEDRing::showSymbol(ARROWUP,3,FRONT,br,fclr,bclr);
			if(length==BACK  || length==BOTH) LEDRing::showSymbol(ARROWUP,3,BACK,br,fclr,bclr);
			if(mode==UPDATE) LEDRing::updateRing(RGBLED);
			break;
		case ARROWUP2 :
			if(length==FRONT || length==BOTH) LEDRing::showSymbol(ARROWUP,2,FRONT,br,fclr,bclr);
			if(length==BACK  || length==BOTH) LEDRing::showSymbol(ARROWUP,2,BACK,br,fclr,bclr);
			if(mode==UPDATE) LEDRing::updateRing(RGBLED);
			break;
		case ARROWUP1 :
			if(length==FRONT || length==BOTH) LEDRing::showSymbol(ARROWUP,1,FRONT,br,fclr,bclr);
			if(length==BACK  || length==BOTH) LEDRing::showSymbol(ARROWUP,1,BACK,br,fclr,bclr);
			if(mode==UPDATE) LEDRing::updateRing(RGBLED);
			break;
			
		 case ARROWWEST7 :
			if(length==FRONT || length==BOTH) LEDRing::showSymbol(ARROWLEFT,7,FRONT,br,fclr,bclr);
			if(length==BACK  || length==BOTH) LEDRing::showSymbol(ARROWRIGHT,7,BACK,br,fclr,bclr);
			if(mode==UPDATE) LEDRing::updateRing(RGBLED);
			break;
		 case ARROWWEST6 :
			if(length==FRONT || length==BOTH) LEDRing::showSymbol(ARROWLEFT,6,FRONT,br,fclr,bclr);
			if(length==BACK  || length==BOTH) LEDRing::showSymbol(ARROWRIGHT,6,BACK,br,fclr,bclr);
			if(mode==UPDATE) LEDRing::updateRing(RGBLED);
			break;
		 case ARROWWEST5:
			if(length==FRONT || length==BOTH) LEDRing::showSymbol(ARROWLEFT,5,FRONT,br,fclr,bclr);
			if(length==BACK  || length==BOTH) LEDRing::showSymbol(ARROWRIGHT,5,BACK,br,fclr,bclr);
			if(mode==UPDATE) LEDRing::updateRing(RGBLED);
			break;
		case ARROWWEST4 :
			if(length==FRONT || length==BOTH) LEDRing::showSymbol(ARROWLEFT,4,FRONT,br,fclr,bclr);
			if(length==BACK  || length==BOTH) LEDRing::showSymbol(ARROWRIGHT,4,BACK,br,fclr,bclr);
			if(mode==UPDATE) LEDRing::updateRing(RGBLED);
			break;
		case ARROWWEST3 :
			if(length==FRONT || length==BOTH) LEDRing::showSymbol(ARROWLEFT,3,FRONT,br,fclr,bclr);
			if(length==BACK  || length==BOTH) LEDRing::showSymbol(ARROWRIGHT,3,BACK,br,fclr,bclr);
			if(mode==UPDATE) LEDRing::updateRing(RGBLED);
			break;
		case ARROWWEST2 :
			if(length==FRONT || length==BOTH) LEDRing::showSymbol(ARROWLEFT,2,FRONT,br,fclr,bclr);
			if(length==BACK  || length==BOTH) LEDRing::showSymbol(ARROWRIGHT,2,BACK,br,fclr,bclr);
			if(mode==UPDATE) LEDRing::updateRing(RGBLED);
			break;
		case ARROWWEST1 :
			if(length==FRONT || length==BOTH) LEDRing::showSymbol(ARROWLEFT,1,FRONT,br,fclr,bclr);
			if(length==BACK  || length==BOTH) LEDRing::showSymbol(ARROWRIGHT,1,BACK,br,fclr,bclr);
			if(mode==UPDATE) LEDRing::updateRing(RGBLED);
			break;

		case ARROWEAST7 :
			if(length==FRONT || length==BOTH) LEDRing::showSymbol(ARROWRIGHT,7,FRONT,br,fclr,bclr);
			if(length==BACK  || length==BOTH) LEDRing::showSymbol(ARROWLEFT,7,BACK,br,fclr,bclr);
			if(mode==UPDATE) LEDRing::updateRing(RGBLED);
			break;
		 case ARROWEAST6 :
			if(length==FRONT || length==BOTH) LEDRing::showSymbol(ARROWRIGHT,6,FRONT,br,fclr,bclr);
			if(length==BACK  || length==BOTH) LEDRing::showSymbol(ARROWLEFT,6,BACK,br,fclr,bclr);
			if(mode==UPDATE) LEDRing::updateRing(RGBLED);
			break;
		 case ARROWEAST5:
			if(length==FRONT || length==BOTH) LEDRing::showSymbol(ARROWRIGHT,5,FRONT,br,fclr,bclr);
			if(length==BACK  || length==BOTH) LEDRing::showSymbol(ARROWLEFT,5,BACK,br,fclr,bclr);
			if(mode==UPDATE) LEDRing::updateRing(RGBLED);
			break;
		case ARROWEAST4 :
			if(length==FRONT || length==BOTH) LEDRing::showSymbol(ARROWRIGHT,4,FRONT,br,fclr,bclr);
			if(length==BACK  || length==BOTH) LEDRing::showSymbol(ARROWLEFT,4,BACK,br,fclr,bclr);
			if(mode==UPDATE) LEDRing::updateRing(RGBLED);
			break;
		case ARROWEAST3 :
			if(length==FRONT || length==BOTH) LEDRing::showSymbol(ARROWRIGHT,3,FRONT,br,fclr,bclr);
			if(length==BACK  || length==BOTH) LEDRing::showSymbol(ARROWLEFT,3,BACK,br,fclr,bclr);
			if(mode==UPDATE) LEDRing::updateRing(RGBLED);
			break;
		case ARROWEAST2 :
			if(length==FRONT || length==BOTH) LEDRing::showSymbol(ARROWRIGHT,2,FRONT,br,fclr,bclr);
			if(length==BACK  || length==BOTH) LEDRing::showSymbol(ARROWLEFT,2,BACK,br,fclr,bclr);
			if(mode==UPDATE) LEDRing::updateRing(RGBLED);
			break;
		case ARROWEAST1 :
			if(length==FRONT || length==BOTH) LEDRing::showSymbol(ARROWRIGHT,1,FRONT,br,fclr,bclr);
			if(length==BACK  || length==BOTH) LEDRing::showSymbol(ARROWLEFT,1,BACK,br,fclr,bclr);
			if(mode==UPDATE) LEDRing::updateRing(RGBLED);
			break;
		case FULLON :
			if(length==FRONT || length==BOTH) LEDRing::rangeLED(0x00,0x10,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY);
			if(length==BACK  || length==BOTH) LEDRing::rangeLED(0x10,0x10,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),MEMORY);
			if(mode==UPDATE) LEDRing::updateRing(RGBLED);
			break;
		default :
			LEDRing::rangeLED(0x00,0x20,br,(uint8_t)((bclr&0xFF0000)>>16),(uint8_t)((bclr&0xFF00)>>8),(uint8_t)(bclr&0xFF),MEMORY); 					// Background color
			LEDRing::rangeLED(start,length,br,(uint8_t)((fclr&0xFF0000)>>16),(uint8_t)((fclr&0xFF00)>>8),(uint8_t)(fclr&0xFF),UPDATE); 					// Foreground color
			break;
	}
}