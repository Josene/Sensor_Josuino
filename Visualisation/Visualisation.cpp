/************************************************************************************
* Visualisation.cpp - Library for Visualisation 				                            	*
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

#include "Visualisation.h"
#include "Josuino.h"

static uint16_t visVisual = 0;
//static uint16_t visCounter = 0;
//static uint16_t visLED = 0;
static uint16_t visBackup = 0;
static uint16_t visActual = 0;
static uint16_t visBackup2= 0;
static uint16_t visActual2= 0;

extern uint16_t DemoCursorActive;


// Constructor
Visualisation::Visualisation()
{

}

void Visualisation::showCO2(uint16_t hw, uint8_t br, uint16_t val) {
	if(visActual<352) visActual=352;
	if(visActual<val) visActual++;
	if(visActual>val) visActual--;
	if((visActual==visBackup) && (DemoCursorActive==OFF)) return;
	DemoCursorActive=OFF;
	visBackup = visActual;
	if (visActual <=352) 					 LEDRing::easyDemoTM(DUALCENTER,16,br,BLUE,BLUE);
	if (visActual > 352 && visActual <= 448) LEDRing::easyDemoTM(DUALCENTER,(visActual-352)/6,br,GREEN,BLUE);
	if (visActual > 448 && visActual <= 544) LEDRing::easyDemoTM(DUALCENTER,(visActual-448)/6,br,YELLOW,GREEN);
	if (visActual > 544 && visActual <= 640) LEDRing::easyDemoTM(DUALCENTER,(visActual-544)/6,br,RED,YELLOW);
	if (visActual > 640 && visActual <= 736) LEDRing::easyDemoTM(DUALCENTER,(visActual-640)/6,br,MAGENTA,RED);
	if (visActual > 736) 					 LEDRing::easyDemoTM(DUALCENTER,16,br,MAGENTA,MAGENTA);
}

void Visualisation::showPM10(uint16_t hw, uint8_t br, uint16_t val) {
	if(visActual<val) visActual++;
	if(visActual>val) visActual--;
	if((visActual==visBackup) && (DemoCursorActive==OFF)) return;
	DemoCursorActive=OFF;
	visBackup = visActual;
	if (visActual==0) 						 LEDRing::easyDemoTM(DUALCENTER,16,br,BLUE,BLUE);
	if (visActual > 0 && visActual <= 16) 	 LEDRing::easyDemoTM(DUALCENTER,visActual,br,GREEN,BLUE);
	if (visActual > 16 && visActual <= 48) 	 LEDRing::easyDemoTM(DUALCENTER,(visActual-16)/2,br,YELLOW,GREEN);
	if (visActual > 48 && visActual <= 128)  LEDRing::easyDemoTM(DUALCENTER,(visActual-48)/5,br,RED,YELLOW);
	if (visActual > 128 && visActual <= 176) LEDRing::easyDemoTM(DUALCENTER,(visActual-128)/3,br,MAGENTA,RED);
	if (visActual > 176)                     LEDRing::easyDemoTM(DUALCENTER,16,br,MAGENTA,MAGENTA);
}

void Visualisation::showPM2_5(uint16_t hw, uint8_t br, uint16_t val) {
	if(visActual<val) visActual++;
	if(visActual>val) visActual--;
	if((visActual==visBackup) && (DemoCursorActive==OFF)) return;
	DemoCursorActive=OFF;
	visBackup = visActual;
	if (visActual==0) 						 LEDRing::easyDemoTM(DUALCENTER,16,br,BLUE,BLUE);
	if (visActual > 0 && visActual <= 16) 	 LEDRing::easyDemoTM(DUALCENTER,visActual,br,GREEN,BLUE);
	if (visActual > 16 && visActual <= 32) 	 LEDRing::easyDemoTM(DUALCENTER,(visActual-16),br,YELLOW,GREEN);
	if (visActual > 32 && visActual <= 80) 	 LEDRing::easyDemoTM(DUALCENTER,(visActual-32)/3,br,RED,YELLOW);
	if (visActual > 80 && visActual <= 128)	 LEDRing::easyDemoTM(DUALCENTER,(visActual-80)/3,br,MAGENTA,RED);
	if (visActual > 128)                     LEDRing::easyDemoTM(DUALCENTER,16,br,MAGENTA,MAGENTA);
}

void Visualisation::showTemperature(uint16_t hw, uint8_t br, uint16_t val) {
	if(visActual<val) visActual++;
	if(visActual>val) visActual--;
	if((visActual==visBackup) && (DemoCursorActive==OFF)) return;
	DemoCursorActive=OFF;
	visBackup = visActual;
	if (visActual <= 16) 					 LEDRing::easyDemoTM(DUALCENTER,16,br,DARKBLUE,DARKBLUE);
	if (visActual > 16 && visActual <= 32) 	 LEDRing::easyDemoTM(DUALCENTER,(visActual-16),br,ORANGE,DARKBLUE);
	if (visActual > 32)					 	 LEDRing::easyDemoTM(DUALCENTER,16,br,ORANGE,ORANGE);
}

void Visualisation::showHumidity(uint16_t hw, uint8_t br, uint16_t val) {
	if(visActual<val) visActual++;
	if(visActual>val) visActual--;
	if((visActual==visBackup) && (DemoCursorActive==OFF)) return;
	DemoCursorActive=OFF;
	visBackup = visActual;
	if (visActual <= 16) 					 LEDRing::easyDemoTM(DUALCENTER,16,br,DARKBLUE,DARKBLUE);
	if (visActual > 16 && visActual <= 80) 	 LEDRing::easyDemoTM(DUALCENTER,(visActual-16)/4,br,CYAN,DARKBLUE);
	if (visActual > 80)					 	 LEDRing::easyDemoTM(DUALCENTER,16,br,CYAN,CYAN);
}

void Visualisation::showPressure(uint16_t hw, uint8_t br, uint16_t val) {
	if(visActual<976) visActual=976;
	if(visActual<val) visActual++;
	if(visActual>val) visActual--;
	if((visActual==visBackup) && (DemoCursorActive==OFF)) return;
	DemoCursorActive=OFF;
	visBackup = visActual;
	if (visActual <=976) 					 LEDRing::easyDemoTM(DUALCENTER,16,br,DARKBLUE,DARKBLUE);
	if (visActual > 976 && visActual <= 984) LEDRing::easyDemoTM(DUALCENTER,(visActual- 976)*2,br,BLUE,DARKBLUE);
	if (visActual > 984 && visActual <= 992) LEDRing::easyDemoTM(DUALCENTER,(visActual- 984)*2,br,CYAN,BLUE);
	if (visActual > 992 && visActual <=1000) LEDRing::easyDemoTM(DUALCENTER,(visActual- 992)*2,br,GREEN,CYAN);
	if (visActual >1000 && visActual <=1008) LEDRing::easyDemoTM(DUALCENTER,(visActual-1000)*2,br,GREENYELLOW,GREEN);
	if (visActual >1008 && visActual <=1016) LEDRing::easyDemoTM(DUALCENTER,(visActual-1008)*2,br,YELLOW,GREENYELLOW);
	if (visActual >1016 && visActual <=1024) LEDRing::easyDemoTM(DUALCENTER,(visActual-1016)*2,br,ORANGE,YELLOW);
	if (visActual >1024 && visActual <=1032) LEDRing::easyDemoTM(DUALCENTER,(visActual-1024)*2,br,RED,ORANGE);
	if (visActual >1032 && visActual <=1040) LEDRing::easyDemoTM(DUALCENTER,(visActual-1032)*2,br,MAGENTA,RED);
	if (visActual >1040)					 LEDRing::easyDemoTM(DUALCENTER,16*2,br,MAGENTA,MAGENTA);
}

void Visualisation::reset() {
	visActual=0;
	visBackup=0;
}

void Visualisation::showDemo(uint16_t hw, uint8_t br, uint8_t pk) {
	//if(visActual2<pk) visActual2++;
	//if(visActual2>pk) visActual2--;
	visActual2=pk;
	if(visActual2>82) visActual2=82; 
	
	
	if(visActual>=0x09FF) visActual=0; else visActual+=1;
	if((visActual==visBackup) && (DemoCursorActive==OFF)) return;
	DemoCursorActive=OFF;
	visBackup = visActual;

	uint16_t repeat = visActual & 0x003F;
	uint16_t number = ((visActual & 0x3FC0) >> 6);	
	uint8_t rd = 0;
	uint8_t gn = 0xFF;
	
	if(visActual2>=40) {
		rd = ((visActual2-40)*6)+3;
		gn = 0xFF-rd;
	}

	//if(bar>7)bar=7;

	LEDRing::rangeLED(0x10,0x04,br,rd,0,0,MEMORY);
	LEDRing::rangeLED(0x15,0x02,br,rd,0,0,MEMORY);
	LEDRing::rangeLED(0x19,0x02,br,rd,0,0,MEMORY);
	LEDRing::rangeLED(0x1C,0x04,br,rd,0,0,MEMORY);
	
	LEDRing::rangeLED(0x14,0x01,br-2,0,gn,0,MEMORY); 
	LEDRing::rangeLED(0x17,0x02,br-2,0,gn,0,MEMORY); 
	LEDRing::rangeLED(0x1B,0x01,br-2,0,gn,0,UPDATE); 
	//LEDRing::rangeLED(0x00,0x10,br,rd,0,0,UPDATE); 

	//DRing::easyDemoTM(FULLON,BACK,br,,BLACK);
		
	
	
	
	
	if(number >= 0 && number <= 9) {
		if(repeat==0x07) LEDRing::easyDemoTM(ARROWWEST1,FRONT,br,ORANGE,BLACK);
		if(repeat==0x0E) LEDRing::easyDemoTM(ARROWWEST2,FRONT,br,ORANGE,BLACK);
		if(repeat==0x15) LEDRing::easyDemoTM(ARROWWEST3,FRONT,br,ORANGE,BLACK);
		if(repeat==0x1C) LEDRing::easyDemoTM(ARROWWEST4,FRONT,br,ORANGE,BLACK);
		if(repeat==0x23) LEDRing::easyDemoTM(ARROWWEST5,FRONT,br,ORANGE,BLACK);
		if(repeat==0x2A) LEDRing::easyDemoTM(ARROWWEST6,FRONT,br,ORANGE,BLACK);
		if(repeat==0x31) LEDRing::easyDemoTM(ARROWWEST7,FRONT,br,ORANGE,BLACK);
		if(repeat==0x38) LEDRing::easyDemoTM(FULLON,FRONT,br,BLACK,BLACK);
	} else if(number >= 10 && number <= 19) {
		if(repeat==0x07) LEDRing::easyDemoTM(ARROWEAST1,FRONT,br,LIGHTBLUE,BLACK);
		if(repeat==0x0E) LEDRing::easyDemoTM(ARROWEAST2,FRONT,br,LIGHTBLUE,BLACK);
		if(repeat==0x15) LEDRing::easyDemoTM(ARROWEAST3,FRONT,br,LIGHTBLUE,BLACK);
		if(repeat==0x1C) LEDRing::easyDemoTM(ARROWEAST4,FRONT,br,LIGHTBLUE,BLACK);
		if(repeat==0x23) LEDRing::easyDemoTM(ARROWEAST5,FRONT,br,LIGHTBLUE,BLACK);
		if(repeat==0x2A) LEDRing::easyDemoTM(ARROWEAST6,FRONT,br,LIGHTBLUE,BLACK);
		if(repeat==0x31) LEDRing::easyDemoTM(ARROWEAST7,FRONT,br,LIGHTBLUE,BLACK);
		if(repeat==0x38) LEDRing::easyDemoTM(FULLON,FRONT,br,BLACK,BLACK);
	} else if(number >= 20 && number <= 29) {
		if(repeat==0x07) LEDRing::easyDemoTM(ARROWUP1,FRONT,br,WHITE,BLACK);
		if(repeat==0x0E) LEDRing::easyDemoTM(ARROWUP2,FRONT,br,WHITE,BLACK);
		if(repeat==0x15) LEDRing::easyDemoTM(ARROWUP3,FRONT,br,WHITE,BLACK);
		if(repeat==0x1C) LEDRing::easyDemoTM(ARROWUP4,FRONT,br,WHITE,BLACK);
		if(repeat==0x23) LEDRing::easyDemoTM(ARROWUP5,FRONT,br,WHITE,BLACK);
		if(repeat==0x2A) LEDRing::easyDemoTM(ARROWUP6,FRONT,br,WHITE,BLACK);
		if(repeat==0x31) LEDRing::easyDemoTM(ARROWUP7,FRONT,br,WHITE,BLACK);
		if(repeat==0x38) LEDRing::easyDemoTM(FULLON,FRONT,br,BLACK,BLACK);
	} else if(number >= 30 && number <= 39) {
		if(repeat==0x07) LEDRing::easyDemoTM(ARROWDOWN1,FRONT,br,LIGHTGREEN,BLACK);
		if(repeat==0x0E) LEDRing::easyDemoTM(ARROWDOWN2,FRONT,br,LIGHTGREEN,BLACK);
		if(repeat==0x15) LEDRing::easyDemoTM(ARROWDOWN3,FRONT,br,LIGHTGREEN,BLACK);
		if(repeat==0x1C) LEDRing::easyDemoTM(ARROWDOWN4,FRONT,br,LIGHTGREEN,BLACK);
		if(repeat==0x23) LEDRing::easyDemoTM(ARROWDOWN5,FRONT,br,LIGHTGREEN,BLACK);
		if(repeat==0x2A) LEDRing::easyDemoTM(ARROWDOWN6,FRONT,br,LIGHTGREEN,BLACK);
		if(repeat==0x31) LEDRing::easyDemoTM(ARROWDOWN7,FRONT,br,LIGHTGREEN,BLACK);
		if(repeat==0x38) LEDRing::easyDemoTM(FULLON,FRONT,br,BLACK,BLACK);
	}	
	
	
	
	
//	if(visActual>0&&visActual<=32) LEDRing::easyDemoTM(visActual-0,4,br,RED,BLUE);
//	if(visActual>32&&visActual<=64)	LEDRing::easyDemoTM(visActual-32,4,br,GREEN,RED);
//	if(visActual>64&&visActual<=96)	LEDRing::easyDemoTM(visActual-64,4,br,BLUE,GREEN);
}
