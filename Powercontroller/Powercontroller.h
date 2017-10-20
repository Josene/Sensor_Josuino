/************************************************************************************
* Powercontroller.h - Header for Powercontroller           	                    	*
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
#ifndef Powercontroller_h
#define Powercontroller_h
//#define Powercontroller_DEBUGMEAS
#include <Arduino.h> 
#include <I2C_Device.h>

#define OUTPUTS			0x20
#define BATGAUGE		0x2E
#define CURRENTMODE		0x2F
#define BATVOLTAGE		0x30
#define PSUVOLTAGE		0x32
#define BATCURRENT		0x34
#define PSUCURRENT		0x36
#define PSUTEMPERATURE	0x38
#define COULOMBCOUNTER	0x3A
#define SENSORBOARD		0
#define LEDMATRIX1		1		
#define LEDMATRIX2		2
#define AUDIOSENSOR		3
#define ROUTER			4
#define CAMERASERVER	5
#define OPTIONAL1		6
#define OPTIONAL2		7

struct PSUDataSet {
	uint8_t Testbutton = 0;				// Incremental testbuttonreg.	(reg 0x2E)
	uint8_t OutputStatus = 1;			// Outputs on powerctrl      	(reg 0x20)
	uint8_t FunctionMode = 0;			// Get current mode          	(reg 0x2F)
										// Bit7==1 main PSU on, bit 7==0 main PSU off
										// Bit6==1 on battery, bit6==0 not on battery
										// Bit1+0==11 battery not connected
										// Bit1+0==10 battery charging
										// Bit1+0==01 battery fully charged
										// Bit1+0==00 battery error
	uint16_t BatteryVoltage = 0;		// Battery voltage in mV     	(reg 0x30+0x31)
	uint16_t PSUVoltage = 0;			// Powersupply voltage mV    	(reg 0x32+0x33)
	int16_t BatteryCurrent = 0;			// Battery current in mA     	(reg 0x34+0x35)
	int16_t PSUCurrent = 0;				// Powersupply current in mA 	(reg 0x36+0x37)
	uint16_t PSUTemperature = 0;		// Powersupply temperature in K (reg 0x38+0x39)
	uint16_t BatteryCoulombs = 0;		// Coulomb counter				(reg 0x3A+0x3B)
};

class Powercontroller : public i2c_device
{
    public:
        Powercontroller();
        void begin(uint8_t Address, boolean TWI_Begin);
		void setOutput(uint8_t Register, uint8_t Data);
		void getData(struct PSUDataSet *psu, boolean dbg=false);
		void writeISP(uint32_t ISPdest, uint8_t ISPlen, uint8_t *d);
		uint8_t getMode();
		void setMode(uint8_t mode);
		void getTest();
};

#endif
