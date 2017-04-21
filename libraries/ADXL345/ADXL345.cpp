/************************************************************************************
* ADXL345.cpp - Library for ADXL345 accelerometer                                  	*
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

#include "ADXL345.h"
#include "math.h"

float Accelero[4];
float MedianSensorData[4];
const float Factor=0.10;

// Constructor
ADXL345::ADXL345()
{

}

void ADXL345::begin(uint8_t Address, boolean TWI_Begin) {
    i2c_device::begin(Address, TWI_Begin);
	delay(20);
	i2c_device::write(0x2D, 0x08);		// Write 
	delay(20);
	i2c_device::write(0x31, 0x0B);		// Write register format
	delay(20);
	i2c_device::write(0x2C, 0x09);		// Write datarate 100Hz
	delay(20);
	i2c_device::write(0x38, 0x9F);		// Write Fifo mode is streaming
	delay(20);
	i2c_device::write(0x1E, 0xFD);		// X Offset
	delay(20);
	i2c_device::write(0x1F, 0x00);		// Y Offset
	delay(20);
	i2c_device::write(0x20, 0x03);		// Z Offset
	delay(20);
}

float * ADXL345::getSensorData() {
	uint8_t FifoInfo[2];
	uint8_t RawSensorData[6];
	int32_t AddedSensorData[3]={ };
    
#ifdef ADXL345_DEBUGMEAS
	i2c_device::read_many(0x38, 2, FifoInfo);
	if (FifoInfo[0]<16) Serial.print("0");
	Serial.print(FifoInfo[0],HEX);
	Serial.print(" ");
	if (FifoInfo[1]<16) Serial.print("0");
	Serial.print(FifoInfo[1],HEX);
	Serial.print("  ");
#endif
	
	for(uint8_t y=0; y<32;y++) {
		i2c_device::read_many(0x32, 4, RawSensorData);
		int16_t TempX = (((int16_t)RawSensorData[1]<<8)|(int16_t)RawSensorData[0]);
		int16_t TempY = (((int16_t)RawSensorData[3]<<8)|(int16_t)RawSensorData[2]);
		AddedSensorData[X]+=TempX;
		AddedSensorData[Y]+=TempY;
	}
	AddedSensorData[X]/=32;
	AddedSensorData[Y]/=32;
	MedianSensorData[X]=(Factor*AddedSensorData[X])+((1-Factor)*MedianSensorData[X]);
	MedianSensorData[Y]=(Factor*AddedSensorData[Y])+((1-Factor)*MedianSensorData[Y]);

	if((MedianSensorData[X]>20 || MedianSensorData[X]<-20) || (MedianSensorData[Y]>20 || MedianSensorData[Y]<-20)){
		Accelero[X]=(float)MedianSensorData[X]/(float)256;
		Accelero[Y]=((float)MedianSensorData[Y]/(float)256)*-1;
		Accelero[D]=((atan2(Accelero[Y],Accelero[X]))*57.295779);
		if(Accelero[D]<0) Accelero[D]=(180+Accelero[D])+180;
	} else {
		Accelero[X]=0;
		Accelero[Y]=0;
		Accelero[D]=NAN;
	}
		
	#ifdef ADXL345_DEBUGMEAS
		i2c_device::read_many(0x38, 2, FifoInfo);
		if (FifoInfo[0]<16) Serial.print("0");
		Serial.print(FifoInfo[0],HEX);
		Serial.print(" ");
		if (FifoInfo[1]<16) Serial.print("0");
		Serial.print(FifoInfo[1],HEX);
		Serial.print(F("  "));
		Serial.print(F(" cX:"));Serial.print(AddedSensorData[0]);
		Serial.print(F(" cY:"));Serial.print(AddedSensorData[1]);
		Serial.print(F(" mX:"));Serial.print(MedianSensorData[0]);
		Serial.print(F(" mY:"));Serial.print(MedianSensorData[1]);
		Serial.print(F(" aX:"));Serial.print(Accelero[X]);
		Serial.print(F(" aY:"));Serial.print(Accelero[Y]);
		Serial.print(F(" Degr:"));Serial.print(Accelero[D]);
		Serial.print(F("\n"));
	#endif
		
	return Accelero;
}