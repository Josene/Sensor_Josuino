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

//oat Accelero[4];
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

void ADXL345::getSensorData(struct AccelerometerDataSet *amds, uint8_t mode) {
	uint8_t FifoInfo[2];
	uint8_t RawSensorData[8];
	int32_t AddedSensorData[3]={ };
	//oat Angle = 0;
    
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
		i2c_device::read_many(0x32, 6, RawSensorData);
		int16_t TempX = (((int16_t)RawSensorData[1]<<8)|(int16_t)RawSensorData[0]);
		int16_t TempY = (((int16_t)RawSensorData[3]<<8)|(int16_t)RawSensorData[2]);
		if(mode==VERTICAL) TempY = (((int16_t)RawSensorData[5]<<8)|(int16_t)RawSensorData[4]);
		AddedSensorData[X]+=TempX;
		AddedSensorData[Y]+=TempY;
	}
	AddedSensorData[X]/=32;
	AddedSensorData[Y]/=32;
	MedianSensorData[X]=(Factor*AddedSensorData[X])+((1-Factor)*MedianSensorData[X]);
	MedianSensorData[Y]=(Factor*AddedSensorData[Y])+((1-Factor)*MedianSensorData[Y]);

	if((MedianSensorData[X]>20 || MedianSensorData[X]<-20) || (MedianSensorData[Y]>20 || MedianSensorData[Y]<-20)){
		amds->XAxis=(float)MedianSensorData[X]/(float)256;
		amds->YAxis=((float)MedianSensorData[Y]/(float)256)*-1;
		amds->Degr=((atan2(amds->YAxis,amds->XAxis))*57.295779);
		if(amds->Degr<0) amds->Degr=(180+amds->Degr)+180; //se amds->Degr=Angle;
	} else {
		amds->XAxis=0;
		amds->YAxis=0;
		amds->Degr=NAN;
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
		Serial.print(F(" aX:"));Serial.print(*adms->X);
		Serial.print(F(" aY:"));Serial.print(*adms->Y);
		Serial.print(F(" Degr:"));Serial.print(*adms->Degr);
		Serial.print(F("\n"));
	#endif
		
	//return Accelero;
}