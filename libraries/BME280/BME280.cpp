/************************************************************************************
* BME280.cpp - Library for BME280 temperature sensor                              	*
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

#include "BME280.h"

const uint8_t Osrs_t = 5;
const uint8_t Osrs_p = 5;
const uint8_t Osrs_h = 5;
const uint8_t Mode_reg = 3;
const uint8_t T_sb = 0;
const uint8_t IIR = 4;

uint16_t DigT1;
int16_t DigT2;
int16_t DigT3;
uint16_t DigP1;
int16_t DigP2;
int16_t DigP3;
int16_t DigP4;
int16_t DigP5;
int16_t DigP6;
int16_t DigP7;
int16_t DigP8;
int16_t DigP9;
uint8_t DigH1;
int16_t DigH2;
uint8_t DigH3;
int16_t DigH4;
int16_t DigH5;
uint8_t DigH6;
float BME280value[2];

// Constructor
BME280::BME280()
{

}

void BME280::begin(uint8_t Address, boolean TWI_Begin) {
    i2c_device::begin(Address, TWI_Begin);
	delay(20);
	i2c_device::write(0xF5, 0x10);		// Write IIR + T_sb
	delay(20);
	i2c_device::write(0xF2, 0x05);		// Write Osrs_h
	delay(20);
	i2c_device::write(0xF4, 0xB7);		// Write Osrs_t, Osrs_p, Mode
	delay(20);

	byte RawCalibrationData[26];
    i2c_device::read_many(0x88, 26, RawCalibrationData);

#ifdef BME280_DEBUGDIAG
	Serial.println(F("BME280 calibration data:"));
	for(int x=0; x<26; x++) {
		if (RawCalibrationData[x]<16) Serial.print(F("0"));
		Serial.print(RawCalibrationData[x],HEX);
		Serial.print(F(" "));
	}
#endif
	
	DigT1 = (((uint16_t)(RawCalibrationData[1])<<8) | (uint16_t)(RawCalibrationData[0]));
	DigT2 = (((int16_t)(RawCalibrationData[3])<<8) | (int16_t)(RawCalibrationData[2]));
	DigT3 = (((int16_t)(RawCalibrationData[5])<<8) | (int16_t)(RawCalibrationData[4]));
	DigP1 = (((uint16_t)(RawCalibrationData[7])<<8) | (uint16_t)(RawCalibrationData[6]));
	DigP2 = (((int16_t)(RawCalibrationData[9])<<8) | (int16_t)(RawCalibrationData[8]));
	DigP3 = (((int16_t)(RawCalibrationData[11])<<8) | (int16_t)(RawCalibrationData[10]));
	DigP4 = (((int16_t)(RawCalibrationData[13])<<8) | (int16_t)(RawCalibrationData[12]));
	DigP5 = (((int16_t)(RawCalibrationData[15])<<8) | (int16_t)(RawCalibrationData[14]));
	DigP6 = (((int16_t)(RawCalibrationData[17])<<8) | (int16_t)(RawCalibrationData[16]));
	DigP7 = (((int16_t)(RawCalibrationData[19])<<8) | (int16_t)(RawCalibrationData[18]));
	DigP8 = (((int16_t)(RawCalibrationData[21])<<8) | (int16_t)(RawCalibrationData[20]));
	DigP9 = (((int16_t)(RawCalibrationData[23])<<8) | (int16_t)(RawCalibrationData[22]));
	DigH1 = RawCalibrationData[25];
	
    i2c_device::read_many(0xE1, 7, RawCalibrationData);

	DigH2 = (((int16_t)(RawCalibrationData[1])<<8) | (int16_t)(RawCalibrationData[0]));
	DigH3 = RawCalibrationData[2];
	DigH4 = (((int16_t)(RawCalibrationData[3])<<4) | ((int16_t)(RawCalibrationData[4]) & 0x0F)); 
	DigH5 = ((((int16_t)(RawCalibrationData[5])<<4) | (((int16_t)(RawCalibrationData[4])&0xF0) >> 4)));
	DigH6 = RawCalibrationData[6];

	
#ifdef BME280_DEBUGDIAG
	for(int x=0; x<7; x++) {
		if (RawCalibrationData[x]<16) Serial.print(F("0"));
		Serial.print(RawCalibrationData[x],HEX);
		Serial.print(F(" "));
	}
	Serial.print(F("\n"));

	Serial.print(F("DigT: ")); 
	Serial.print(DigT1); Serial.print(F(", ")); Serial.print(DigT2); Serial.print(F(", ")); Serial.println(DigT3); 
	Serial.print(F("DigP: ")); 
	Serial.print(DigP1); Serial.print(F(", ")); Serial.print(DigP2); Serial.print(F(", ")); Serial.print(DigP3); Serial.print(F(", "));  
	Serial.print(DigP4); Serial.print(F(", ")); Serial.print(DigP5); Serial.print(F(", ")); Serial.print(DigP6); Serial.print(F(", ")); 
	Serial.print(DigP7); Serial.print(F(", ")); Serial.print(DigP8); Serial.print(F(", ")); Serial.println(DigP9); 
	Serial.print(F("DigH: ")); 
	Serial.print(DigH1); Serial.print(F(", ")); Serial.print(DigH2); Serial.print(F(", ")); Serial.print(DigH3); Serial.print(F(", "));  
	Serial.print(DigH4); Serial.print(F(", ")); Serial.print(DigH5); Serial.print(F(", ")); Serial.println(DigH6); 

#endif
}

float *BME280::getSensorData() {
	byte RawSensorData[8];

	byte RawCalibrationData[26];
    i2c_device::read_many(0x88, 26, RawCalibrationData);
	DigT1 = (((uint16_t)(RawCalibrationData[1])<<8) | (uint16_t)(RawCalibrationData[0]));
	DigT2 = (((int16_t)(RawCalibrationData[3])<<8) | (int16_t)(RawCalibrationData[2]));
	DigT3 = (((int16_t)(RawCalibrationData[5])<<8) | (int16_t)(RawCalibrationData[4]));
	DigP1 = (((uint16_t)(RawCalibrationData[7])<<8) | (uint16_t)(RawCalibrationData[6]));
	DigP2 = (((int16_t)(RawCalibrationData[9])<<8) | (int16_t)(RawCalibrationData[8]));
	DigP3 = (((int16_t)(RawCalibrationData[11])<<8) | (int16_t)(RawCalibrationData[10]));
	DigP4 = (((int16_t)(RawCalibrationData[13])<<8) | (int16_t)(RawCalibrationData[12]));
	DigP5 = (((int16_t)(RawCalibrationData[15])<<8) | (int16_t)(RawCalibrationData[14]));
	DigP6 = (((int16_t)(RawCalibrationData[17])<<8) | (int16_t)(RawCalibrationData[16]));
	DigP7 = (((int16_t)(RawCalibrationData[19])<<8) | (int16_t)(RawCalibrationData[18]));
	DigP8 = (((int16_t)(RawCalibrationData[21])<<8) | (int16_t)(RawCalibrationData[20]));
	DigP9 = (((int16_t)(RawCalibrationData[23])<<8) | (int16_t)(RawCalibrationData[22]));
	DigH1 = RawCalibrationData[25];
	
    i2c_device::read_many(0xE1, 7, RawCalibrationData);

	DigH2 = (((int16_t)(RawCalibrationData[1])<<8) | (int16_t)(RawCalibrationData[0]));
	DigH3 = RawCalibrationData[2];
	DigH4 = (((int16_t)(RawCalibrationData[3])<<4) | ((int16_t)(RawCalibrationData[4]) & 0x0F)); 
	DigH5 = ((((int16_t)(RawCalibrationData[5])<<4) | (((int16_t)(RawCalibrationData[4])&0xF0) >> 4)));
	DigH6 = RawCalibrationData[6];


	
    i2c_device::read_many(0xF7, 8, RawSensorData);
	long RawPressure=((((long)(RawSensorData[0])<<16)|((long)(RawSensorData[1])<<8)|(long)(RawSensorData[2]))>>4);
	long RawTemperature=((((long)RawSensorData[3]<<16)|((long)RawSensorData[4]<<8)|(long)RawSensorData[5])>>4);
	long RawHumidity=(((long)RawSensorData[6]<<8)|(long)RawSensorData[7]);

#ifdef BME280_DEBUGMEAS
	for(int x=0; x<8; x++) {
		if (RawSensorData[x]<16) Serial.print(F("0"));
		Serial.print(RawSensorData[x],HEX);
		Serial.print(F(" "));
	}
	Serial.print(F(" Tu: ")); Serial.print(RawTemperature);
	Serial.print(F(" Pu: ")); Serial.print(RawPressure);
	Serial.print(F(" Hu: ")); Serial.print(RawHumidity);
	Serial.print(F("  \n"));
	
	Serial.print(F("DigT: ")); 
	Serial.print(DigT1); Serial.print(F(", ")); Serial.print(DigT2); Serial.print(F(", ")); Serial.println(DigT3); 
	Serial.print(F("DigP: ")); 
	Serial.print(DigP1); Serial.print(F(", ")); Serial.print(DigP2); Serial.print(F(", ")); Serial.print(DigP3); Serial.print(F(", "));  
	Serial.print(DigP4); Serial.print(F(", ")); Serial.print(DigP5); Serial.print(F(", ")); Serial.print(DigP6); Serial.print(F(", ")); 
	Serial.print(DigP7); Serial.print(F(", ")); Serial.print(DigP8); Serial.print(F(", ")); Serial.println(DigP9); 
	Serial.print(F("DigH: ")); 
	Serial.print(DigH1); Serial.print(F(", ")); Serial.print(DigH2); Serial.print(F(", ")); Serial.print(DigH3); Serial.print(F(", "));  
	Serial.print(DigH4); Serial.print(F(", ")); Serial.print(DigH5); Serial.print(F(", ")); Serial.println(DigH6); 
#endif


	int32_t t1=((((RawTemperature>>3)-((int32_t)DigT1 <<1)))*((int32_t)DigT2))>>11;
	int32_t t2=(((((RawTemperature>>4)-((int32_t)DigT1))*((RawTemperature>>4)-((int32_t)DigT1)))>>12)*((int32_t)DigT3))>>14;
	int32_t TempFine=t1+t2;
	BME280value[0]=(float)((float)((TempFine*5+128)>>8)/100)-3.8;
	//Temperature=(float)((TempFine*5+128)>>8)/100;

	int64_t p1, p2;
	int64_t PresFine;
	p1=((int64_t)TempFine)-128000;
	p2=p1*p1*(int64_t)DigP6;
	p2=p2+((p1*(int64_t)DigP5)<<17);
	p2=p2+(((int64_t)DigP4)<<35);
	p1=((p1*p1*(int64_t)DigP3)>>8)+((p1*(int64_t)DigP2)<<12);
	p1=(((((int64_t)1)<<47)+p1))*((int64_t)DigP1)>>33;
	if (p1==0) return 0;  		// avoid exception caused by division by zero
	PresFine=1048576-RawPressure;
	PresFine=(((PresFine<<31)-p2)*3125)/p1;
	p1=(((int64_t)DigP9)*(PresFine>>13)*(PresFine>>13))>>25;
	p2=(((int64_t)DigP8)*PresFine)>>19;
	PresFine=((PresFine+p1+p2)>>8)+(((int64_t)DigP7)<<4);
	BME280value[1]=((float)(((float)(PresFine)/256))/100);
	//Pressure=(float)(((float)(PresFine)/256))/100;
	
	int32_t h1=(TempFine-((int32_t)76800));
	h1=(((((RawHumidity<<14)-(((int32_t)DigH4)<<20)-(((int32_t)DigH5)*h1))+((int32_t)16384))>>15)*
	   (((((((h1*((int32_t)DigH6))>>10)*(((h1*((int32_t)DigH3))>>11)+((int32_t)32768)))>>10)+
       ((int32_t)2097152))*((int32_t)DigH2)+8192)>>14));
	h1=(h1-(((((h1>>15)*(h1>>15))>>7)*((int32_t)DigH1))>>4));
	h1=(h1<0)?0:h1;
	h1=(h1>419430400)?419430400:h1;
	BME280value[2]=((float)(h1>>12)/(float)1024)+14;
	//Humidity=(float)(h1>>12)/(float)1024;
	
#ifdef BME280_DEBUGMEAS
	Serial.print(F("T: ")); Serial.print(BME280value[0]); Serial.print(F(" degrC "));
	Serial.print(F("P: ")); Serial.print(BME280value[1]); Serial.print(F(" hPa "));
	Serial.print(F("H: ")); Serial.print(BME280value[2]); Serial.print(F(" %\n"));
#endif
	return BME280value;
}


