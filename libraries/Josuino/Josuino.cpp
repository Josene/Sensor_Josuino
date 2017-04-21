/************************************************************************************
* Josuino.cpp - Library for controlling Arduino Based Jose indoor sensor           	*
* Copyright (c) 2014-2017 Antoine van de Cruyssen. All rights reserved             	*
*************************************************************************************
* Rev 1.0 - August 2016																*
* - Initial release																	*
* Rev 1.1 - March 2017 																*
* - Added new hardware "D" revisions												*
* Rev 1.2 - April 2017 																*
* - Added new hardware "60" variants 												*
* - Added communication options per variant											*
* - Fixed demo cursor issue                											*
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

#include "Arduino.h"
#include "Josuino.h"
#include <stdarg.h>

pca9635 Icons;
pca9635 RGB[6];
TMP112 TMP112;
BME280 BME280;
BH1750 BH1750;
SC16IS752 SC16IS752;
ADXL345 ADXL345;
T6713 T6713;
JosenePM JosenePM;
M41T00S M41T00S;
PCB150011 PCB150011;
DS2482S DS2482S;

byte RGBLED[96];
byte IconLED[16];
uint8_t TimerCount = 0;
uint8_t CursorPos = 0xFF;

// Declaration of globals used for asynchronically transmit data
uint8_t Vermajor = 1;
uint8_t Verminor = 20;
uint32_t UnitID = 0;
uint32_t PowerState = 0;
uint32_t PowerStateVisual = 1;
uint32_t ConnectionState = 0;
uint32_t ConnectionStateVisual = 0;
uint32_t SessionTimer = 0;
uint32_t TotalUpTimer = 0;
uint16_t PMTimer = 0;
uint8_t PMMode[2] = {DISABLED, POWEREDDOWN};
uint32_t CalibrationTimer = 0;
uint8_t IconRingLEDs = ENABLED;
uint8_t DemoModes = ENABLED;
uint16_t Demo = 0;		// 8 MSB bits = used sensor (thus icon, 8 LSB = demo number
uint16_t DemoVisual = 0;
uint16_t DemoCounter = 0;
uint16_t DemoLED = 0;
uint16_t DemoBackup = 0;
uint16_t DemoActual = 0;
uint16_t DemoCursorActive = OFF;

uint32_t MemoryPointer = 0;
uint32_t BaseTimer = 0;
uint32_t HardwareRev = 0;				// Describes used PCB revision
uint32_t HardwareConfig = 0;			// Describes installed optional modules
uint32_t HardwareCommConfig = 0;		// Describes installed communication options

float UnitTemperature = 0;
float ExtTemperature = 0;
float Temperature = 0;
float Humidity = 0;
float Pressure = 0;
uint16_t LightIntensity = 0;
float AcceleroX = 0;
float AcceleroY = 0;
float DeviceAngle = 0;
float AcceleroI = 0;
uint16_t CO2gas = 0;
uint16_t PM10 = 0;
uint16_t PM2_5 = 0;
uint16_t CO2Status = 0;
uint16_t TxDataCounter = 100; //TXTIMERMAX;
uint8_t RTC[7] = {0,0,0,1,0,0,0};
uint8_t OldSec = 0;
uint32_t Longitude = 0;
uint32_t Latitude = 0;
uint8_t LaEQ[124]={ };

/************************************************************************************
*************************************************************************************
* Communications class																*
*************************************************************************************
************************************************************************************/
Comms::Comms() {

}

/***********************************************************
* Universal kind of printf for all 4 ways of communication *
************************************************************
* Use: Comms::txData(IF_USB,"Count %d, unit temperature: %f degr C\n",Test,Sensor.GetUnitTemperature());
***********************************************************/
int Comms::txData(uint8_t port, char const *str, ...) {
	uint8_t i, j, count = 0;
	String TxStr = "";
	va_list argv;
	va_start(argv, str);
	Utils::ctrlDebugLEDs(port,TOGGLE);
	for(i=0,j=0;str[i]!='\0';i++) {
		if (str[i]=='%') {
			count++;
			TxStr="";
			switch(port) {
				case IF_USB: Serial.write(reinterpret_cast<const uint8_t*>(str+j), i-j);break;
				case IF_WiFi: Serial1.write(reinterpret_cast<const uint8_t*>(str+j), i-j);break;
				case IF_LoRa: Serial2.write(reinterpret_cast<const uint8_t*>(str+j), i-j);break;
				case IF_MiRa: Serial3.write(reinterpret_cast<const uint8_t*>(str+j), i-j);break;
			};
			switch (str[++i]) {
				case 'd': TxStr=(va_arg(argv, int));break;
				case 'D': TxStr=(va_arg(argv, int)); if(TxStr.length()==1) TxStr="0"+TxStr; break;
				case 'u': TxStr=(va_arg(argv, uint32_t));break;
				case 'l': TxStr=(va_arg(argv, long));break;
				case 'f': TxStr=(va_arg(argv, double));break;
				case 'H': Comms::txHex8(port, va_arg(argv,uint16_t)); break;
				case 'h': Comms::txHex32(port, va_arg(argv,uint32_t)); break;
				case 'w': Comms::txDow(port, va_arg(argv,uint16_t)); break;
				case 'c': TxStr=((char) va_arg(argv, int));break;
				case 's': TxStr=(va_arg(argv, char *));break;
				case '%': TxStr="%";break;
				case '>': TxStr=char(2);break;
				case '<': TxStr=char(3);break;
				case '|': TxStr=char(0xDC);break;
				default:;
			};
			switch(port) {
				case IF_USB: if(TxStr!="")Serial.print(TxStr);break;
				case IF_WiFi: if(TxStr!="")Serial1.print(TxStr);break;
				case IF_LoRa: if(TxStr!="")Serial2.print(TxStr);break;
				case IF_MiRa: if(TxStr!="")Serial3.print(TxStr);break;
			};
			j = i+1;
		}
	};
	va_end(argv);

	if(i>j) {
		switch(port) {
			case IF_USB: Serial.write(reinterpret_cast<const uint8_t*>(str+j), i-j);break;
			case IF_WiFi: Serial1.write(reinterpret_cast<const uint8_t*>(str+j), i-j);break;
			case IF_LoRa: Serial2.write(reinterpret_cast<const uint8_t*>(str+j), i-j);break;
			case IF_MiRa: Serial3.write(reinterpret_cast<const uint8_t*>(str+j), i-j);break;
		};
	}
	Utils::ctrlDebugLEDs(port,TOGGLE);
	return count;
}

void Comms::txHex32(uint8_t port, uint32_t n) {
	char buf[8 * sizeof(uint32_t) + 1]; 
	char *str = &buf[sizeof(buf) - 1];
	*str = '\0';
	uint8_t cnt=8;
	do {									// first do conversion of uint32_t
		cnt--;
		char c=n%16;
		n/=16;
		*--str=c<10?c+'0':c+'A'-10;
	} while(n);
	if(cnt){ 
		do {								// then fill with leading zero's
			*--str='0';
			cnt--;
		} while(cnt);
	}
	switch(port) {
		case IF_USB: Serial.print(str);break;
		case IF_WiFi: Serial1.print(str);break;
		case IF_LoRa: Serial2.print(str);break;
		case IF_MiRa: Serial3.print(str);break;
	};
	return;
}

void Comms::txHex8(uint8_t port, uint8_t n) {
	char buf[8 * sizeof(uint32_t) + 1]; 
	char *str = &buf[sizeof(buf) - 1];
	*str = '\0';
	uint8_t cnt=2;
	do {									// first do conversion of uint32_t
		cnt--;
		char c=n%16;
		n/=16;
		*--str=c<10?c+'0':c+'A'-10;
	} while(n);
	if(cnt){ 
		do {								// then fill with leading zero's
			*--str='0';
			cnt--;
		} while(cnt);
	}
	switch(port) {
		case IF_USB: Serial.print(str);break;
		case IF_WiFi: Serial1.print(str);break;
		case IF_LoRa: Serial2.print(str);break;
		case IF_MiRa: Serial3.print(str);break;
	};
	return;
}

void Comms::txDow(uint8_t port, uint8_t n) {
	String dow;
	switch(n) {
		case 1: dow = "Mon";break;
		case 2: dow = "Tue";break;
		case 3: dow = "Wed";break;
		case 4: dow = "Thu";break;
		case 5: dow = "Fri";break;
		case 6: dow = "Sat";break;
		case 7: dow = "Sun";break;
		default: dow = "-?-";break;
	};
	switch(port) {
		case IF_USB: Serial.print(dow);break;
		case IF_WiFi: Serial1.print(dow);break;
		case IF_LoRa: Serial2.print(dow);break;
		case IF_MiRa: Serial3.print(dow);break;
	};

}

/*****************************************
* Determine protocol and parse correctly *
*****************************************/ 

void Comms::parseData(uint8_t port, String& cmd) {
	byte c[cmd.length()+1] = { };
	uint8_t result=0;
	uint8_t detprotocol=PR_VERBOSE;
	cmd.replace("\n","");
	cmd.replace("\r","");

	//Comms::txData(IF_USB,"Length %d, port %d, protocol ",cmd.length(),port);

	if(cmd.length()==12){									// S2900001F40& (length=12 with STX/ETX)
		cmd.getBytes(c,cmd.length()+1);
		for(uint8_t x=1; x<=10; x++) if(isHexadecimalDigit(c[x])) result++;
		if(((c[11]==38)||(c[11]==36))&&(result==10)) {		// Detect closing character & or $
			cmd = char(2) + cmd + char(3);					// Add STX and ETX for correct length
			detprotocol=PR_JOSE;
		}
	} 
	else if(cmd.length()==14) {								// <02>S2900001F40&<03> (length=14 with STX/ETX)
		cmd.getBytes(c,cmd.length()+1);
		for(uint8_t x=2; x<=11; x++) if(isHexadecimalDigit(c[x])) result++;
		if(((c[12]==38)||(c[12]==36))&&(result==10)) {		// Detect closing character & or $
			detprotocol=PR_JOSE;
		}
	}
	
	//if (detprotocol==0x10) Serial.println("Jose");
	//if (detprotocol==0x11) Serial.println("Verbose");
	
	if(detprotocol==PR_VERBOSE) Comms::parseVerboseProtocol(port, cmd);
	if(detprotocol==PR_JOSE) Comms::parseJoseProtocol(port, cmd);
}


/**********************
* Parse Jose Protocol *
**********************/
int Comms::parseJoseProtocol(uint8_t port, String& cmd) {
	byte c[cmd.length()] = { };
	cmd.getBytes(c,cmd.length());
	uint8_t Cmd = c[1];
	uint8_t Opr = Comms::arrToUint8(c,2);
	uint32_t Opd = Comms::arrToUint32(c,4);

//	Serial.print(char(Cmd)); Serial.print(",");
//	Serial.print(Opr, HEX); Serial.print(",");
//	Serial.print(Opd, HEX); Serial.print("\n");
		
	switch (Cmd) {
		case 'F':
			if(port!=IF_USB) Comms::txData(IF_USB,"Ext. Sensor Request, Opr: 0x%H, Opd: 0x%h\n",Opr,Opd);
			if(((EXTREQ_SENSOR + Opr)==EXTREQ_TIME)&&((Opd&0x80000000)!=0)) {
				RTC[SECONDS] = (uint8_t)(Opd&0x000000FF);
				RTC[MINUTES] = (uint8_t)((Opd&0x0000FF00)>>8);
				RTC[HOURS] = (uint8_t)((Opd&0x00FF0000)>>16);
				M41T00S.setData(RTC);
				Sensor::getRTC();
			}
			if(((EXTREQ_SENSOR + Opr)==EXTREQ_DATE)&&((Opd&0x80000000)!=0)) {
				RTC[YEAR] =  (uint8_t)((Opd&0x00FF0000)>>16);
				RTC[MONTH] = (uint8_t)((Opd&0x0000F000)>>12);
				RTC[DATE] =  (uint8_t)((Opd&0x00000FF0)>>4);
				RTC[DAY] =   (uint8_t)(Opd&0x0000000F);
				M41T00S.setData(RTC);
				Sensor::getRTC();
			}
			Comms::txSensor(port, PR_JOSE, (EXTREQ_SENSOR + Opr)); 
			break;
		case 'C':
			if(port!=IF_USB) Comms::txData(IF_USB,"Ext. Control Request, Opr: 0x%H, Opd: 0x%h\n",Opr,Opd);
			if((EXTREQ_CTRL + Opr)==EXTREQ_CO2UTILS) {
				if(Opd==0x82000000) Sensor::askCO2Calibration();
			}
			if((EXTREQ_CTRL + Opr)==EXTREQ_RESET){
				if(Opd==0x13370539) asm volatile ("  jmp 0");
			}
			if((EXTREQ_CTRL + Opr)==EXTREQ_ALL) {
				for(uint8_t x=PUSH_MAX;x>0;x--) {
					Comms::txSensor(port, PR_JOSE, x);
					delay(30);
				}
				PCB150011.clearThirdsSpectrum(LaEQ);
			}
			Comms::txSensor(port, PR_JOSE, (EXTREQ_CTRL + Opr)); 
			break;
		case 'M':
			if(port!=IF_USB) Comms::txData(IF_USB,"Ext. Memory Request, Opr: 0x%H, Opd: 0x%h\n",Opr,Opd);
			if(Opr>=0x01 && Opr<=0x04) {
				MemoryPointer=Opd;
				Comms::txMemory(port,Utils::readEEPROM(Opd,Opr),Opr,RD);
			}
			if(Opr>=0x81 && Opr<=0x84) {
				Comms::txMemory(port,Utils::writeEEPROM(MemoryPointer,Opd,(Opr&0x7F)),(Opr&0x7F),WR);
			}
			break;
		case 'L':
			if(port!=IF_USB) Comms::txData(IF_USB,"Ext. Demo Request, Opr: 0x%H, Opd: 0x%h\n",Opr,Opd);
			if(Opr==0xDE) {
				Comms::txData(port,"%>YDE%h&%<", (uint32_t)(Opd));				
				Demo=Opd+0x80;
			}
			break;
		default:
			break;
	}

}

uint8_t Comms::arrToUint8(uint8_t d[], uint8_t s) {
	uint8_t upper = d[s];
	uint8_t lower = d[s+1];
	if(upper>=0x30 && upper<=0x39) upper=(upper-0x30)<<4;
	if(upper>=0x41 && upper<=0x46) upper=(upper-0x37)<<4;
	if(lower>=0x30 && lower<=0x39) lower=lower-0x30;
	if(lower>=0x41 && lower<=0x46) lower=lower-0x37;
	return (upper+lower);
}

uint32_t Comms::arrToUint32(uint8_t d[], uint8_t s) {
	uint32_t result = 0;
	result = ((uint32_t)arrToUint8(d,s) << 24) | ((uint32_t)arrToUint8(d,s+2) << 16) | ((uint32_t)arrToUint8(d,s+4) << 8) | arrToUint8(d,s+6);
	return result;
}

/**************************************
* Parse readable commandline commands *
**************************************/
int Comms::parseVerboseProtocol(uint8_t port, String& cmd) {
	cmd.toLowerCase();
	if (cmd=="reset audio") {
		PCB150011.clearThirdsSpectrum(LaEQ);
		return 1;
	}
	if (cmd=="get all") {
		for(uint8_t x=PUSH_MAX;x>0;x--) {
			Comms::txSensor(port, PR_VERBOSE, x);
			
		}
		PCB150011.clearThirdsSpectrum(LaEQ);
		return 1;
	}
	if (cmd=="get accelerometer" || cmd=="get accelero"){
		Comms::txSensor(port, PR_VERBOSE, SENSOR_ACCELEROX);
		Comms::txSensor(port, PR_VERBOSE, SENSOR_ACCELEROY);
		return 1;
	}
	if (cmd=="get location"){
		Comms::txSensor(port, PR_VERBOSE, SENSOR_LATITUDE);
		Comms::txSensor(port, PR_VERBOSE, SENSOR_LONGITUDE);
		return 1;
	}
	if (cmd=="get id") {
		Comms::txSensor(port, PR_VERBOSE, SENSOR_ID);
		return 1;
	}
	if (cmd=="get session") {
		Comms::txSensor(port, PR_VERBOSE, SENSOR_SESSIONTIMER);
		return 1;
	}
	if (cmd=="get total") {
		Comms::txSensor(port, PR_VERBOSE, SENSOR_TOTALUPTIMER);
		return 1;
	}
	if (cmd=="get caltimer") {
		Comms::txSensor(port, PR_VERBOSE, SENSOR_CALTIMER);
		return 1;
	}
	if (cmd=="get time") {
		Comms::txSensor(port, PR_VERBOSE, SENSOR_TIME);
		return 1;
	}
	if (cmd=="get date") {
		Comms::txSensor(port, PR_VERBOSE, SENSOR_DATE);
		return 1;
	}
	if (cmd=="get power") {
		Comms::txSensor(port, PR_VERBOSE, SENSOR_POWERSTATE);
		return 1;
	}
	if (cmd=="get angle") {
		Comms::txSensor(port, PR_VERBOSE, SENSOR_DEVICEANGLE);
		return 1;
	}
	if (cmd=="get co2") {
		Comms::txSensor(port, PR_VERBOSE, SENSOR_CO2);
		return 1;
	}
	if (cmd=="get exttemperature" || cmd=="get exttemp") {
		if((HardwareConfig&CONFIG_EXTTEMPSENSOR)!=DISABLED){ 
			Comms::txSensor(port, PR_VERBOSE, SENSOR_EXTTEMPERATURE);
			return 1;
		}
	}
	if (cmd=="get unittemperature" || cmd=="get unittemp") {
		Comms::txSensor(port, PR_VERBOSE, SENSOR_UNITTEMPERATURE);
		return 1;
	}
	if (cmd=="get temperature" || cmd=="get temp") {
		Comms::txSensor(port, PR_VERBOSE, SENSOR_TEMPERATURE);
		return 1;
	}
	if (cmd=="get humidity") {
		Comms::txSensor(port, PR_VERBOSE, SENSOR_HUMIDITY);
		return 1;
	}
	if (cmd=="get pressure") {
		Comms::txSensor(port, PR_VERBOSE, SENSOR_PRESSURE);
		return 1;
	}
	if (cmd=="get light") {
		Comms::txSensor(port, PR_VERBOSE, SENSOR_LIGHTINTENSITY);
		return 1;
	}
	if (cmd=="get pm10") {
		Comms::txSensor(port, PR_VERBOSE, SENSOR_PM10);
		return 1;
	}
	if (cmd=="get pm2.5") {
		Comms::txSensor(port, PR_VERBOSE, SENSOR_PM2_5);
		return 1;
	}
	if (cmd=="get pm") {
		Comms::txSensor(port, PR_VERBOSE, SENSOR_PM10);
		Comms::txSensor(port, PR_VERBOSE, SENSOR_PM2_5);
		return 1;
	}
	if (cmd=="calibrate co2" || cmd=="cal co2") {
		if(T6713.checkPower()==true) {
			Comms::txData(port,"CO2 sensor calibration started\n");
			Sensor::askCO2Calibration();
		} else {
			Comms::txData(port,"CO2 sensor is switched off\n",CO2gas); 
		}
		return 1;
	}
	if (cmd=="co2 on")	{
		Utils::powerCO2(ENABLED);
		return 1;
	}
	if (cmd=="co2 off")	{
		Utils::powerCO2(DISABLED);
		return 1;
	}
	if (cmd=="pm on")	{
		PMMode[MEASMODE]==ENABLED;
		Utils::powerPM(ENABLED);
		return 1;
	}
	if (cmd=="pm off")	{
		PMMode[MEASMODE]==DISABLED;
		Utils::powerPM(DISABLED);
		return 1;
	}
	if (cmd=="debugleds on") {
		Utils::enableDebugLEDs();
		return 1;
	}
	if (cmd=="debugleds off") {
		Utils::disableDebugLEDs();
		return 1;
	}
	if (cmd=="help") {
		Utils::showHelp();
		return 1;		
	}
	if (cmd=="comms query")	{
		Utils::queryWiFi();
		Utils::queryLoRa();
		Utils::queryMiRa();
		Utils::checkUSB();
		return 1;
	}
	if (cmd=="wifi on")	{
		Utils::enableWiFi();
		return 1;
	}
	if (cmd=="wifi off")	{
		Utils::disableWiFi();
		return 1;
	}
	if (cmd=="wifi recover")	{
		Utils::assertDefaultWiFi();
		delay(1000);
		Utils::enableWiFi();
		delay(1000);
		Utils::deAssertDefaultWiFi();
		return 1;
	}
	if (cmd=="wifi" || cmd== "wifi query") {
		Utils::queryWiFi();
		return 1;
	}
	if (cmd=="lora on")	{
		Utils::enableLoRa();
		return 1;
	}
	if (cmd=="lora off")	{
		Utils::disableLoRa();
		return 1;
	}
	if (cmd=="mira on") {
         Utils::enableMiRa();
         return 1;
    }
    if (cmd=="mira off") {
         Utils::disableMiRa();
         return 1;
    }
    if (cmd=="mira" || cmd== "mira query") {
         Utils::queryMiRa();
         return 1;
    }
	if (cmd=="lora" || cmd== "lora query") {
		Utils::queryLoRa();
		return 1;
	}
	if (cmd=="usb" || cmd== "usb query") {
		Utils::checkUSB();
		return 1;
	}
    Comms::txData(port,"This is not a valid command\n"); 
	return 0;
}

/*****************************************************
* Tx sensor in various formats to various interfaces *
*****************************************************/
void Comms::txSensor(uint8_t port, uint8_t protocol, uint16_t sns) {
	char buffer[14];
//	for (int i = 0; i < 6; i++)
//	{
//		strcpy_P(buffer, (char*)pgm_read_word(&(string_table[i]))); // Necessary casts and dereferencing, just copy.
//		Serial.println(buffer);
//	}

	switch (sns){
		case EXTREQ_ID :
		case SENSOR_ID :
			if(protocol==PR_VERBOSE) Comms::txData(port,"Unit ID: %u\n",UnitID);
			if(protocol==PR_JOSE) Comms::txData(port,"%>PA0%h&%<", (uint32_t)(UnitID));
			break;
		case EXTREQ_SESSIONTIMER :
		case SENSOR_SESSIONTIMER :
			if(protocol==PR_VERBOSE) Comms::txData(port,"Session uptime: %ud %D:%D:%D\n",(SessionTimer/86400),(uint8_t)((SessionTimer%86400)/3600),(uint8_t)((SessionTimer%3600)/60),(uint8_t)(SessionTimer%60));
			if(protocol==PR_JOSE) Comms::txData(port,"%>P0E%h&%<", (uint32_t)(SessionTimer));
			break;
		case EXTREQ_TOTALUPTIMER :
		case SENSOR_TOTALUPTIMER :
			if(protocol==PR_VERBOSE) Comms::txData(port,"Total uptime: %ud %D:%D:%D\n",((Utils::readEEPROM(MEM_UPTIME,4)+TotalUpTimer)/86400),(uint8_t)(((Utils::readEEPROM(MEM_UPTIME,4)+TotalUpTimer)%86400)/3600),(uint8_t)(((Utils::readEEPROM(MEM_UPTIME,4)+TotalUpTimer)%3600)/60),(uint8_t)((Utils::readEEPROM(MEM_UPTIME,4)+TotalUpTimer)%60));
			if(protocol==PR_JOSE) Comms::txData(port,"%>P0F%h&%<", (uint32_t)(Utils::readEEPROM(MEM_UPTIME)+TotalUpTimer));
			break;
		case EXTREQ_CALTIMER :
		case SENSOR_CALTIMER :
			if(protocol==PR_VERBOSE) Comms::txData(port,"Calibration in: %ud %D:%D:%D\n",(CalibrationTimer/86400),(uint8_t)((CalibrationTimer%86400)/3600),(uint8_t)((CalibrationTimer%3600)/60),(uint8_t)(CalibrationTimer%60));
			if(protocol==PR_JOSE) Comms::txData(port,"%>P0A%h&%<", (uint32_t)(CalibrationTimer));
			break;
		case EXTREQ_TIME :
		case SENSOR_TIME :
			if(protocol==PR_VERBOSE) Comms::txData(port,"Time: %D:%D:%D\n",RTC[HOURS],RTC[MINUTES],RTC[SECONDS]);
			if(protocol==PR_JOSE) Comms::txData(port,"%>S0E%h&%<", (uint32_t)(((uint32_t)RTC[HOURS] << 16) | ((uint32_t)RTC[MINUTES] << 8) | RTC[SECONDS]));
			break;
		case EXTREQ_DATE :
		case SENSOR_DATE :
			if(protocol==PR_VERBOSE) Comms::txData(port,"Date: %w %D-%D-%D\n",RTC[DAY],RTC[DATE],RTC[MONTH],RTC[YEAR]);
			if(protocol==PR_JOSE) Comms::txData(port,"%>S0F%h&%<", (uint32_t)(((uint32_t)RTC[YEAR] << 16) | ((uint32_t)RTC[MONTH] << 12) | ((uint32_t)RTC[DATE] << 4) | RTC[DAY]));
			break;
		case EXTREQ_POWERSTATE :
		case SENSOR_POWERSTATE :
			if(protocol==PR_VERBOSE) Comms::txData(port,"Powerstate: 0x%h\n",PowerState);
			if(protocol==PR_JOSE) Comms::txData(port,"%>P00%h&%<", (uint32_t)(PowerState));
			break;
		case EXTREQ_UNITTEMPERATURE :
		case SENSOR_UNITTEMPERATURE :
			if(protocol==PR_VERBOSE) Comms::txData(port,"Unit temperature: %f degr C\n",UnitTemperature);
			if(protocol==PR_JOSE) Comms::txData(port,"%>S11%h&%<", (uint32_t)((float)(UnitTemperature*1000)+273150));
			break;
		case EXTREQ_EXTTEMPERATURE :
		case SENSOR_EXTTEMPERATURE :
			if((HardwareConfig&CONFIG_EXTTEMPSENSOR)!=DISABLED){
				if(ExtTemperature==0xFFFF) {
					if(protocol==PR_VERBOSE) Comms::txData(port,"Ext. temperature sensor error\n");
					if(protocol==PR_JOSE) Comms::txData(port,"%>S2500000000&%<");
				} else {
					if(protocol==PR_VERBOSE) Comms::txData(port,"Ext. temperature: %f degr C\n",ExtTemperature);
					if(protocol==PR_JOSE) Comms::txData(port,"%>S25%h&%<", (uint32_t)((float)(ExtTemperature*1000)+273150));
				}
			}
			break;
		case EXTREQ_TEMPERATURE :
		case SENSOR_TEMPERATURE :
			if(protocol==PR_VERBOSE) Comms::txData(port,"Temperature: %f degr C\n",Temperature);
			if(protocol==PR_JOSE) Comms::txData(port,"%>S12%h&%<", (uint32_t)((float)(Temperature*1000)+273150));
			break;
		case EXTREQ_HUMIDITY :
		case SENSOR_HUMIDITY :
			if(protocol==PR_VERBOSE) Comms::txData(port,"Humidity: %f %%RH\n",Humidity);
			if(protocol==PR_JOSE) Comms::txData(port,"%>S13%h&%<", (uint32_t)((float)(Humidity*1000)));
			break;
		case EXTREQ_PRESSURE :
		case SENSOR_PRESSURE :
			if(protocol==PR_VERBOSE) Comms::txData(port,"Pressure: %f hPa\n",Pressure);
			if(protocol==PR_JOSE) Comms::txData(port,"%>S16%h&%<", (uint32_t)((float)(Pressure*100)));
			break;
		case EXTREQ_LIGHTINTENSITY :
		case SENSOR_LIGHTINTENSITY :
			if(protocol==PR_VERBOSE) Comms::txData(port,"Light Intensity: %d lux\n",LightIntensity);
			if(protocol==PR_JOSE) Comms::txData(port,"%>S14%h&%<", (uint32_t)(LightIntensity));
			break;
		case EXTREQ_ACCELEROX :
		case SENSOR_ACCELEROX :
			if(protocol==PR_VERBOSE) Comms::txData(port,"Accelerometer X: %f G\n",AcceleroX);
			if(protocol==PR_JOSE)Comms::txData(port,"%>S1A%h&%<", (uint32_t)(float)((AcceleroX*256)+512));
			break;
		case EXTREQ_ACCELEROY :
		case SENSOR_ACCELEROY :
			if(protocol==PR_VERBOSE) Comms::txData(port,"Accelerometer Y: %f G\n",AcceleroY);
			if(protocol==PR_JOSE) Comms::txData(port,"%>S1B%h&%<", (uint32_t)(float)((AcceleroY*256)+512));
			break;
		case EXTREQ_ACCELEROZ :
		case SENSOR_ACCELEROZ :
			break;
		case EXTREQ_DEVICEANGLE :
		case SENSOR_DEVICEANGLE :
			if(DeviceAngle==DeviceAngle) {
				if(protocol==PR_VERBOSE) Comms::txData(port,"Device angle: %f degr\n",DeviceAngle); 
				if(protocol==PR_JOSE) Comms::txData(port,"%>S2F%h&%<", (uint32_t)((DeviceAngle)*1000));
			} else {
				if(protocol==PR_VERBOSE) Comms::txData(port,"Device lays flat\n"); 
				if(protocol==PR_JOSE) Comms::txData(port,"%>S2F%h&%<", (uint32_t)(0xFFFFFFFF));
		}	
			break;
		case EXTREQ_CO2UTILS :
		case EXTREQ_CO2 :
		case SENSOR_CO2 :
			if(T6713.checkPower()==true) {
				if(CO2Status!=0) {
					if((CO2Status & 2048) != 0)	{
						if(protocol==PR_VERBOSE) Comms::txData(port,"CO2: %d ppm, status: Warming up\n",CO2gas); 
						if(protocol==PR_JOSE) Comms::txData(port,"%>P13%h&%<", (uint32_t)(0x01010000));
					} else if((CO2Status & 32768) != 0) {
						if(protocol==PR_VERBOSE) Comms::txData(port,"CO2: %d ppm, status: Calibration\n",CO2gas); 
						if(protocol==PR_JOSE) Comms::txData(port,"%>P13%h&%<", (uint32_t)(0x01020000));
					} else {
						if(protocol==PR_VERBOSE) Comms::txData(port,"CO2: %d ppm, status: Unknown > %u \n",CO2gas, CO2Status); 
						if(protocol==PR_JOSE) Comms::txData(port,"%>S23%h&%<", ((uint32_t)(CO2gas)*(uint32_t)(1000)));
					}
				} else { 
					if(protocol==PR_VERBOSE) Comms::txData(port,"CO2: %d ppm\n",CO2gas); 
					if(protocol==PR_JOSE) Comms::txData(port,"%>S23%h&%<", ((uint32_t)(CO2gas)*(uint32_t)(1000)));
				}
			} else {
				if(protocol==PR_VERBOSE) Comms::txData(port,"CO2 sensor is switched off\n",CO2gas); 
				if(protocol==PR_JOSE) Comms::txData(port,"%>P13%h&%<", (uint32_t)(0x00000000));
			}
			break;
		case EXTREQ_PM10 :		
		case SENSOR_PM10 :		
			if(PMMode[MEASMODE]!=CYCLIC) {
				if(JosenePM.checkPower()==true) {
					if(protocol==PR_VERBOSE) Comms::txData(port,"PM10: %d ug/m3\n",PM10);
					if(protocol==PR_JOSE) Comms::txData(port,"%>S29%h&%<", ((uint32_t)(PM10)*(uint32_t)(1000)));
				} else {
					if(protocol==PR_VERBOSE) Comms::txData(port,"PM10 sensor is switched off\n"); 
					if(protocol==PR_JOSE) Comms::txData(port,"%>P16%h&%<", (uint32_t)(0x00000000));
				}
			} else {
				if(PMMode[POWERMODE]==POWEREDUP) {
					if(protocol==PR_VERBOSE) Comms::txData(port,"PM10: %d ug/m3\n",PM10);
					if(protocol==PR_JOSE) Comms::txData(port,"%>S29%h&%<", ((uint32_t)(PM10)*(uint32_t)(1000)));
				} else {
					if(protocol==PR_VERBOSE) Comms::txData(port,"PM10: %d ug/m3 (historic)\n",PM10);
					if(protocol==PR_JOSE) Comms::txData(port,"%>S29%h&%<", ((uint32_t)(PM10)*(uint32_t)(1000)));
				}
			}
			break;
		case EXTREQ_PM2_5 :
		case SENSOR_PM2_5 :
			if(PMMode[MEASMODE]!=CYCLIC) {
				if(JosenePM.checkPower()==true) {
					if(protocol==PR_VERBOSE) Comms::txData(port,"PM2.5: %d ug/m3\n",PM2_5);
					if(protocol==PR_JOSE) Comms::txData(port,"%>S2A%h&%<", ((uint32_t)(PM2_5)*(uint32_t)(1000)));
				} else {
					if(protocol==PR_VERBOSE) Comms::txData(port,"PM2.5 sensor is switched off\n"); 
					if(protocol==PR_JOSE) Comms::txData(port,"%>P16%h&%<", (uint32_t)(0x00000000));
				}
			} else {
				if(PMMode[POWERMODE]==POWEREDUP) {
					if(protocol==PR_VERBOSE) Comms::txData(port,"PM2.5: %d ug/m3\n",PM2_5);
					if(protocol==PR_JOSE) Comms::txData(port,"%>S2A%h&%<", ((uint32_t)(PM2_5)*(uint32_t)(1000)));
				} else {
					if(protocol==PR_VERBOSE) Comms::txData(port,"PM2.5: %d ug/m3 (historic)\n",PM2_5);
					if(protocol==PR_JOSE) Comms::txData(port,"%>S2A%h&%<", ((uint32_t)(PM2_5)*(uint32_t)(1000)));
				}
			}
			break;
		case EXTREQ_PM1 :
		case SENSOR_PM1 :
			break;
		
		case EXTREQ_AUDIO_OCT0_CUR :
		case SENSOR_AUDIO_OCT0_CUR :
			if((HardwareConfig&CONFIG_AUDIOPRESSURE)!=DISABLED) {
				strcpy_P(buffer,(char*)pgm_read_word(&(JStr[J_S35]))); // "%>S3500%H%H%H&%<"
				if(protocol==PR_JOSE) Comms::txData(port, buffer, LaEQ[AUDIOBAND16+CUR], LaEQ[AUDIOBAND15+CUR], LaEQ[AUDIOBAND14+CUR]);
			}
			break;
		case SENSOR_AUDIO_OCT0_MAX :
			if((HardwareConfig&CONFIG_AUDIOPRESSURE)!=DISABLED) {
				strcpy_P(buffer,(char*)pgm_read_word(&(JStr[J_T35]))); // "%>T3500%H%H%H&%<"
				if(protocol==PR_JOSE) Comms::txData(port, buffer, LaEQ[AUDIOBAND16+MAX], LaEQ[AUDIOBAND15+MAX], LaEQ[AUDIOBAND14+MAX]);
			}
			break;
		case SENSOR_AUDIO_OCT0_MIN :
			if((HardwareConfig&CONFIG_AUDIOPRESSURE)!=DISABLED) {
				strcpy_P(buffer,(char*)pgm_read_word(&(JStr[J_U35]))); // "%>U3500%H%H%H&%<"
				if(protocol==PR_JOSE) Comms::txData(port, buffer, LaEQ[AUDIOBAND16+MIN], LaEQ[AUDIOBAND15+MIN], LaEQ[AUDIOBAND14+MIN]);
			}
			break;
		case SENSOR_AUDIO_OCT0_AVG :
			if((HardwareConfig&CONFIG_AUDIOPRESSURE)!=DISABLED) {
				strcpy_P(buffer,(char*)pgm_read_word(&(JStr[J_V35]))); // "%>V3500%H%H%H&%<"
				if(protocol==PR_JOSE) Comms::txData(port, buffer, LaEQ[AUDIOBAND16+AVG], LaEQ[AUDIOBAND15+AVG], LaEQ[AUDIOBAND14+AVG]);
			}
			break;
			
		case EXTREQ_AUDIO_OCT1_CUR :
		case SENSOR_AUDIO_OCT1_CUR :
			if((HardwareConfig&CONFIG_AUDIOPRESSURE)!=DISABLED) {
				strcpy_P(buffer,(char*)pgm_read_word(&(JStr[J_S36]))); // "%>S3600%H%H%H&%<"
				if(protocol==PR_JOSE) Comms::txData(port, buffer, LaEQ[AUDIOBAND19+CUR], LaEQ[AUDIOBAND18+CUR], LaEQ[AUDIOBAND17+CUR]);
			}
			break;
		case SENSOR_AUDIO_OCT1_MAX :
			if((HardwareConfig&CONFIG_AUDIOPRESSURE)!=DISABLED) {
				strcpy_P(buffer,(char*)pgm_read_word(&(JStr[J_T36]))); // "%>T3600%H%H%H&%<"
				if(protocol==PR_JOSE) Comms::txData(port, buffer, LaEQ[AUDIOBAND19+MAX], LaEQ[AUDIOBAND18+MAX], LaEQ[AUDIOBAND17+MAX]);
			}
			break;
		case SENSOR_AUDIO_OCT1_MIN :
			if((HardwareConfig&CONFIG_AUDIOPRESSURE)!=DISABLED) {
				strcpy_P(buffer,(char*)pgm_read_word(&(JStr[J_U36]))); // "%>U3600%H%H%H&%<"
				if(protocol==PR_JOSE) Comms::txData(port, buffer, LaEQ[AUDIOBAND19+MIN], LaEQ[AUDIOBAND18+MIN], LaEQ[AUDIOBAND17+MIN]);
			}
			break;
		case SENSOR_AUDIO_OCT1_AVG :
			if((HardwareConfig&CONFIG_AUDIOPRESSURE)!=DISABLED) {
				strcpy_P(buffer,(char*)pgm_read_word(&(JStr[J_V36]))); // "%>V3600%H%H%H&%<"
				if(protocol==PR_JOSE) Comms::txData(port, buffer, LaEQ[AUDIOBAND19+AVG], LaEQ[AUDIOBAND18+AVG], LaEQ[AUDIOBAND17+AVG]);
			}
			break;

		case EXTREQ_AUDIO_OCT2_CUR :
		case SENSOR_AUDIO_OCT2_CUR :
			if((HardwareConfig&CONFIG_AUDIOPRESSURE)!=DISABLED) {
				strcpy_P(buffer,(char*)pgm_read_word(&(JStr[J_S37]))); // "%>S3700%H%H%H&%<"
				if(protocol==PR_JOSE) Comms::txData(port, buffer, LaEQ[AUDIOBAND22+CUR], LaEQ[AUDIOBAND21+CUR], LaEQ[AUDIOBAND20+CUR]);
			}
			break;
		case SENSOR_AUDIO_OCT2_MAX :
			if((HardwareConfig&CONFIG_AUDIOPRESSURE)!=DISABLED) {
				strcpy_P(buffer,(char*)pgm_read_word(&(JStr[J_T37]))); // "%>T3700%H%H%H&%<"
				if(protocol==PR_JOSE) Comms::txData(port, buffer, LaEQ[AUDIOBAND22+MAX], LaEQ[AUDIOBAND21+MAX], LaEQ[AUDIOBAND20+MAX]);
			}
			break;
		case SENSOR_AUDIO_OCT2_MIN :
			if((HardwareConfig&CONFIG_AUDIOPRESSURE)!=DISABLED) {
				strcpy_P(buffer,(char*)pgm_read_word(&(JStr[J_U37]))); // "%>U3700%H%H%H&%<"
				if(protocol==PR_JOSE) Comms::txData(port, buffer, LaEQ[AUDIOBAND22+MIN], LaEQ[AUDIOBAND21+MIN], LaEQ[AUDIOBAND20+MIN]);
			}
			break;
		case SENSOR_AUDIO_OCT2_AVG :
			if((HardwareConfig&CONFIG_AUDIOPRESSURE)!=DISABLED) {
				strcpy_P(buffer,(char*)pgm_read_word(&(JStr[J_V37]))); // "%>V3700%H%H%H&%<"
				if(protocol==PR_JOSE) Comms::txData(port, buffer, LaEQ[AUDIOBAND22+AVG], LaEQ[AUDIOBAND21+AVG], LaEQ[AUDIOBAND20+AVG]);
			}
			break;

		case EXTREQ_AUDIO_OCT3_CUR :
		case SENSOR_AUDIO_OCT3_CUR :
			if((HardwareConfig&CONFIG_AUDIOPRESSURE)!=DISABLED) {
				strcpy_P(buffer,(char*)pgm_read_word(&(JStr[J_S38]))); // "%>S3800%H%H%H&%<"
				if(protocol==PR_JOSE) Comms::txData(port, buffer, LaEQ[AUDIOBAND25+CUR], LaEQ[AUDIOBAND24+CUR], LaEQ[AUDIOBAND23+CUR]);
			}
			break;
		case SENSOR_AUDIO_OCT3_MAX :
			if((HardwareConfig&CONFIG_AUDIOPRESSURE)!=DISABLED) {
				strcpy_P(buffer,(char*)pgm_read_word(&(JStr[J_T38]))); // "%>T3800%H%H%H&%<"
				if(protocol==PR_JOSE) Comms::txData(port, buffer, LaEQ[AUDIOBAND25+MAX], LaEQ[AUDIOBAND24+MAX], LaEQ[AUDIOBAND23+MAX]);
			}
			break;
		case SENSOR_AUDIO_OCT3_MIN :
			if((HardwareConfig&CONFIG_AUDIOPRESSURE)!=DISABLED) {
				strcpy_P(buffer,(char*)pgm_read_word(&(JStr[J_U38]))); // "%>U3800%H%H%H&%<"
				if(protocol==PR_JOSE) Comms::txData(port, buffer, LaEQ[AUDIOBAND25+MIN], LaEQ[AUDIOBAND24+MIN], LaEQ[AUDIOBAND23+MIN]);
			}
			break;
		case SENSOR_AUDIO_OCT3_AVG :
			if((HardwareConfig&CONFIG_AUDIOPRESSURE)!=DISABLED) {
				strcpy_P(buffer,(char*)pgm_read_word(&(JStr[J_V38]))); // "%>V3800%H%H%H&%<"
				if(protocol==PR_JOSE) Comms::txData(port, buffer, LaEQ[AUDIOBAND25+AVG], LaEQ[AUDIOBAND24+AVG], LaEQ[AUDIOBAND23+AVG]);
			}
			break;

		case EXTREQ_AUDIO_OCT4_CUR :
		case SENSOR_AUDIO_OCT4_CUR :
			if((HardwareConfig&CONFIG_AUDIOPRESSURE)!=DISABLED) {
				strcpy_P(buffer,(char*)pgm_read_word(&(JStr[J_S39]))); // "%>S3900%H%H%H&%<"
				if(protocol==PR_JOSE) Comms::txData(port, buffer, LaEQ[AUDIOBAND28+CUR], LaEQ[AUDIOBAND27+CUR], LaEQ[AUDIOBAND26+CUR]);
			}
			break;
		case SENSOR_AUDIO_OCT4_MAX :
			if((HardwareConfig&CONFIG_AUDIOPRESSURE)!=DISABLED) {
				strcpy_P(buffer,(char*)pgm_read_word(&(JStr[J_T39]))); // "%>T3900%H%H%H&%<"
				if(protocol==PR_JOSE) Comms::txData(port, buffer, LaEQ[AUDIOBAND28+MAX], LaEQ[AUDIOBAND27+MAX], LaEQ[AUDIOBAND26+MAX]);
			}
			break;
		case SENSOR_AUDIO_OCT4_MIN :
			if((HardwareConfig&CONFIG_AUDIOPRESSURE)!=DISABLED) {
				strcpy_P(buffer,(char*)pgm_read_word(&(JStr[J_U39]))); // "%>U3900%H%H%H&%<"
				if(protocol==PR_JOSE) Comms::txData(port, buffer, LaEQ[AUDIOBAND28+MIN], LaEQ[AUDIOBAND27+MIN], LaEQ[AUDIOBAND26+MIN]);
			}
			break;
		case SENSOR_AUDIO_OCT4_AVG :
			if((HardwareConfig&CONFIG_AUDIOPRESSURE)!=DISABLED) {
				strcpy_P(buffer,(char*)pgm_read_word(&(JStr[J_V39]))); // "%>V3900%H%H%H&%<"
				if(protocol==PR_JOSE) Comms::txData(port, buffer, LaEQ[AUDIOBAND28+AVG], LaEQ[AUDIOBAND27+AVG], LaEQ[AUDIOBAND26+AVG]);
			}
			break;

		case EXTREQ_AUDIO_OCT5_CUR :
		case SENSOR_AUDIO_OCT5_CUR :
			if((HardwareConfig&CONFIG_AUDIOPRESSURE)!=DISABLED) {
				strcpy_P(buffer,(char*)pgm_read_word(&(JStr[J_S3A]))); // "%>S3A00%H%H%H&%<"
				if(protocol==PR_JOSE) Comms::txData(port, buffer, LaEQ[AUDIOBAND31+CUR], LaEQ[AUDIOBAND30+CUR], LaEQ[AUDIOBAND29+CUR]);
			}
			break;
		case SENSOR_AUDIO_OCT5_MAX :
			if((HardwareConfig&CONFIG_AUDIOPRESSURE)!=DISABLED) {
				strcpy_P(buffer,(char*)pgm_read_word(&(JStr[J_T3A]))); // "%>T3A00%H%H%H&%<"
				if(protocol==PR_JOSE) Comms::txData(port, buffer, LaEQ[AUDIOBAND31+MAX], LaEQ[AUDIOBAND30+MAX], LaEQ[AUDIOBAND29+MAX]);
			}
			break;
		case SENSOR_AUDIO_OCT5_MIN :
			if((HardwareConfig&CONFIG_AUDIOPRESSURE)!=DISABLED) {
				strcpy_P(buffer,(char*)pgm_read_word(&(JStr[J_U3A]))); // "%>U3A00%H%H%H&%<"
				if(protocol==PR_JOSE) Comms::txData(port, buffer, LaEQ[AUDIOBAND31+MIN], LaEQ[AUDIOBAND30+MIN], LaEQ[AUDIOBAND29+MIN]);
			}
			break;
		case SENSOR_AUDIO_OCT5_AVG :
			if((HardwareConfig&CONFIG_AUDIOPRESSURE)!=DISABLED) {
				strcpy_P(buffer,(char*)pgm_read_word(&(JStr[J_V3A]))); // "%>V3A00%H%H%H&%<"
				if(protocol==PR_JOSE) Comms::txData(port, buffer, LaEQ[AUDIOBAND31+AVG], LaEQ[AUDIOBAND30+AVG], LaEQ[AUDIOBAND29+AVG]);
			}
			break;

		case EXTREQ_AUDIO_OCT6_CUR :
		case SENSOR_AUDIO_OCT6_CUR :
			if((HardwareConfig&CONFIG_AUDIOPRESSURE)!=DISABLED) {
				strcpy_P(buffer,(char*)pgm_read_word(&(JStr[J_S3B]))); // "%>S3B00%H%H%H&%<"
				if(protocol==PR_JOSE) Comms::txData(port, buffer, LaEQ[AUDIOBAND34+CUR], LaEQ[AUDIOBAND33+CUR], LaEQ[AUDIOBAND32+CUR]);
			}
			break;
		case SENSOR_AUDIO_OCT6_MAX :
			if((HardwareConfig&CONFIG_AUDIOPRESSURE)!=DISABLED) {
				strcpy_P(buffer,(char*)pgm_read_word(&(JStr[J_T3B]))); // "%>T3B00%H%H%H&%<"
				if(protocol==PR_JOSE) Comms::txData(port, buffer, LaEQ[AUDIOBAND34+MAX], LaEQ[AUDIOBAND33+MAX], LaEQ[AUDIOBAND32+MAX]);
			}
			break;
		case SENSOR_AUDIO_OCT6_MIN :
			if((HardwareConfig&CONFIG_AUDIOPRESSURE)!=DISABLED) {
				strcpy_P(buffer,(char*)pgm_read_word(&(JStr[J_U3B]))); // "%>U3B00%H%H%H&%<"
				if(protocol==PR_JOSE) Comms::txData(port, buffer, LaEQ[AUDIOBAND34+MIN], LaEQ[AUDIOBAND33+MIN], LaEQ[AUDIOBAND32+MIN]);
			}
			break;
		case SENSOR_AUDIO_OCT6_AVG :
			if((HardwareConfig&CONFIG_AUDIOPRESSURE)!=DISABLED) {
				strcpy_P(buffer,(char*)pgm_read_word(&(JStr[J_V3B]))); // "%>V3B00%H%H%H&%<"
				if(protocol==PR_JOSE) Comms::txData(port, buffer, LaEQ[AUDIOBAND34+AVG], LaEQ[AUDIOBAND33+AVG], LaEQ[AUDIOBAND32+AVG]);
			}
			break;

		case EXTREQ_AUDIO_OCT7_CUR :
		case SENSOR_AUDIO_OCT7_CUR :
			if((HardwareConfig&CONFIG_AUDIOPRESSURE)!=DISABLED) {
				strcpy_P(buffer,(char*)pgm_read_word(&(JStr[J_S3C]))); // "%>S3C00%H%H%H&%<"
				if(protocol==PR_JOSE) Comms::txData(port, buffer, LaEQ[AUDIOBAND37+CUR], LaEQ[AUDIOBAND36+CUR], LaEQ[AUDIOBAND35+CUR]);
			}
			break;
		case SENSOR_AUDIO_OCT7_MAX :
			if((HardwareConfig&CONFIG_AUDIOPRESSURE)!=DISABLED) {
				strcpy_P(buffer,(char*)pgm_read_word(&(JStr[J_T3C]))); // "%>T3C00%H%H%H&%<"
				if(protocol==PR_JOSE) Comms::txData(port, buffer, LaEQ[AUDIOBAND37+MAX], LaEQ[AUDIOBAND36+MAX], LaEQ[AUDIOBAND35+MAX]);
			}
			break;
		case SENSOR_AUDIO_OCT7_MIN :
			if((HardwareConfig&CONFIG_AUDIOPRESSURE)!=DISABLED) {
				strcpy_P(buffer,(char*)pgm_read_word(&(JStr[J_U3C]))); // "%>U3C00%H%H%H&%<"
				if(protocol==PR_JOSE) Comms::txData(port, buffer, LaEQ[AUDIOBAND37+MIN], LaEQ[AUDIOBAND36+MIN], LaEQ[AUDIOBAND35+MIN]);
			}
			break;
		case SENSOR_AUDIO_OCT7_AVG :
			if((HardwareConfig&CONFIG_AUDIOPRESSURE)!=DISABLED) {
				strcpy_P(buffer,(char*)pgm_read_word(&(JStr[J_V3C]))); // "%>V3C00%H%H%H&%<"
				if(protocol==PR_JOSE) Comms::txData(port, buffer, LaEQ[AUDIOBAND37+AVG], LaEQ[AUDIOBAND36+AVG], LaEQ[AUDIOBAND35+AVG]);
			}
			break;

		case EXTREQ_AUDIO_OCT8_CUR :
		case SENSOR_AUDIO_OCT8_CUR :
			if((HardwareConfig&CONFIG_AUDIOPRESSURE)!=DISABLED) {
				strcpy_P(buffer,(char*)pgm_read_word(&(JStr[J_S3D]))); // "%>S3D00%H%H%H&%<"
				if(protocol==PR_JOSE) Comms::txData(port, buffer, LaEQ[AUDIOBAND40+CUR], LaEQ[AUDIOBAND39+CUR], LaEQ[AUDIOBAND38+CUR]);
			}
			break;
		case SENSOR_AUDIO_OCT8_MAX :
			if((HardwareConfig&CONFIG_AUDIOPRESSURE)!=DISABLED) {
				strcpy_P(buffer,(char*)pgm_read_word(&(JStr[J_T3D]))); // "%>T3D00%H%H%H&%<"
				if(protocol==PR_JOSE) Comms::txData(port, buffer, LaEQ[AUDIOBAND40+MAX], LaEQ[AUDIOBAND39+MAX], LaEQ[AUDIOBAND38+MAX]);
			}
			break;
		case SENSOR_AUDIO_OCT8_MIN :
			if((HardwareConfig&CONFIG_AUDIOPRESSURE)!=DISABLED) {
				strcpy_P(buffer,(char*)pgm_read_word(&(JStr[J_U3D]))); // "%>U3D00%H%H%H&%<"
				if(protocol==PR_JOSE) Comms::txData(port, buffer, LaEQ[AUDIOBAND40+MIN], LaEQ[AUDIOBAND39+MIN], LaEQ[AUDIOBAND38+MIN]);
			}
			break;
		case SENSOR_AUDIO_OCT8_AVG :
			if((HardwareConfig&CONFIG_AUDIOPRESSURE)!=DISABLED) {
				strcpy_P(buffer,(char*)pgm_read_word(&(JStr[J_V3D]))); // "%>V3D00%H%H%H&%<"
				if(protocol==PR_JOSE) Comms::txData(port, buffer, LaEQ[AUDIOBAND40+AVG], LaEQ[AUDIOBAND39+AVG], LaEQ[AUDIOBAND38+AVG]);
			}
			break;

		case EXTREQ_AUDIO_OCT9_CUR :
		case SENSOR_AUDIO_OCT9_CUR :
			if((HardwareConfig&CONFIG_AUDIOPRESSURE)!=DISABLED) {
				strcpy_P(buffer,(char*)pgm_read_word(&(JStr[J_S3E]))); // "%>S3E00%H%H%H&%<"
				if(protocol==PR_JOSE) Comms::txData(port, buffer, LaEQ[AUDIOBAND43+CUR], LaEQ[AUDIOBAND42+CUR], LaEQ[AUDIOBAND41+CUR]);
			}
			break;
		case SENSOR_AUDIO_OCT9_MAX :
			if((HardwareConfig&CONFIG_AUDIOPRESSURE)!=DISABLED) {
				strcpy_P(buffer,(char*)pgm_read_word(&(JStr[J_T3E]))); // "%>T3E00%H%H%H&%<"
				if(protocol==PR_JOSE) Comms::txData(port, buffer, LaEQ[AUDIOBAND43+MAX], LaEQ[AUDIOBAND42+MAX], LaEQ[AUDIOBAND41+MAX]);
			}
			break;
		case SENSOR_AUDIO_OCT9_MIN :
			if((HardwareConfig&CONFIG_AUDIOPRESSURE)!=DISABLED) {
				strcpy_P(buffer,(char*)pgm_read_word(&(JStr[J_U3E]))); // "%>U3E00%H%H%H&%<"
				if(protocol==PR_JOSE) Comms::txData(port, buffer, LaEQ[AUDIOBAND43+MIN], LaEQ[AUDIOBAND42+MIN], LaEQ[AUDIOBAND41+MIN]);
			}
			break;
		case SENSOR_AUDIO_OCT9_AVG :
			if((HardwareConfig&CONFIG_AUDIOPRESSURE)!=DISABLED) {
				strcpy_P(buffer,(char*)pgm_read_word(&(JStr[J_V3E]))); // "%>V3E00%H%H%H&%<"
				if(protocol==PR_JOSE) Comms::txData(port, buffer, LaEQ[AUDIOBAND43+AVG], LaEQ[AUDIOBAND42+AVG], LaEQ[AUDIOBAND41+AVG]);
			}
			break;

		case EXTREQ_LATITUDE :
		case SENSOR_LATITUDE :
			if(protocol==PR_VERBOSE && Latitude!=0xFFFFFFFF) Comms::txData(port,"Latitude: 0x%h\n",Latitude);
			if(protocol==PR_JOSE && Latitude!=0xFFFFFFFF) Comms::txData(port,"%>S41%h&%<", (uint32_t)(Latitude));
			break;
		case EXTREQ_LONGITUDE :
		case SENSOR_LONGITUDE :
			if(protocol==PR_VERBOSE && Longitude!=0xFFFFFFFF) Comms::txData(port,"Longitude: 0x%h\n",Longitude);
			if(protocol==PR_JOSE && Longitude!=0xFFFFFFFF) Comms::txData(port,"%>S42%h&%<", (uint32_t)(Longitude));
			break;
	
		default:
			break;
	}
}

void Comms::txMemory(uint8_t port, uint32_t data, uint8_t length, boolean rw) {
//	if(port==IF_WiFi){
	if(rw==RD) {
		Comms::txData(port,"%>Z0%d%h&%<", length, data);
	} else {
		Comms::txData(port,"%>Z8%d%h&%<", length, data);
	}
//	}
}


/************************************************************************************
*************************************************************************************
* Sensor class																		*
*************************************************************************************
************************************************************************************/
Sensor::Sensor(void) {

}

void Sensor::begin() {
	TMP112.begin(0x49,false);
    BME280.begin(0x76,false);
    BH1750.begin(0x5C,false);
	ADXL345.begin(0x1D,false);
	T6713.begin(0x4E,0);
	if(Utils::readEEPROM(MEM_CO2BOOTMODE, 1)==ENABLED) {
		Utils::powerCO2(ENABLED);
	} else {
		Utils::powerCO2(DISABLED);
	}
	
	JosenePM.begin(0x4E,1);
	M41T00S.begin(0x68,false);

	if(PMMode[MEASMODE]==ENABLED) { 
		Utils::powerPM(ENABLED); 
	} else if(PMMode[MEASMODE]==CYCLIC) { 
		Utils::powerPM(CYCLIC);
	} else {
		Utils::powerPM(DISABLED);
	}
	if((Utils::readEEPROM(MEM_AMPFACTOR,2)!=0xFFFF)) {		// Check if AmpFactor is programmed, if yes, init soundmodule
		PCB150011.begin(0x10,false);
		PCB150011.setAmpFactor(Utils::readEEPROM(MEM_AMPFACTOR,2));
		if(((Utils::readEEPROM(MEM_DEBUGLEDS,1)&0x02)==0x00)) PCB150011.disableLED(); else PCB150011.enableLED();
		PCB150011.clearThirdsSpectrum(LaEQ);
	}
	DS2482S.begin(0x18,false);
}

/**********************************
* Retrieve sensordata from TMP112 *
**********************************/
void Sensor::getUnitTemperature() {
	Utils::ctrlDebugLEDs(LEDI2C, ON);
	UnitTemperature = TMP112.getSensorData();
	Utils::ctrlDebugLEDs(LEDI2C, OFF);
}

/******************************************
* Retrieve sensordata from DS2482/DB18B20 *
******************************************/
void Sensor::getExtTemperature() {
	Utils::ctrlDebugLEDs(LEDI2C, ON);
	ExtTemperature = DS2482S.getSensorData();
	Utils::ctrlDebugLEDs(LEDI2C, OFF);
}

/******************************************
* Request sensordata from DS2482/DB18B20 *
******************************************/
void Sensor::askExtTemperature() {
	Utils::ctrlDebugLEDs(LEDI2C, ON);
	if((HardwareConfig&CONFIG_EXTTEMPSENSOR)!=DISABLED) DS2482S.requestDS18B20();
	Utils::ctrlDebugLEDs(LEDI2C, OFF);
}
/**********************************
* Retrieve sensordata from BME280 *
**********************************/
void Sensor::getBME280Values() {
	float *p;
	Utils::ctrlDebugLEDs(LEDI2C, ON);
	p=BME280.getSensorData();
	Temperature=*(p);
	Pressure=*(p+1);
	Humidity=*(p+2);
	Utils::ctrlDebugLEDs(LEDI2C, OFF);	
}

/***********************************
* Retrieve sensordata from ADXL345 *
***********************************/
void Sensor::getADXL345Values() {
	float *p;
	Utils::ctrlDebugLEDs(LEDI2C, ON);
	p=ADXL345.getSensorData();
	AcceleroX=*(p);
	AcceleroY=*(p+1);
	DeviceAngle=*(p+2);
	AcceleroI=*(p+3);
	Utils::ctrlDebugLEDs(LEDI2C, OFF);	
}

/***********************************
* Retrieve sensordata from M41T00S *
***********************************/
void Sensor::getRTC() {
	uint8_t *p;
	Utils::ctrlDebugLEDs(LEDI2C, ON);
	p=M41T00S.getData();
	for (uint8_t x=0; x<=6; x++){
		RTC[x]=*(p+x);
	}
	Utils::ctrlDebugLEDs(LEDI2C, OFF);	
}

/**********************************
* Retrieve sensordata from BH1750 *
**********************************/
void Sensor::getLightIntensity() {
	Utils::ctrlDebugLEDs(LEDI2C, ON);
	LightIntensity = BH1750.getSensorData();
	Utils::ctrlDebugLEDs(LEDI2C, OFF);
}

void Sensor::askCO2() {
	Utils::ctrlDebugLEDs(LEDAsy, ON);
	T6713.askSensor();
	Utils::ctrlDebugLEDs(LEDAsy, OFF);
}

void Sensor::getCO2() {
	Utils::ctrlDebugLEDs(LEDAsy, ON);
	CO2gas = T6713.getData();
	Utils::ctrlDebugLEDs(LEDAsy, OFF);
}

void Sensor::getPM() {
	if((PMMode[MEASMODE]==CYCLIC)&&(PMMode[POWERMODE]!=POWEREDUP)) return;
	uint16_t *p;
	Utils::ctrlDebugLEDs(LEDAsy, ON);
	p=JosenePM.getData();
	PM10=*(p);
	PM2_5=*(p+1);
	Utils::ctrlDebugLEDs(LEDAsy, OFF);
}

void Sensor::getAudioSpectrum() {
	Utils::ctrlDebugLEDs(LEDI2C, ON);
	if((HardwareConfig&CONFIG_AUDIOPRESSURE)!=DISABLED) PCB150011.getThirdsSpectrum(LaEQ);
	Utils::ctrlDebugLEDs(LEDI2C, OFF);
}

void Sensor::askCO2Status() {
	Utils::ctrlDebugLEDs(LEDAsy, ON);
	T6713.askStatus();
	Utils::ctrlDebugLEDs(LEDAsy, OFF);
}

void Sensor::getCO2Status() {
	Utils::ctrlDebugLEDs(LEDAsy, ON);
	CO2Status = T6713.getData();
	Utils::ctrlDebugLEDs(LEDAsy, OFF);
}

void Sensor::askCO2Calibration() {
	Utils::ctrlDebugLEDs(LEDAsy, ON);
	T6713.askCalibration();
	Utils::ctrlDebugLEDs(LEDAsy, OFF);
}

void Sensor::getLocation() {
	Latitude = Utils::readEEPROM(MEM_LAT,4);
	Longitude = Utils::readEEPROM(MEM_LON,4);
}
/************************************************************************************
*************************************************************************************
* LEDRing class																		*
*************************************************************************************
************************************************************************************/
LEDRing::LEDRing(void) {
	
}

/***********
* Show CO2 *
***********/
void LEDRing::showCO2(uint8_t mode) {
	// DemoLED == Holds recalculated LED number
	// DemoBackup == Backup value
	// DemoActual == Running towards PM10 value
	uint8_t Br = 0x08;
	if((PowerState&0x80)!=0) Br=0x0C;			// If on mains, high power is available
	
	if(DemoActual<352) DemoActual=352;
	if(DemoActual<CO2gas) DemoActual++;
	if(DemoActual>CO2gas) DemoActual--;

	if((DemoActual==DemoBackup) && (DemoCursorActive==OFF)) return;
	DemoCursorActive=OFF;
	
	DemoBackup = DemoActual;
	if (DemoActual<=352) {											//Blue
		LEDRing::rangeLED(0x00,0x20,Br,0x00,0x00,0xFF,UPDATE);
	}
	if (DemoActual > 352 && DemoActual <= 448) {						//Blue to green
		DemoLED = (DemoActual - 352) / 6;
		LEDRing::rangeLED(0x00,0x20,Br,0x00,0x00,0xFF,MEMORY);
		LEDRing::rangeLED(((DemoLED*-1)+16),DemoLED*2,Br,0x00,0xFF,0x00,UPDATE);
	}
	if (DemoActual > 448 && DemoActual <= 544) {						//Green to yellow
		DemoLED = (DemoActual - 448) / 6;
		LEDRing::rangeLED(0x00,0x20,Br,0x00,0xFF,0x00,MEMORY);
		LEDRing::rangeLED(((DemoLED*-1)+16),DemoLED*2,Br,0xFF,0xFF,0x00,UPDATE);
	}
	if (DemoActual > 544 && DemoActual <= 640) {						//Yellow to red
		DemoLED = (DemoActual - 544) / 6;
		LEDRing::rangeLED(0x00,0x20,Br,0xFF,0xFF,0x00,MEMORY);
		LEDRing::rangeLED(((DemoLED*-1)+16),DemoLED*2,Br,0xFF,0x00,0x00,UPDATE);
	}
	if (DemoActual > 640 && DemoActual <= 736) {						//Red to pink
		DemoLED = (DemoActual - 640) / 6;
		LEDRing::rangeLED(0x00,0x20,Br,0xFF,0x00,0x00,MEMORY);
		LEDRing::rangeLED(((DemoLED*-1)+16),DemoLED*2,Br,0xFF,0x00,0x6F,UPDATE);
	}
	if (DemoActual > 736) {											//Pink
		LEDRing::rangeLED(0x00,0x20,Br,0xFF,0x00,0x6F,UPDATE);
	}
}

/*******************
* Show Temperature *
*******************/
void LEDRing::showTemperature(uint8_t mode) {
	// DemoLED == Holds recalculated LED number
	// DemoBackup == Backup value
	// DemoActual == Running towards PM10 value
	// DemoCursorActive != Used
	uint8_t Br = 0x08;
	if((PowerState&0x80)!=0) Br=0x0C;			// If on mains, high power is available
	
	if(DemoActual<(uint16_t)Temperature) DemoActual++;
	if(DemoActual>(uint16_t)Temperature) DemoActual--;

	if((DemoActual==DemoBackup) && (DemoCursorActive==OFF)) return;
	DemoCursorActive=OFF;
		
	DemoBackup = DemoActual;
	if (DemoActual <= 16) {											//Blue
		LEDRing::rangeLED(0x00,0x20,Br,0x00,0x00,0x7F,UPDATE);
	}
	if (DemoActual > 16 && DemoActual <= 32) {						//Blue to cyan
		DemoLED = DemoActual-16;
		LEDRing::rangeLED(0x00,0x20,Br,0x00,0x00,0x7F,MEMORY);
		LEDRing::rangeLED(((DemoLED*-1)+16),DemoLED*2,Br,0xFF,0x3F,0x00,UPDATE);
	}
	if (DemoActual > 32) {											//Cyan
		LEDRing::rangeLED(0x00,0x20,Br,0x00,0xFF,0x3F,UPDATE);
	}
}

/****************
* Show Humidity	*
****************/
void LEDRing::showHumidity(uint8_t mode) {
	// DemoLED == Holds recalculated LED number
	// DemoBackup == Backup value
	// DemoActual == Running towards PM10 value
	// DemoCursorActive != Used
	uint8_t Br = 0x08;
	if((PowerState&0x80)!=0) Br=0x0C;			// If on mains, high power is available
	
	if(DemoActual<(uint16_t)Humidity) DemoActual++;
	if(DemoActual>(uint16_t)Humidity) DemoActual--;

	if((DemoActual==DemoBackup) && (DemoCursorActive==OFF)) return;
	DemoCursorActive=OFF;
		
	DemoBackup = DemoActual;
	if (DemoActual <= 16) {											//Blue
		LEDRing::rangeLED(0x00,0x20,Br,0x00,0x00,0x7F,UPDATE);
	}
	if (DemoActual > 16 && DemoActual <= 80) {						//Blue to cyan
		DemoLED = (DemoActual-16) / 4;
		LEDRing::rangeLED(0x00,0x20,Br,0x00,0x00,0x7F,MEMORY);
		LEDRing::rangeLED(((DemoLED*-1)+16),DemoLED*2,Br,0x00,0xAF,0xFF,UPDATE);
	}
	if (DemoActual > 80) {											//Cyan
		LEDRing::rangeLED(0x00,0x20,Br,0x00,0xAF,0xFF,UPDATE);
	}
}

/****************
* Show Pressure	*
****************/
void LEDRing::showPressure(uint8_t mode) {
	// DemoLED == Holds recalculated LED number
	// DemoBackup == Backup value
	// DemoActual == Running towards PM10 value
	// DemoCursorActive != Used
	uint8_t Br = 0x08;
	if((PowerState&0x80)!=0) Br=0x0C;			// If on mains, high power is available
	
	if(DemoActual<976) DemoActual=976;
	if(DemoActual<(uint16_t)Pressure) DemoActual++;
	if(DemoActual>(uint16_t)Pressure) DemoActual--;

	if((DemoActual==DemoBackup) && (DemoCursorActive==OFF)) return;
	DemoCursorActive=OFF;

	DemoBackup = DemoActual;
	if (DemoActual <= 976) {										//Dark Blue
		LEDRing::rangeLED(0x00,0x20,Br,0x00,0x00,0x7F,UPDATE);
	}
	if (DemoActual > 976 && DemoActual <= 984) {					//Dark Blue to blue
		DemoLED = (DemoActual - 976)*2;
		LEDRing::rangeLED(0x00,0x20,Br,0x00,0x00,0x7F,MEMORY);
		LEDRing::rangeLED(((DemoLED*-1)+16),DemoLED*2,Br,0x00,0x00,0xFF,UPDATE);
	}
	if (DemoActual > 984 && DemoActual <= 992) {					//Blue to Cyan
		DemoLED = (DemoActual - 984)*2;
		LEDRing::rangeLED(0x00,0x20,Br,0x00,0x00,0xFF,MEMORY);
		LEDRing::rangeLED(((DemoLED*-1)+16),DemoLED*2,Br,0x00,0x7F,0xFF,UPDATE);
	}
	if (DemoActual > 992 && DemoActual <= 1000) {					//Blue to green
		DemoLED = (DemoActual - 992)*2;
		LEDRing::rangeLED(0x00,0x20,Br,0x00,0x7F,0xFF,MEMORY);
		LEDRing::rangeLED(((DemoLED*-1)+16),DemoLED*2,Br,0x00,0xFF,0x00,UPDATE);
	}
	if (DemoActual > 1000 && DemoActual <= 1008) {				//Green to green yellow
		DemoLED = (DemoActual - 1000)*2;
		LEDRing::rangeLED(0x00,0x20,Br,0x00,0xFF,0x00,MEMORY);
		LEDRing::rangeLED(((DemoLED*-1)+16),DemoLED*2,Br,0x7F,0xFF,0x00,UPDATE);
	}
	if (DemoActual > 1008 && DemoActual <= 1016) {				//Green yellow to yellow
		DemoLED = (DemoActual - 1008)*2;
		LEDRing::rangeLED(0x00,0x20,Br,0x7F,0xFF,0x00,MEMORY);
		LEDRing::rangeLED(((DemoLED*-1)+16),DemoLED*2,Br,0xFF,0xFF,0x00,UPDATE);
	}
	if (DemoActual > 1016 && DemoActual <= 1024) {				//Yellow to orange
		DemoLED = (DemoActual - 1016)*2;
		LEDRing::rangeLED(0x00,0x20,Br,0xFF,0xFF,0x00,MEMORY);
		LEDRing::rangeLED(((DemoLED*-1)+16),DemoLED*2,Br,0xFF,0x3F,0x00,UPDATE);
	}
	if (DemoActual > 1024 && DemoActual <= 1032) {				//Orange to red   
		DemoLED = (DemoActual - 1024)*2;
		LEDRing::rangeLED(0x00,0x20,Br,0xFF,0x3F,0x00,MEMORY);
		LEDRing::rangeLED(((DemoLED*-1)+16),DemoLED*2,Br,0xFF,0x00,0x00,UPDATE);
	}
	if (DemoActual > 1032 && DemoActual <= 1040) {				//Red to pink   
		DemoLED = (DemoActual - 1032)*2;
		LEDRing::rangeLED(0x00,0x20,Br,0xFF,0x00,0x00,MEMORY);
		LEDRing::rangeLED(((DemoLED*-1)+16),DemoLED*2,Br,0xFF,0x00,0x6F,UPDATE);
	}
	if (DemoActual > 1040) {										//Pink
		LEDRing::rangeLED(0x00,0x20,Br,0xFF,0x00,0x6F,UPDATE);
	}
}

/************
* Show PM10	*
************/
void LEDRing::showPM10(uint8_t mode) {
	// DemoLED == Holds recalculated LED number
	// DemoBackup == Backup value
	// DemoActual == Running towards PM10 value
	// DemoCursorActive != Used
	uint8_t Br = 0x08;
	if((PowerState&0x80)!=0) Br=0x0C;			// If on mains, high power is available
	
	if(DemoActual<PM10) DemoActual++;
	if(DemoActual>PM10) DemoActual--;

	if((DemoActual==DemoBackup) && (DemoCursorActive==OFF)) return;
	DemoCursorActive=OFF;

	DemoBackup = DemoActual;
	if (DemoActual==0) {											//Blue
		LEDRing::rangeLED(0x00,0x20,Br,0x00,0x00,0xFF,UPDATE);
	}
	if (DemoActual > 0 && DemoActual <= 16) {						//Blue to green
		DemoLED = DemoActual;
		LEDRing::rangeLED(0x00,0x20,Br,0x00,0x00,0xFF,MEMORY);
		LEDRing::rangeLED(((DemoLED*-1)+16),DemoLED*2,Br,0x00,0xFF,0x00,UPDATE);
	}
	if (DemoActual > 16 && DemoActual <= 48) {					//Green to yellow
		DemoLED = (DemoActual - 16) / 2;
		LEDRing::rangeLED(0x00,0x20,Br,0x00,0xFF,0x00,MEMORY);
		LEDRing::rangeLED(((DemoLED*-1)+16),DemoLED*2,Br,0xFF,0xFF,0x00,UPDATE);
	}
	if (DemoActual > 48 && DemoActual <= 128) {					//Yellow to red
		DemoLED = (DemoActual - 48) / 5;
		LEDRing::rangeLED(0x00,0x20,Br,0xFF,0xFF,0x00,MEMORY);
		LEDRing::rangeLED(((DemoLED*-1)+16),DemoLED*2,Br,0xFF,0x00,0x00,UPDATE);
	}
	if (DemoActual > 128 && DemoActual <= 176) {					//Red to pink
		DemoLED = (DemoActual - 128) / 3;
		LEDRing::rangeLED(0x00,0x20,Br,0xFF,0x00,0x00,MEMORY);
		LEDRing::rangeLED(((DemoLED*-1)+16),DemoLED*2,Br,0xFF,0x00,0x6F,UPDATE);
	}
	if (DemoActual > 176) {										//Pink
		LEDRing::rangeLED(0x00,0x20,Br,0xFF,0x00,0x6F,UPDATE);
	}
}

/*************
* Show PM2.5 *
*************/
void LEDRing::showPM2_5(uint8_t mode) {
	// DemoLED == Holds recalculated LED number
	// DemoBackup == Backup value
	// DemoActual == Running towards PM10 value
	// DemoCursorActive != Used
	uint8_t Br = 0x08;
	if((PowerState&0x80)!=0) Br=0x0C;			// If on mains, high power is available
	
	if(DemoActual<PM2_5) DemoActual++;
	if(DemoActual>PM2_5) DemoActual--;

	if((DemoActual==DemoBackup) && (DemoCursorActive==OFF)) return;
	DemoCursorActive=OFF;

	DemoBackup = DemoActual;
	if (DemoActual==0) {											//Blue
		LEDRing::rangeLED(0x00,0x20,Br,0x00,0x00,0xFF,UPDATE);
	}
	if (DemoActual > 0 && DemoActual <= 16) {						//Blue to green
		DemoLED = DemoActual;
		LEDRing::rangeLED(0x00,0x20,Br,0x00,0x00,0xFF,MEMORY);
		LEDRing::rangeLED(((DemoLED*-1)+16),DemoLED*2,Br,0x00,0xFF,0x00,UPDATE);
	}
	if (DemoActual > 16 && DemoActual <= 32) {					//Green to yellow
		DemoLED = (DemoActual - 16);
		LEDRing::rangeLED(0x00,0x20,Br,0x00,0xFF,0x00,MEMORY);
		LEDRing::rangeLED(((DemoLED*-1)+16),DemoLED*2,Br,0xFF,0xFF,0x00,UPDATE);
	}
	if (DemoActual > 32 && DemoActual <= 80) {					//Yellow to red
		DemoLED = (DemoActual - 32) / 3;
		LEDRing::rangeLED(0x00,0x20,Br,0xFF,0xFF,0x00,MEMORY);
		LEDRing::rangeLED(((DemoLED*-1)+16),DemoLED*2,Br,0xFF,0x00,0x00,UPDATE);
	}
	if (DemoActual > 80 && DemoActual <= 128) {					//Red to pink
		DemoLED = (DemoActual - 80) / 3;
		LEDRing::rangeLED(0x00,0x20,Br,0xFF,0x00,0x00,MEMORY);
		LEDRing::rangeLED(((DemoLED*-1)+16),DemoLED*2,Br,0xFF,0x00,0x6F,UPDATE);
	}
	if (DemoActual > 128) {										//Pink
		LEDRing::rangeLED(0x00,0x20,Br,0xFF,0x00,0x6F,UPDATE);
	}
}


void LEDRing::cursor(float Angle) {
	uint8_t Br=0x08;
	if((PowerState&0x80)!=0) Br=0x0C;			// If on mains, high power is available
	int16_t LEDno;
	if(Angle==Angle) {
		LEDno=Angle-90;
		if(LEDno<0) LEDno+=360;
		LEDno/=(float)11.25;
	} else {
		LEDno=0xFF;
	}
	if(LEDno==CursorPos) return;
	CursorPos=LEDno;
//	Serial.println(CursorPos);

	if(CursorPos==0xFF) {
		LEDRing::rangeLED(0x00,0x20,0x00,0x00,0x00,0x00,UPDATE);
	} else {
		LEDRing::rangeLED(0x00,0x20,0x00,0x00,0x00,0x00,MEMORY);
		LEDRing::rangeLED(0x03,0x02,Br,0xFF,0x00,0x00,MEMORY);
		LEDRing::rangeLED(0x0B,0x02,Br,0xFF,0x00,0x00,MEMORY);
		LEDRing::rangeLED(0x13,0x02,Br,0xFF,0x00,0x00,MEMORY);
		LEDRing::rangeLED(0x1B,0x02,Br,0xFF,0x00,0x00,MEMORY);
		if(CursorPos==0x1F) {
			LEDRing::rangeLED(0x00,0x01,Br,0xFF,0xFF,0xFF,MEMORY);
			LEDRing::rangeLED(0x1F,0x01,Br,0xFF,0xFF,0xFF,UPDATE);
		} else {
			LEDRing::rangeLED(CursorPos,0x02,Br,0xFF,0xFF,0xFF,UPDATE);
		}
		DemoCursorActive=ON;
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
* External command to update LEDring, can also be done by mode=UPDATE in single LED function *
*********************************************************************************************/
void LEDRing::updateNow() {
	LEDRing::updateRing(RGBLED);	
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

/**************************************
* Private function to update hardware *
**************************************/
void LEDRing::updateRing(byte *arr) {
    for (byte rgbno = 0; rgbno < 6; rgbno++) {
        for (byte ledno = 0; ledno < 16; ledno++) {
            byte cnt = (rgbno*16)+(ledno);
			Utils::ctrlDebugLEDs(LEDI2C, ON);
			RGB[rgbno].set_led_pwm(ledno, arr[cnt]);
			Utils::ctrlDebugLEDs(LEDI2C, OFF);
        }
    }
}

/************************************************************************************
*************************************************************************************
* IconRing class																	*
*************************************************************************************
************************************************************************************/
IconRing::IconRing(void) {
	
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
		Utils::ctrlDebugLEDs(LEDI2C, ON);
		Icons.set_led_pwm(ledno, arr[ledno]);
		Utils::ctrlDebugLEDs(LEDI2C, OFF);
	}
}

/************************************************************************************
*************************************************************************************
* Josuino Utils class																*
*************************************************************************************
************************************************************************************/
Utils::Utils(void) {
	
}

/******************************************************
* Start Utils, can be used for initial startup script *
******************************************************/
void Utils::begin() {
    Serial.begin(115200);			// Setup USB
    while (!Serial) { 
    }
    Serial1.begin(115200);			// Setup Wifi
    while (!Serial1) { 
    }
	Serial2.begin(57600);          	// Setup LoRa
    while (!Serial2) {
    }
	Serial3.begin(115200);          // Setup MiRa
    while (!Serial3) {
    }
    Serial.print(F("Hello Josuino with firmware version "));
	Serial.print(Vermajor);
	Serial.print(F("."));
	Serial.println(Verminor);
	Serial.println(F("Setting things up, one moment"));

	Utils::setupIO();
	Utils::enableDebugLEDs(false);
    Utils::ctrlDebugLEDs(FORWARD, ON);
    Utils::ctrlDebugLEDs(FORWARD, OFF);
	Utils::checkUSB();
	Utils::loadAndApplyParameters();
	Utils::initI2C();
	Utils::enableI2C();
    Utils::checkI2C();
    if(IconRingLEDs==ENABLED) Utils::initLEDcontrollers();
	Sensor::begin();
	
}

void Utils::finishUp() {
	if(IconRingLEDs==ENABLED) LEDRing::startUpRing();
	if(IconRingLEDs==ENABLED) IconRing::startUpRing();
	Utils::setupTimers();
	interrupts();             // enable all interrupts
	Serial.println(F("\nFinished setup\n\nType 'help' + \\n for help"));
}

void Utils::setupTimers() {
	TCCR1A = 0;
	TCCR1B = 0;
	TCNT1 = 0xF3CB;            
	TCCR1B |= (1 << CS12);    // 256 prescaler 
	TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
}

uint8_t Utils::timerIRQ(int Value) {
	if(Value>=0) {
		TimerCount = Value; 
	} else {
		if(TimerCount==TIMERMAX) {
			TimerCount = 0;
			Utils::ctrlDebugLEDs(LEDTmr, ON);
//			Serial.println(TimerCount);			
		} else {
			TimerCount+=1;
			Utils::ctrlDebugLEDs(LEDTmr, OFF);
//			Serial.print(TimerCount);			
		}
	}

/******************
* Sensor requests *
******************/

	if(TimerCount==1) Sensor::askExtTemperature();
	if(TimerCount==5) Sensor::askCO2();
	if(TimerCount==10) Sensor::getUnitTemperature();
	if(TimerCount==15) Sensor::getCO2();
	if(TimerCount==18) Sensor::getRTC();
	if(TimerCount==20) Sensor::getBME280Values();
	if(TimerCount==25) Sensor::askCO2Status();
	if(TimerCount==30) Sensor::getLightIntensity();
	if(TimerCount==32) Sensor::getADXL345Values();
	if(TimerCount==35) Sensor::getCO2Status();
	if(TimerCount==43) Sensor::getRTC();
	if(TimerCount==45) Sensor::getPM();
	if(TimerCount==46) Sensor::getLocation();
	if(TimerCount==47) Sensor::getAudioSpectrum();
	if(TimerCount==49) Sensor::getExtTemperature();
    
/******************
* Second counters *
******************/
	if(OldSec!=RTC[SECONDS]) {
		OldSec=RTC[SECONDS];
		SessionTimer++;
		TotalUpTimer++;
		CalibrationTimer--;
		if(TotalUpTimer>=1800) {
			TotalUpTimer+=Utils::readEEPROM(MEM_UPTIME);
			Utils::writeEEPROM(MEM_UPTIME, TotalUpTimer, 4);
			TotalUpTimer=0;
		}
		if(CalibrationTimer==0) {
			CalibrationTimer=Utils::readEEPROM(MEM_CALTIMER,1)*3600;
			Sensor::askCO2Calibration();
		}
		if(PMMode[MEASMODE]==CYCLIC){
			if(PMTimer==0) {
				while(PMTimer==0) {
					if(PMMode[POWERMODE]==3) PMMode[POWERMODE]=0; else PMMode[POWERMODE]++;
					PMTimer=Utils::readEEPROM(((PMMode[POWERMODE]*2)+0x230),2);
					if(PMTimer>0) PMTimer--;
				}
				Utils::powerPM(CYCLIC, false);
			} else {
				PMTimer--;				
			}
			/*
			Serial.print(F("T:"));
			Serial.print(PMTimer);
			Serial.print(F(" M:"));
			Serial.println(PMMode[POWERMODE]); */
		}
	}

/****************
* Visualisation *
****************/
	#define DCSMAX	100
	int16_t DCS=0;
	if(DemoModes==ENABLED) {
		Sensor::getADXL345Values();
		LEDRing::cursor(DeviceAngle);
		DCS = (float)(AcceleroX * (DCSMAX-2));
		DCS=DCS<0?DCS*-1:DCS*1;	DCS=DCS*-1+DCSMAX;
	
		if(DemoCounter>=DCS) DemoCounter = 0; else DemoCounter+=1;
		if(DCS==DCSMAX && Demo<0x80 && Demo>0) Demo+=0x80;			// If Demo not yet confirmed, confirm it, with device upright

		if(DemoCounter==0){
			if(CursorPos>=5 && CursorPos<=10) {				// Right
				if(Demo>0x80) Demo-=0x80;					// If Demo is confirmed, deconfirm it
				if(Demo<6) Demo+=1; else Demo=0;
			}
			if(CursorPos>=21 && CursorPos<=26) {			// Left
				if(Demo>0x80) Demo-=0x80;					// If Demo is confirmed, deconfirm it
				if(Demo>0) Demo-=1; else Demo=6;
			}
			if(CursorPos>=13 && CursorPos<=18) {			// Down
				Demo=0x0;
			}
		}
	}
	
/***************
* Sensor icons *
***************/
	if(Demo!=DemoVisual) {
		DemoLED=0;DemoBackup=0;DemoActual=0;
		//Serial.print("Demo:0x"); Serial.println(Demo,HEX); 
		IconRing::singleIcon(ICONTemperature,ICONOFF,0xFF,0xAF,0xAF, MEMORY);
		IconRing::singleIcon(ICONHumidity,ICONOFF,0xFF,0xAF,0xAF, MEMORY);
		IconRing::singleIcon(ICONPressure,ICONOFF,0xFF,0xAF,0xAF, MEMORY);
		IconRing::singleIcon(ICONCO2,ICONOFF,0xFF,0xAF,0xAF, MEMORY);
		IconRing::singleIcon(ICONPM10,ICONOFF,0xFF,0xAF,0xAF, MEMORY);
		IconRing::singleIcon(ICONPM2_5,ICONOFF,0xFF,0xAF,0xAF, MEMORY);
		IconRing::singleIcon(ICONPM1,ICONOFF,0xFF,0xAF,0xAF, MEMORY);
		IconRing::singleIcon(ICONLocation,ICONOFF,0x00,0xFF,0x00, MEMORY);

		switch(Demo) {
			case 0x01: IconRing::singleIcon(ICONTemperature,ICONHALF,0xFF,0xAF,0xAF, MEMORY); break;
			case 0x02: IconRing::singleIcon(ICONHumidity,ICONHALF,0xFF,0xAF,0xAF, MEMORY); break;
			case 0x03: IconRing::singleIcon(ICONPressure,ICONHALF,0xFF,0xAF,0xAF, MEMORY); break;
			case 0x04: IconRing::singleIcon(ICONCO2,ICONHALF,0xFF,0xAF,0xAF, MEMORY); break;
			case 0x05: IconRing::singleIcon(ICONPM10,ICONHALF,0xFF,0xAF,0xAF, MEMORY); break;
			case 0x06: IconRing::singleIcon(ICONPM2_5,ICONHALF,0xFF,0xAF,0xAF, MEMORY); break;
			case 0x07: IconRing::singleIcon(ICONPM1,ICONHALF,0xFF,0xAF,0xAF, MEMORY); break;
			case 0x80: LEDRing::rangeLED(0x00,0x20,0x00,0x00,0x00,0xFF,UPDATE); break;
			case 0x81: IconRing::singleIcon(ICONTemperature,ICONFULL,0xFF,0xAF,0xAF, MEMORY); break;
			case 0x82: IconRing::singleIcon(ICONHumidity,ICONFULL,0xFF,0xAF,0xAF, MEMORY); break;
			case 0x83: IconRing::singleIcon(ICONPressure,ICONFULL,0xFF,0xAF,0xAF, MEMORY); break;
			case 0x84: IconRing::singleIcon(ICONCO2,ICONFULL,0xFF,0xAF,0xAF, MEMORY); break;
			case 0x85: IconRing::singleIcon(ICONPM10,ICONFULL,0xFF,0xAF,0xAF, MEMORY); break;
			case 0x86: IconRing::singleIcon(ICONPM2_5,ICONFULL,0xFF,0xAF,0xAF, MEMORY); break;
			case 0x87: IconRing::singleIcon(ICONPM1,ICONFULL,0xFF,0xAF,0xAF, MEMORY); break;
			default:
				break;
		}
		DemoVisual=Demo;
		int8_t b=ICONFULL-6;
		if(b<1)b=1;
		if(Demo>0x80) IconRing::singleIcon(ICONLocation,b,0x00,0xBF,0x00, MEMORY);
		IconRing::updateNow();
	}
	
	ConnectionState=Utils::checkConnections();

/***************
* Actual Demos *
***************/
	if(DCS==DCSMAX) {
		if(Demo==0x81) LEDRing::showTemperature();
		if(Demo==0x82) LEDRing::showHumidity();
		if(Demo==0x83) LEDRing::showPressure();
		if(Demo==0x84) LEDRing::showCO2();
		if(Demo==0x85) LEDRing::showPM10();
		if(Demo==0x86) LEDRing::showPM2_5();
	}
	
/**********************
* Communication icons *
**********************/
	if((PowerState!=PowerStateVisual) || (ConnectionState!=ConnectionStateVisual)) {
		if((PowerState & POWERWIFION) != 0) {
			if((ConnectionState & POWERWIFION) != 0) {
				IconRing::singleIcon(ICONWiFi,ICONFULL);
			} else {
				IconRing::singleIcon(ICONWiFi,ICONHALF);
			}
		} else {
			IconRing::singleIcon(ICONWiFi,ICONOFF);
		}

		if((PowerState & POWERLORAON) != 0) {
			if(ConnectionState & POWERLORAON != 0) {
				IconRing::singleIcon(ICONLoRa,ICONFULL);
			} else {
				IconRing::singleIcon(ICONLoRa,ICONHALF);
			}
		} else {
			IconRing::singleIcon(ICONLoRa,ICONOFF);
		}

		if((PowerState & POWERMIRAON) != 0) {
			IconRing::singleIcon(ICONMiRa,ICONFULL);
		} else {
			IconRing::singleIcon(ICONMiRa,ICONOFF);
		}

		if((PowerState & 0x80)!=0) {
			IconRing::singleIcon(ICONStatus,9,0xFF,0xAF,0xAF,UPDATE);
		} else {
			IconRing::singleIcon(ICONStatus,9,0x00,0xBF,0x00,UPDATE);
		}
		
		PowerStateVisual=PowerState;
		ConnectionStateVisual=ConnectionState;
	}


/*************************
* Periodacally send data *
* Basetimer = 10*50=500  *
*************************/
	if(BaseTimer==0) return TimerCount;											// If Basetimer==0 then disabled
	if(TxDataCounter==0) TxDataCounter=BaseTimer; else TxDataCounter-=1;
	if(TxDataCounter<=PUSH_MAX) Comms::txSensor(IF_WiFi, PR_JOSE, TxDataCounter);
	return TimerCount;
}

/**********************************
* Setups data direction registers *
* "  " == Active HIGH
* " /" == Active LOW
* "_/" == Rising EDGE
* "\_" == Falling EDGE
**********************************/
void Utils::setupIO() {
	/************************************
	* PortA - Development / debugport
	************************************/
	DDRA = 0xFF;
	PORTA = 0x00;

	/************************************
	* PortB
	* P : Dir : Def : Sg : Function
	*************************************
	* 0 : OUT :  0  :    : Not used
	* 1 : OUT :  1  : _/ : Clock
	* 2 : OUT :  1  :    : SPI MOSI
	* 3 : INP :  H  :    : MISO
	* 4 : OUT :  1  :  / : CS Memory bank 1
	* 5 : OUT :  1  :  / : CS Memory bank 2
	* 6 : OUT :  1  :  / : CS Memory bank 3
	* 7 : OUT :  1  :  / : CS Memory bank 4
	************************************/
	DDRB = 0xF7;
	PORTB = 0xF6;	

	/************************************
	* PortC - Development / debugport
	************************************/
	DDRC = 0xFF;
	PORTC = 0x00;

	/************************************
	* PortD
	* P : Dir : Def : Sg : Function
	*************************************
	* 0 : OUT :  H  : \_ : I2C clock
	* 1 : <-> :  H  :    : I2C data
	* 2 : <-- :  U  :    : WiFi Receive data
	* 3 : --> :  U  :    : Wifi Transmit data
	* 4 : INP :  1  :    : WiFi Clear to Send 
	* 5 : OUT :  1  :    : WiFi Request to Send
	* 6 : INP :  0  :    : WiFi Status
	* 7 : INP :  1  :  / : WiFi Link Active
	************************************/
	DDRD = 0x20;
	PORTD = 0x00;
	
	/************************************
	* PortE
	* P : Dir : Def : Sg : Function
	*************************************
	* 0 : <-- :  U  : \_ : USB Receive data
	* 1 : --> :  U  :    : USB Transmit data
	* 2 : INP :  1  :    : USB Clear to send
	* 3 : OUT :  1  :    : USB Request to send
	* 4 : INP :  1  :    : USB Data set ready 
	* 5 : OUT :  1  :    : USB Data terminal ready
	* 6 : INP :  H  :  / : USB Enumeration complete 
	* 7 : OUT :  0  :    : I2C Enable Hardware
	************************************/
	DDRE = 0xA8;
	PORTE = 0x28;

	/************************************
	* PortF
	* P : Dir : Def : Sg : Function
	*************************************
	* 0 : OUT :  1  :  / : CS SPI device 5    
	* 1 : OUT :  1  :  / : CS SPI device 6          
	* 2 : INP :  H  :    : ExtIO 4
	* 3 : INP :  H  :    : ExtIO 5            
	* 4 : INP :  U  :    : Not used (JTAG TCK)
	* 5 : INP :  U  :    : Not used (JTAG TMS)
	* 6 : INP :  U  :    : Not used (JTAG TDO)
	* 7 : INP :  U  :    : Not used (JTAG TDI)
	************************************/
	DDRF = 0x03;
	PORTF = 0x0F;

	/************************************
	* PortG
	* P : Dir : Def : Sg : Function
	*************************************
	* 0 : OUT :  0  :    : Not used 
	* 1 : OUT :  0  :    : Not used 
	* 2 : OUT :  0  :    : Not used 
	* 3 : OUT :  1  :  / : LED Output enable
	* 4 : OUT :  0  :    : Not used 
	* 5 : OUT :  0  :    : Not used 
	************************************/
	DDRG = 0x3F;
	PORTG = 0x08;
	
	/************************************
	* PortH
	* P : Dir : Def : Sg : Function
	*************************************
	* 0 : <-- :  U  : \_ : LoRa Receive data
	* 1 : --> :  U  :    : LoRa Transmit data
	* 2 : INP :  1  :    : LoRa Clear to send
	* 3 : OUT :  1  :    : LoRa Request to send
	* 4 : OUT :  0  :  / : LoRa Reset
	* 5 : OUT :  0  :    : Not used 
	* 6 : OUT :  0  :    : Not used 
	* 7 : OUT :  0  :    : DebugLEDflag 
	************************************/
	DDRH = 0xF8;
	PORTH = 0x08;
	
	/************************************
	* PortJ
	* P : Dir : Def : Function
	*************************************
	* 0 : <-- :  U  : \_ : MiRa Receive data
	* 1 : --> :  U  :    : MiRa Transmit data
	* 2 : OUT :  1  :  / : MiRa Reset
	* 3 : OUT :  0  :    : Not used 
	* 4 : OUT :  0  : _/ : WiFi Wakeup
	* 5 : OUT :  1  :  / : WiFi Defaults
	* 6 : OUT :  0  :  / : WiFi Reset
	* 7 : OUT :  0  :    : Not used 
	************************************/
	DDRJ = 0xFF;
	PORTJ = 0x24;
	
	/************************************
	* PortK - Development / debugport
	************************************/
	DDRK = 0xFF;
	PORTK = 0x00;

	/************************************
	* PortJ
	* P : Dir : Def : Function
	*************************************
	* 0 : OUT :  1  :  / : TMR LED          
	* 1 : OUT :  1  :  / : I2C LED
	* 2 : OUT :  1  :  / : MEM LED
	* 3 : OUT :  1  :  / : ASY LED
	* 4 : OUT :  1  :  / : WiFi LED
	* 5 : OUT :  1  :  / : MiRa LED
	* 6 : OUT :  1  :  / : LoRa LED
	* 7 : OUT :  1  :  / : USB LED
	************************************/
	DDRL = 0xFF;
	PORTL = 0xFF;
}

/****************************
* Inits PCA9635 controllers *
****************************/
void Utils::initLEDcontrollers() {
    PCA9635.reset();                          	// Reset all PCA9635 by reset on ALL CALL address 0x70
    PCA9635.set_sleep(0x0);                   	// Wake oscillators on ALL CALL address 0x70
    delayMicroseconds(500);					  	// Let oscillators stabilize
    PCA9635.set_led_mode(3);                  	// Enable PWM controls for all LEDs on ALL CALL address 0x70

    RGB[0].begin(0x12, false);					// Container for segment 1 of LED ring
    RGB[1].begin(0x13, false);					// Container for segment 2 of LED ring
    RGB[2].begin(0x14, false);					// Container for segment 3 of LED ring
    RGB[3].begin(0x15, false);					// Container for segment 4 of LED ring
    RGB[4].begin(0x16, false);					// Container for segment 5 of LED ring
    RGB[5].begin(0x17, false);					// Container for segment 6 of LED ring
    Icons.begin(0x1A, false);                 	// Container for icons ring on top
}

/***********************
* Inits I2C controller *
***********************/
void Utils::initI2C() {
	Utils::ctrlDebugLEDs(LEDI2C, ON);
	I2c.begin();
    I2c.timeOut(100);
	I2c.setSpeed(1);
	Serial.println(F("I2C hardware initialized"));
	Utils::ctrlDebugLEDs(LEDI2C, OFF);
}

/********************************
* Checks for FTDI #PWREN signal *
********************************/
void Utils::checkUSB() {
    if ((PINE & 0x40) == 0) {
		Serial.println(F("USB enumerated correctly"));
		PowerState &= 0xFFFFFF00;
		PowerState |= 0x0000004F;
	} else {
		PowerState &= 0xFFFFFF00;
		PowerState |= 0x0000008F;
    }
}

/************************************************
* Checks for I2C devices on all busses          *
*************************************************
* Main concept by Wayne Truchsess               *
* Fitted for purpose by Antoine van de Cruyssen * 
************************************************/
void Utils::checkI2C() {
	uint8_t ret = 0;
	uint16_t timeOutDelay = 0;
	uint16_t tempTime = timeOutDelay;
	I2c.timeOut(10);
	uint8_t totalDev = 0;
	uint8_t knownDev = 0;
	uint8_t expDev = 0;
	if(HardwareRev==0xA10) expDev=17;
	if((HardwareRev==0xB10) || (HardwareRev==0xD10)) expDev=18;		// Todo
	if((HardwareRev==0xB20) || (HardwareRev==0xD20)) expDev=18;		// Todo
	if((HardwareRev==0xB30) || (HardwareRev==0xD30)) expDev=10;
	if((HardwareRev==0xB40) || (HardwareRev==0xD40)) expDev=18;		
	if((HardwareRev==0xB50) || (HardwareRev==0xD50)) expDev=18;		
	if((HardwareRev==0xB60) || (HardwareRev==0xD60)) expDev=18;		

	Serial.println(F("Scanning I2C devices, one moment"));
	for(uint8_t s = 0; s <= 0x7F; s++) {
		Utils::ctrlDebugLEDs(LEDI2C, ON);
		if(I2c.start()==0) ret = I2c.sendAddress(SLA_W(s));		
		Utils::ctrlDebugLEDs(LEDI2C, OFF);

		if(ret!=0) {
			if(ret==1) {
				Serial.println(F("There is a problem with the bus, could not complete scan"));
				timeOutDelay = tempTime;
				return;
			}
		} else {
			totalDev++;
			Serial.print(F("Found device at: 0x")); 
			if(s<0x10) Serial.print(0);
			Serial.print(s,HEX);
			Serial.print(F(" - "));

			switch (s) {
				case 0x00 :
					Serial.println(F("I2C General Call"));
					knownDev++;
					break;
				case 0x03 :
					Serial.println(F("PCA9635 General I2C Reset"));
					knownDev++;
					break;
				case 0x10 :
					Serial.println(F("3PCB150011 Soundpressure sensor"));
					knownDev++;
					break;
				case 0x12 :
					Serial.println(F("PCA9635 RGB segment 1/6"));
					knownDev++;
					break;
				case 0x13 :
					Serial.println(F("PCA9635 RGB segment 2/6"));
					knownDev++;
					break;
				case 0x14 :
					Serial.println(F("PCA9635 RGB segment 3/6"));
					knownDev++;
					break;
				case 0x15 :
					Serial.println(F("PCA9635 RGB segment 4/6"));
					knownDev++;
					break;
				case 0x16 :
					Serial.println(F("PCA9635 RGB segment 5/6"));
					knownDev++;
					break;
				case 0x17 :
					Serial.println(F("PCA9635 RGB segment 6/6"));
					knownDev++;
					break;
				case 0x18 :
					Serial.println(F("DS2482S 1wire bridge"));
					knownDev++;
					break;
				case 0x1A :
					Serial.println(F("PCA9635 Icon LEDs"));
					knownDev++;
					break;
				case 0x1D :
					Serial.println(F("ADXL345 Accelerometer"));
					knownDev++;
					break;
				case 0x49 :
					Serial.println(F("TMP112 Temperature sensor"));
					knownDev++;
					break;
				case 0x4E :
					Serial.println(F("SC16IS752 UART bridge"));
					knownDev++;
					break;
				case 0x50 :
					Serial.println(F("AT24C256C 32kx8 memory"));
					knownDev++;
					break;
				case 0x5C :
					Serial.println(F("BH1750FVI Light sensor"));
					knownDev++;
					break;
				case 0x68 :
					Serial.println(F("M41T00S Real time clock"));
					knownDev++;
					break;
				case 0x70 :
					Serial.println(F("PCA9635 All Call address"));
					knownDev++;
					break;
				case 0x76 :
					Serial.println(F("BME280 Temp, RH, pressure sensor"));
					knownDev++;
					break;
				default :
					Serial.println(F("Unknown device"));
					break;
			}				
		}
		I2c.stop();

	}

	if(knownDev==expDev) {
		Serial.print(F("I2C selftest OK, all ")); Serial.print(expDev) ; Serial.print(F(" devices are available\n"));
	}
	else if (knownDev<expDev && totalDev==expDev) {
		Serial.print(F("I2C selftest failed, addressing error on ")) ; Serial.print(expDev-knownDev) ; Serial.print(F(" device"));
		if(expDev-knownDev>1){
			Serial.print(F("s\n"));
		} else {
			Serial.print(F("\n"));
		}
	}
	else if (totalDev<expDev) {
		Serial.print(F("I2C selftest failed, missing ")); Serial.print(expDev-totalDev); Serial.print(F(" device(s)\n"));
	} 
	else if (totalDev==0) {
		Serial.print(F("I2C selftest failed, no devices found, check PCA9516"));
	}
	else {
		Serial.print(F("Some weird I2C problem occured (config wrong?), total: ")); Serial.print(totalDev) ; Serial.print(F(", known: ")); Serial.print(knownDev) ; Serial.print(F(", expected: ")); Serial.println(expDev) ;
	}
	
	timeOutDelay = tempTime;
}


/**************************
* Enables I2C hub PCA9516 *
**************************/
void Utils::enableI2C() {
	sbi(PORTE, 7);			// Set bit 7 on PORTE
	Serial.println(F("I2C hub is switched on"));
}

/***************************
* Disables I2C hub PCA9516 *
***************************/
void Utils::disableI2C() {
	cbi(PORTE, 7);			// Reset bit 7 on PORTE
	Serial.println(F("I2C hub is switched off"));
}

/*********************************
* Enables LED outputs on PCA9536 *
*********************************/
void Utils::enableLED(boolean verbose) {
	cbi(PORTG, 3);			// Reset bit 3 on PORTG
	IconRingLEDs = ENABLED;
	if(verbose) Serial.println(F("RGB and icon LEDs are switched on"));
}

/**********************************
* Disables LED outputs on PCA9536 *
**********************************/
void Utils::disableLED(boolean verbose) {
	sbi(PORTG, 3);			// Set bit 3 on PORTG
	IconRingLEDs = DISABLED;
	if(verbose) Serial.println(F("RGB and icon LEDs are switched off"));
}

/****************************
* Asserts WiFi default line *
****************************/
void Utils::assertDefaultWiFi() {
	cbi(PORTJ, 5);			// Set bit 5 on PORTJ
	Serial.println(F("WiFi defaults line is asserted"));
}

/******************************
* Deasserts WiFi default line *
******************************/
void Utils::deAssertDefaultWiFi() {
	sbi(PORTJ, 5);			// Reset bit 5 on PORTJ
	Serial.println(F("WiFi defaults line is deasserted"));
}

/*******************************
* Enables WiFi module XPCW1002 *
*******************************/
void Utils::enableWiFi() {
	if((HardwareCommConfig&CONFIG_WiFi)!=DISABLED) {
		sbi(PORTJ, 6);			// Set bit 6 on PORTJ
		PowerState|=POWERWIFION;
		Serial.println(F("WiFi is switched on"));
	} else {
		cbi(PORTJ, 6);			// Reset bit 6 on PORTJ
		PowerState&=POWERWIFIOFF;
		Serial.println(F("WiFi is not installed"));
	}
	
}	

/********************************
* Disables WiFi module XPCW1002 *
********************************/
void Utils::disableWiFi() {
	cbi(PORTJ, 6);			// Reset bit 6 on PORTJ
	PowerState&=POWERWIFIOFF;
	if((HardwareCommConfig&CONFIG_WiFi)!=DISABLED) {
		Serial.println(F("WiFi is switched off"));
	} else {
		Serial.println(F("WiFi is not installed"));
	}
}

/********************************
* Queries WiFi module XPCW1002 *
********************************/
int Utils::queryWiFi() {
	if((PORTJ & 0x40)!=0) {	// If not 0 then enabled
		Utils::enableWiFi();
		return 1;
	} else {
		Utils::disableWiFi();
		return 0;
	}
}

/*****************************
* Enables MiRa module *
*****************************/
void Utils::enableMiRa() {
	if((HardwareCommConfig&CONFIG_MiRa)!=DISABLED) {
		sbi(PORTJ, 2);            // Set bit 2 on PORTJ
		PowerState|=POWERMIRAON;
		Serial.println(F("MiRa is switched on")); 
	} else {
		cbi(PORTJ, 2);            // Reset bit 2 on PORTJ
		PowerState&=POWERMIRAOFF;
		Serial.println(F("MiRa is not installed")); 
	}
}

/******************************
* Disables MiRa module *
******************************/
void Utils::disableMiRa() {
	cbi(PORTJ, 2);            // Reset bit 2 on PORTJ
	PowerState&=POWERMIRAOFF;
	if((HardwareCommConfig&CONFIG_MiRa)!=DISABLED) {
		Serial.println(F("MiRa is switched off")); 
	} else {
		Serial.println(F("MiRa is not installed")); 
	}
}

/********************************
* Queries Mira module *
********************************/
int Utils::queryMiRa() {
     if((PORTJ & 0x04)!=0) {    // If not 0 then enabled
         Utils::enableMiRa();
         return 1;
     } else {
         Utils::disableMiRa();
         return 0;
     }
}


/*****************************
* Enables LoRa module RN2483 *
*****************************/
void Utils::enableLoRa() {
	if((HardwareCommConfig&CONFIG_LoRa)!=DISABLED) {
		sbi(PORTH, 4);			// Set bit 4 on PORTH
		PowerState|=POWERLORAON;
		Serial.println(F("LoRa is switched on"));
	} else {
		cbi(PORTH, 4);			// Reset bit 4 on PORTH
		PowerState&=POWERLORAOFF;
		Serial.println(F("LoRa is not installed"));
	}
}

/******************************
* Disables LoRa module RN2483 *
******************************/
void Utils::disableLoRa() {
	cbi(PORTH, 4);			// Reset bit 4 on PORTH
	PowerState&=POWERLORAOFF;
	if((HardwareCommConfig&CONFIG_LoRa)!=DISABLED) {
		Serial.println(F("LoRa is switched off"));
	} else {
		Serial.println(F("LoRa is not installed"));
	}
}

/*****************************
* Queries LoRa module RN2483 *
*****************************/
int Utils::queryLoRa() {
	if((PORTH & 0x10)!=0) {	// If not 0 then enabled
		Utils::enableLoRa();
		return 1;
	} else {
		Utils::disableLoRa();
		return 0;
	}
}

/**********************************************************************************
* Returns number to check if communication module is connected or just powered on *
**********************************************************************************/
uint32_t Utils::checkConnections() {
	uint32_t r=0;
	if((PIND & 0x80) == 0) r |= POWERWIFION; else r &= POWERWIFIOFF;			// Wifi connected
	return r;
}

/********************
* Powers CO2 module *
********************/
void Utils::powerCO2(uint8_t state, boolean verbose) {
	if(state==ENABLED) {
		T6713.powerOn();
		PowerState|=POWERCO2ON;
		if(verbose==true) Serial.print(F("CO2 sensor is switched on\n"));
	}
	else {
		T6713.powerOff();
		PowerState&=POWERCO2OFF;
		if(verbose==true) Serial.print(F("CO2 sensor is switched off\n"));
	}
}

/*******************
* Powers PM module *
*******************/
void Utils::powerPM(uint8_t state, boolean verbose) {
	if(state==ENABLED) {
		JosenePM.powerOn();
		PowerState|=POWERPMON;
		if(verbose==true) Serial.print(F("Particle sensor is switched on\n"));
	}
	else if(state==CYCLIC) {
		if(verbose==true) Serial.print(F("Particle sensor is cyclically "));
		switch (PMMode[POWERMODE]) {
			case POWERINGUP :
				if(verbose==true) Serial.println(F("powering up"));
				JosenePM.powerOn();
				PowerState|=POWERPMON;
				break;
			case POWEREDUP :
				if(verbose==true) Serial.println(F("powered up"));
				JosenePM.powerOn();
				PowerState|=POWERPMON;
				break;
			case POWERINGDOWN :
				if(verbose==true) Serial.println(F("powering down"));
				JosenePM.powerOff();
				PowerState&=POWERPMOFF;
				break;
			case POWEREDDOWN :
				if(verbose==true) Serial.println(F("powered down"));
				JosenePM.powerOff();
				PowerState&=POWERPMOFF;
				break;
			default :
				Serial.println(F("state unknown"));
				break;
		}
	}
	else {
		JosenePM.powerOff();
		PowerState&=POWERPMOFF;
		if(verbose==true) Serial.print(F("Particle sensor is switched off\n"));
	}
}

/********************
* Enables DebugLEDs *
********************/
void Utils::enableDebugLEDs(boolean verbose) {
	sbi(PORTH, 7);			// Set bit 7 on PORTH
	Utils::ctrlDebugLEDs(ALL, OFF);
	if(verbose) Serial.println(F("DebugLEDs are switched on"));
}

/*********************************************
* Disables Selectable demos by accelerometer *
*********************************************/
void Utils::disableDemos(boolean verbose) {
	DemoModes = DISABLED;
	if(verbose) Serial.println(F("Demo's are not selectable with accelerometer"));
}

/********************************************
* Enables Selectable demos by accelerometer *
********************************************/
void Utils::enableDemos(boolean verbose) {
	DemoModes = ENABLED;
	if(verbose) Serial.println(F("Demo's are selectable with accelerometer"));
}

/*********************
* Disables DebugLEDs *
*********************/
void Utils::disableDebugLEDs(boolean verbose) {
	cbi(PORTH, 7);			// Reset bit 7 on PORTH
	Utils::ctrlDebugLEDs(ALL, OFF);
	if(verbose) Serial.println(F("DebugLEDs are switched off"));
}

/*********************************************
* Controls various led patterns on debugLEDs *
*********************************************/
void Utils::ctrlDebugLEDs(byte mode, byte ctrl) {
	if((PINH & 0x80) != 0) {
		if(mode==FORWARD){
			for(uint8_t x = 0; x < 8; x++) {
				if (ctrl==ON) {
					cbi(PORTL,x);
				} else {
					sbi(PORTL,x);
				}
				delay(62);
			}
		}
		if(mode==BACKWARD) {
			for(uint8_t x = 8; x > 0; x--) {
				if (ctrl==ON) {
					cbi(PORTL,x-1);
				} else {
					sbi(PORTL,x-1);
				}
				delay(62);
			}
		}
		if(mode==ALL) {
			if(ctrl==ON) {
				PORTL = 0x00;
			} else {
				PORTL = 0xFF;
			}
		}
		if(mode<8) {
			if(ctrl==ON) {
				cbi(PORTL, mode);
			} else if (ctrl==OFF) {
				sbi(PORTL, mode);
			} else if (ctrl==TOGGLE) {
				if((PORTL&(1<<mode))!=0) {
					cbi(PORTL, mode);
				} else {
					sbi(PORTL, mode);
				}
			}
		}
	} else {
		PORTL = 0xFF;
	}
}


/*****************************************
* Write data to EEPROM, data is uint32_t *
* If length==4, all data is saved		 *
* If length==2, lower 2 bytes are saved  *
* If length==1, lower 1 byte is saved    *
*****************************************/
uint32_t Utils::writeEEPROM(uint16_t Address, uint32_t Data, uint8_t Length) {
	if(Length>4 || Length==3) return 0;
	if(Address>EEPROM.length()) return 0;
	Utils::ctrlDebugLEDs(LEDMem, TOGGLE);
	
	if(Length==1){
		EEPROM.write(Address, (uint8_t)(Data & 0x000000FF));
	}
	if(Length==2){
		EEPROM.write(Address, (uint8_t)((Data & 0x0000FF00)>>8));
		EEPROM.write((Address+1), (uint8_t)(Data & 0x000000FF));
	}
	if(Length==4){
		EEPROM.write(Address, (uint8_t)((Data & 0xFF000000)>>24));
		EEPROM.write((Address+1), (uint8_t)((Data & 0x00FF0000)>>16));
		EEPROM.write((Address+2), (uint8_t)((Data & 0x0000FF00)>>8));
		EEPROM.write((Address+3), (uint8_t)(Data & 0x000000FF));
	}
	Utils::ctrlDebugLEDs(LEDMem, TOGGLE);
	return Data;
}

uint32_t Utils::readEEPROM(uint16_t Address, uint8_t Length) {
	uint32_t Data = 0;
	if(Length>4 || Length==3) return 0;
	if(Address>EEPROM.length()) return 0;
	Utils::ctrlDebugLEDs(LEDMem, TOGGLE);
	if(Length==1) Data = EEPROM.read(Address);
	if(Length==2) Data = ((uint32_t)((EEPROM.read(Address))<<8) | EEPROM.read(Address+1));
	if(Length==4) Data = ((uint32_t)(EEPROM.read(Address))<<24 | (uint32_t)(EEPROM.read(Address+1))<<16 | (uint32_t)(EEPROM.read(Address+2))<<8 | EEPROM.read(Address+3));
	Utils::ctrlDebugLEDs(LEDMem, TOGGLE);
	return Data;
}

void Utils::loadAndApplyParameters() {
	if(Utils::readEEPROM(MEM_HARDWARETYPE,2)!=0x1603) Utils::defaultParameters();
	HardwareRev = Utils::readEEPROM(MEM_HARDWAREREV,2);
	HardwareConfig = Utils::readEEPROM(MEM_CONFIGURATION,4);
	Serial.print(F("PCB number "));
	Serial.print(Utils::readEEPROM(MEM_HARDWARETYPE,2),HEX);
	Serial.print(F(", revision "));
	Serial.print(((HardwareRev & 0xFF00) >> 8),HEX);
	Serial.print(F(", variant "));
	Serial.println((HardwareRev & 0x00FF),HEX);
	
	switch (HardwareRev & 0x00FF) {
		case 0x10 : HardwareCommConfig = CONFIG_USB + CONFIG_WiFi + CONFIG_LoRa + CONFIG_MiRa;
			break;
		case 0x20 : HardwareCommConfig = CONFIG_USB + CONFIG_WiFi;
			break;
		case 0x30 : HardwareCommConfig = CONFIG_USB;
			break;
		case 0x40 : HardwareCommConfig = CONFIG_USB + CONFIG_WiFi + CONFIG_LoRa;
			break;
		case 0x50 : HardwareCommConfig = CONFIG_USB + CONFIG_WiFi + CONFIG_LoRa + CONFIG_MiRa;
			break;
		case 0x60 : HardwareCommConfig = CONFIG_USB + CONFIG_WiFi + CONFIG_MiRa;
			break;
		default :
			Serial.println(F("Variant unknown"));
			break;
	}
	Serial.print(F("Installed communication options: "));
	if((HardwareCommConfig&CONFIG_USB)!=DISABLED) Serial.print(F("USB"));
	if((HardwareCommConfig&CONFIG_WiFi)!=DISABLED) Serial.print(F(", WiFi"));
	if((HardwareCommConfig&CONFIG_LoRa)!=DISABLED) Serial.print(F(", LoRa"));
	if((HardwareCommConfig&CONFIG_MiRa)!=DISABLED) Serial.print(F(", MiRa"));
	Serial.print(F("\n"));
	
	if((HardwareConfig&CONFIG_AUDIOPRESSURE)!=DISABLED) Serial.println(F("Soundpressure sensor enabled"));
	if((HardwareConfig&CONFIG_EXTTEMPSENSOR)!=DISABLED) Serial.println(F("External temperature sensor enabled"));

	UnitID = Utils::readEEPROM(MEM_ID,4);
	BaseTimer = (Utils::readEEPROM(MEM_BASETIMER,2))*50;
	if(BaseTimer>=0x31FFCE) {
		BaseTimer=0;
		Serial.println(F("BaseTimer is disabled, pushdata is switched off"));
	}
	if(((Utils::readEEPROM(MEM_DEBUGLEDS,1)&0x01)==0x00)) Utils::disableDebugLEDs(true); else Utils::enableDebugLEDs(true);
	if(((Utils::readEEPROM(MEM_DEBUGLEDS,1)&0x04)==0x00)) Utils::disableLED(true); else Utils::enableLED(true);
	if((Utils::readEEPROM(MEM_DEMOENABLED,1)==0x00)) Utils::enableDemos(true); else Utils::disableDemos(true);
	CalibrationTimer=(Utils::readEEPROM(MEM_CALTIMER,1))*3600;
	PMMode[MEASMODE]=Utils::readEEPROM(MEM_PMBOOTMODE, 1);
}

void Utils::defaultParameters() {
	Serial.println(F("Loading default parameters"));
	
	Utils::writeEEPROM(MEM_HARDWARETYPE, 0x1603, 2);			// Default hardware type
	Utils::writeEEPROM(MEM_CONFIGURATION,0,4);					// Default configuration
	Utils::writeEEPROM(MEM_UPTIME, 0, 4);						// Total uptime reset
	Utils::writeEEPROM(MEM_BASETIMER, 0xFFFF, 2);				// Basetimer reset
	Utils::writeEEPROM(MEM_DEBUGLEDS, 0xFF, 1);					// DebugLEDs and ringLEDs on
	Utils::writeEEPROM(MEM_DEMOENABLED, 0x00, 1);				// Demomodes selectable by accelerometer
	Utils::writeEEPROM(MEM_AMPFACTOR, 200, 2);					// Audio Amplification factor
	Utils::writeEEPROM(MEM_CALTIMER, 96, 1);					// Calibration timer 96 hours
	
	// Audio correction curve
	for(uint8_t x=0x70; x<=0x85; x++) Utils::writeEEPROM(x,0x00,1);
	Utils::writeEEPROM(0x86,0xFF,1);
	Utils::writeEEPROM(0x87,0xFF,1);
	Utils::writeEEPROM(0x88,0xFE,1);
	Utils::writeEEPROM(0x89,0xFE,1);
	Utils::writeEEPROM(0x8A,0xFD,1);
	Utils::writeEEPROM(0x8B,0xFC,1);
	Utils::writeEEPROM(0x8C,0xFA,1);
	Utils::writeEEPROM(0x8D,0xFA,1);
	Utils::writeEEPROM(0x8E,0xFB,1);
	
	// CO2 sensor defaults
	Utils::writeEEPROM(MEM_CO2BOOTMODE, 0x01, 1);				// Bootmode permanent on
	
	// PM sensor defaults
	Utils::writeEEPROM(MEM_PMBOOTMODE, 0x08, 1);				// Bootmode cyclic
	Utils::writeEEPROM(MEM_PMPOWERINGUP, 15, 2);				// Powering up, 15 seconds
	Utils::writeEEPROM(MEM_PMPOWEREDUP, 15, 2);					// Powered up, 15 seconds
	Utils::writeEEPROM(MEM_PMPOWERINGDOWN, 0, 2);				// Powering down, 0 seconds
	Utils::writeEEPROM(MEM_PMPOWEREDDOWN, 90, 2);				// Powered down, 90 seconds

	// Location reset
	Utils::writeEEPROM(MEM_LAT,0xFFFFFFFF,4);
	Utils::writeEEPROM(MEM_LON,0xFFFFFFFF,4);
}

/****************
* Show helptext *
****************/
void Utils::showHelp() {
	Utils::ctrlDebugLEDs(LEDUSB, TOGGLE);
	Serial.println(F("Commandline Help, commands are not case sensitive."));
	Serial.println(F("--------------------------------------------------"));
	Serial.println(F("->> Communication commands <<-"));
	Serial.println(F("Comms Query   - Check status of communication options"));
	Serial.println(F("USB Query     - Check status of USB connection"));
	Serial.println(F("Wifi Query    - Check status of WiFi module"));
	Serial.println(F("Wifi On       - Switch WiFi module on"));
	Serial.println(F("Wifi Off      - Switch WiFi module off"));
	Serial.println(F("Lora Query    - Check status of LoRa module"));
	Serial.println(F("Lora On       - Switch LoRa module on"));
	Serial.println(F("Lora Off      - Switch LoRa module off"));
	Serial.println(F("Mira Query    - Check status of Mira module"));
	Serial.println(F("Mira On       - Switch MiRa module on"));
	Serial.println(F("Mira Off      - Switch MiRa module off"));
	Serial.println(F("->> Programming assist commands <<-"));
	Serial.println(F("DebugLEDs On  - Switch debugleds on"));
	Serial.println(F("DebugLEDs Off - Switch debugleds off"));
	Serial.println(F("->> Sensor value commands <<-"));
	Serial.println(F("Get ID        - Returns server ID"));
	Serial.println(F("Get Session   - Returns session timer"));
	Serial.println(F("Get Total     - Returns total up timer"));
	Serial.println(F("Get Power     - Returns current powerstate"));
	Serial.println(F("Get Unittemp  - Returns unittemperature"));
	Serial.println(F("Get Exttemp   - Returns external temperature"));
	Serial.println(F("Get Temp      - Returns temperate"));
	Serial.println(F("Get Time      - Returns time hh:mm:ss"));
	Serial.println(F("Get Date      - Returns date ddd DD-mm-yy"));
	Serial.println(F("Get Humidity  - Returns humidity"));
	Serial.println(F("Get Pressure  - Returns pressure"));
	Serial.println(F("Get Light     - Returns light"));
	Serial.println(F("Get Angle     - Returns device angle"));
	Serial.println(F("Get Accelero  - Returns X and Y in G"));
	Serial.println(F("Get Location  - Returns programmed lat and lon"));
	Serial.println(F("Get CO2       - Returns CO2 value"));
	Serial.println(F("Cal CO2       - Starts CO2 calibration, takes ~10 mins"));
	Serial.println(F("CO2 On        - Switch CO2 sensor on"));
	Serial.println(F("CO2 Off       - Switch CO2 sensor off"));
	Serial.println(F("PM On         - Switch particle sensor on"));
	Serial.println(F("PM Off        - Switch particle sensor off"));
	Utils::ctrlDebugLEDs(LEDUSB, TOGGLE);
}
