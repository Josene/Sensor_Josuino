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
* Rev 1.3 - June 2017 																*
* - Added support for easier led control on demos									*
* Rev 1.4 - August 2017 															*
* - Added support for living lab sensor            									*
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

//pca9635 Icons;
//pca9635 RGB[6];
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
A2235H A2235H;
AA025E64 AA025E64;
Powercontroller Powercontroller;
extern LEDRing LEDRing;
extern IconRing IconRing;

uint8_t TimerCount = 0;
uint8_t CursorPos = 0xFF;

// Declaration of globals used for asynchronically transmit data
uint8_t Vermajor = 1;
uint8_t Verminor = 40;

uint32_t UnitID[2] = {0,0};

uint32_t PowerState = 0;
uint32_t PowerStateVisual = 1;
uint32_t ConnectionState = 0;
uint32_t ConnectionStateVisual = 0;
uint32_t ErrorState = 0;
uint32_t ErrorStateVisual = 0;
uint32_t SessionTimer = 0;
uint32_t TotalUpTimer = 0;
uint16_t PMTimer = 0;
uint8_t PMMode[2] = {DISABLED, POWERINGUP};
uint32_t CalibrationTimer = 0;
uint8_t IconRingLEDs = ENABLED;
uint8_t DemoModes = ENABLED;
uint16_t Demo = 0;		// 8 MSB bits = used sensor (thus icon, 8 LSB = demo number

uint16_t DemoVisual = 0;
uint16_t DemoCounter = 0;
uint16_t DemoCursorActive = OFF;


uint32_t MemoryPointer = 0;
uint32_t BaseTimer = 0;
uint32_t HardwareType = 0;				// Describes used PCB type
uint32_t HardwareRev = 0;				// Describes used PCB revision
uint32_t HardwareConfig = 0;			// Describes installed optional modules
uint32_t HardwareCommConfig = 0;		// Describes installed communication options
uint8_t ErrorTimer = 15;					// Used during booting for debugleds and errorhandling.
uint16_t DebugTimer = DEBUGTIMERMAX;
float UnitTemperature = 0;
float ExtTemperature = 0;
float Temperature = 0;
float Humidity = 0;
float Pressure = 0;
uint16_t LightIntensity = 0;
uint16_t CO2gas = 0;
uint16_t PM10 = 0;
uint16_t PM2_5 = 0;
uint16_t CO2Status = 0;
uint16_t TxDataCounter = 100; //TXTIMERMAX;
uint8_t RTC[7] = {0,0,0,1,0,0,0};
uint8_t OldSec = 0;
uint32_t Longitude = 0;					// Programmed longitude
uint32_t Latitude = 0;					// Programmed latitude
uint32_t DopFixSiv = 0;					// GPS signal quality

struct GPSDataSet GPS;
struct PSUDataSet PSU;
struct AccelerometerDataSet ACC;
uint8_t PowerctrlTest = 0;

uint8_t LaEQ[125]={ };
uint8_t LaEQPeak = 0;


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
				case 'I': Comms::txIF(port, va_arg(argv,uint16_t)); break;
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
		case 1: dow = "Mon ";break;
		case 2: dow = "Tue ";break;
		case 3: dow = "Wed ";break;
		case 4: dow = "Thu ";break;
		case 5: dow = "Fri ";break;
		case 6: dow = "Sat ";break;
		case 7: dow = "Sun ";break;
		default: dow = "";break;
	};
	switch(port) {
		case IF_USB: Serial.print(dow);break;
		case IF_WiFi: Serial1.print(dow);break;
		case IF_LoRa: Serial2.print(dow);break;
		case IF_MiRa: Serial3.print(dow);break;
	};

}

void Comms::txIF(uint8_t port, uint8_t n) {
	String i_f;
	switch(n) {
		case IF_USB: i_f = "USB";break;
		case IF_WiFi: i_f = "WiFi";break;
		case IF_LoRa: i_f = "LoRa";break;
		case IF_MiRa: i_f = "MiRa";break;
		default: i_f = "-?-";break;
	};
	switch(port) {
		case IF_USB: Serial.print(i_f);break;
		case IF_WiFi: Serial1.print(i_f);break;
		case IF_LoRa: Serial2.print(i_f);break;
		case IF_MiRa: Serial3.print(i_f);break;
	};
}

void Comms::parseISP(uint8_t port, uint8_t *d) {
	TimerCount=250;
    uint8_t ISPlen = 0;
    uint32_t ISPdest = 0;
	ISPdest=((uint32_t)(d[3])<<24)|((uint32_t)(d[2])<<16)|((uint32_t)(d[1])<<8)|((uint32_t)(d[0]));
	ISPlen=d[15]+13;
	if(ISPdest==SP2_TARGET_ID_SCLL_PWR) Powercontroller.writeISP(ISPdest, ISPlen, d+4);

	
	//Comms::checkISP(ISPdest, d+4);
}

void Comms::checkISP(uint8_t dest, uint8_t *d) {
	
	
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
			if(port!=IF_USB) Comms::txData(IF_USB,"Ext. Sensor Request via %I, Opr: 0x%H, Opd: 0x%h\n",port,Opr,Opd);
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
			if(port!=IF_USB) Comms::txData(IF_USB,"Ext. Control Request via %I, Opr: 0x%H, Opd: 0x%h\n",port,Opr,Opd);
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
			if(port!=IF_USB) Comms::txData(IF_USB,"Ext. Memory Request via %I, Opr: 0x%H, Opd: 0x%h\n",port,Opr,Opd);
			if(Opr>=0x01 && Opr<=0x04) {
				MemoryPointer=Opd;
				Comms::txMemory(port,Utils::readEEPROM(Opd,Opr),Opr,RD);
			}
			if(Opr>=0x81 && Opr<=0x84) {
				Comms::txMemory(port,Utils::writeEEPROM(MemoryPointer,Opd,(Opr&0x7F)),(Opr&0x7F),WR);
			}
			break;
		case 'L':
			if(port!=IF_USB) Comms::txData(IF_USB,"Ext. Demo Request via %I, Opr: 0x%H, Opd: 0x%h\n",port,Opr,Opd);
			if(Opr==0xDE) {
				Comms::txData(port,"%>YDE%h&%<", (uint32_t)(Opd));				
				Demo=Opd+0x80;
			}
			if(Opr==0x01) {
                uint8_t Br = 0x08;
                // If on mains, high power is available
                if((PowerState&0x80)!=0) Br=0x0C;
                Comms::txData(port,"%>Y01%h&%<", (uint32_t)(Opd));
                Serial.println();
                uint8_t a[4];
                *(uint32_t *)a = Opd;
				LEDRing.singleLED(a[0],Br,(a[1]&0x0F)*0x11,((a[1]&0xF0)>>4)*0x11,(a[2]&0x0F)*0x11,(a[2]&0xF0)>>4);
            }
            if(Opr==0x02) {
				uint8_t Br = 0x08;
                // If on mains, high power is available
                if((PowerState&0x80)!=0) Br=0x0C;
                Comms::txData(port,"%>Y02%h&%<", (uint32_t)(Opd));
                Serial.println();
                uint8_t a[4];
                *(uint32_t *)a = Opd;
				LEDRing.rangeLED(a[0],a[1],Br,(a[2]&0x0F)*0x11,((a[2]&0xF0)>>4)*0x11,(a[3]&0x0F)*0x11,(a[3]&0xF0)>>4);
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
	if (cmd=="get batvolt") {
		Comms::txSensor(port, PR_VERBOSE, SENSOR_BATTERYVOLTAGE);
		return 1;
	}
	if (cmd=="get psuvolt") {
		Comms::txSensor(port, PR_VERBOSE, SENSOR_PSUVOLTAGE);
		return 1;
	}
	if (cmd=="get psustatus") {
		Comms::txSensor(port, PR_VERBOSE, SENSOR_PSUVOLTAGE);
		Comms::txSensor(port, PR_VERBOSE, SENSOR_PSUCURRENT);
		Comms::txSensor(port, PR_VERBOSE, SENSOR_BATTERYVOLTAGE);
		Comms::txSensor(port, PR_VERBOSE, SENSOR_BATTERYCURRENT);
		Comms::txSensor(port, PR_VERBOSE, SENSOR_BATTERYTEMP);
		Comms::txSensor(port, PR_VERBOSE, SENSOR_COULOMBCOUNT);
		return 1;
	}
	if (cmd=="reset led") {
		LEDRing.reset(HardwareType);
		Comms::txData(IF_USB, "Done\n");
		return 1;
	}
	if(cmd=="show nothing" || cmd=="demo off") {
		Demo=0x00;
		Comms::txData(IF_USB, "Demo mode switched off\n");
		return 1;
	}
	if(cmd=="show temp") { 
		Demo=0x81;
		Comms::txData(IF_USB, "Showing temperature on LEDs using KNMI colours\n");
		return 1;
	}
	if(cmd=="show humidity") { 
		Demo=0x82;
		Comms::txData(IF_USB, "Showing humidity on LEDs using KNMI colours\n");
		return 1;
	}
	if(cmd=="show pressure") { 
		Demo=0x83;
		Comms::txData(IF_USB, "Showing pressure on LEDs using KNMI colours\n");
		return 1;
	}
	if(cmd=="show co2") { 
		Demo=0x84;
		Comms::txData(IF_USB, "Showing CO2 level on LEDs using RIVM colours\n");
		return 1;
	}
	if(cmd=="show pm10") { 
		Demo=0x85;
		Comms::txData(IF_USB, "Showing PM10 on LEDs using RIVM colours\n");
		return 1;
	}
	if(cmd=="show pm2.5") { 
		Demo=0x86;
		Comms::txData(IF_USB, "Showing PM2.5 on LEDs using RIVM colours\n");
		return 1;
	}
	if(cmd=="show demo") { 
		Demo=0x87;
		Comms::txData(IF_USB, "Showing demo on LEDs\n");
		return 1;
	}

	// if (cmd=="get gps time") {
		// Comms::txData(IF_USB, "GPS time: %u, date: %u\n", GPS.Time, GPS.Date);
		// return 1;
	// }
	
	if (cmd=="get all") {
		for(uint8_t x=PUSH_MAX;x>0;x--) {
			Comms::txSensor(port, PR_VERBOSE, x);
			
		}
		PCB150011.clearThirdsSpectrum(LaEQ);
		return 1;
	}
	if (cmd=="get audio") {
		Comms::txData(IF_USB, "Highest audio peak %d dB(A)\n",LaEQPeak);
		return 1;
	}
	if (cmd=="get test") {
		Powercontroller.getTest();
		Comms::txData(IF_USB, "Done\n");
		return 1;
	}
	
	if (cmd=="get accelerometer" || cmd=="get accelero"){
		Comms::txSensor(port, PR_VERBOSE, SENSOR_ACCELEROX);
		Comms::txSensor(port, PR_VERBOSE, SENSOR_ACCELEROY);
		return 1;
	}
	if (cmd=="get location"){
		Comms::txSensor(port, PR_VERBOSE, SENSOR_DOPFIXSIV);
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
	if (cmd=="get nwk") {
		Comms::txSensor(port, PR_VERBOSE, EXTREQ_NWKKEY);
		return 1;
	}
	if (cmd=="get aes") {
		Comms::txSensor(port, PR_VERBOSE, EXTREQ_AESKEY0);
		Comms::txSensor(port, PR_VERBOSE, EXTREQ_AESKEY1);
		Comms::txSensor(port, PR_VERBOSE, EXTREQ_AESKEY2);
		Comms::txSensor(port, PR_VERBOSE, EXTREQ_AESKEY3);
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
	if (cmd=="gps on")	{
		Utils::powerGPS(ENABLED);
		return 1;
	}
	if (cmd=="gps off")	{
		Utils::powerGPS(DISABLED);
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
		case EXTREQ_ID2 :
		case SENSOR_ID2 :
			if(protocol==PR_JOSE) Comms::txData(port,"%>PA1%h&%<", (uint32_t)(UnitID[1]));
			break;
		case EXTREQ_ID :
		case SENSOR_ID :
			if(protocol==PR_VERBOSE) {
				if(UnitID[1]==0) {
					Comms::txData(IF_USB, "Unit ID: %d\n", UnitID[0]);
				} else {
					Comms::txData(IF_USB, "Unit ID: 0x%h%h (EUI)\n", UnitID[1],UnitID[0]);
				}
			}
			if(protocol==PR_JOSE) Comms::txData(port,"%>PA0%h&%<", (uint32_t)(UnitID[0]));
			break;
		case EXTREQ_ERRORSTATE :
		case SENSOR_ERRORSTATE :
			if(protocol==PR_JOSE) Comms::txData(port,"%>P0C%h&%<", (uint32_t)(ErrorState));
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
		case EXTREQ_NWKKEY :
			if(protocol==PR_VERBOSE) Comms::txData(port, "NWK Key: 0x%H%H, Root: 0x%H\n", (uint8_t)(Utils::readEEPROM(MEM_MIRA_NWKKEY, 1)), (uint8_t)(Utils::readEEPROM(MEM_MIRA_NWKKEY+1, 1)), (uint8_t)(Utils::readEEPROM(MEM_MIRA_ROOT, 1)));
			if(protocol==PR_JOSE) Comms::txData(port, "%>PDF00%H%H%H&%<", (uint8_t)(Utils::readEEPROM(MEM_MIRA_ROOT, 1)), (uint8_t)(Utils::readEEPROM(MEM_MIRA_NWKKEY, 1)), (uint8_t)(Utils::readEEPROM(MEM_MIRA_NWKKEY+1, 1)));
			break;
		case EXTREQ_AESKEY0 :
			if(protocol==PR_VERBOSE) Comms::txData(port, "AES Key (little endian, hex): %H %H %H %H", (uint8_t)(Utils::readEEPROM(MEM_MIRA_AESKEY_0,1)), (uint8_t)(Utils::readEEPROM(MEM_MIRA_AESKEY_1,1)), (uint8_t)(Utils::readEEPROM(MEM_MIRA_AESKEY_2,1)), (uint8_t)(Utils::readEEPROM(MEM_MIRA_AESKEY_3,1)));
			if(protocol==PR_JOSE) Comms::txData(port, "%>PE0%H%H%H%H&%<", (uint8_t)(Utils::readEEPROM(MEM_MIRA_AESKEY_0,1)), (uint8_t)(Utils::readEEPROM(MEM_MIRA_AESKEY_1,1)), (uint8_t)(Utils::readEEPROM(MEM_MIRA_AESKEY_2,1)), (uint8_t)(Utils::readEEPROM(MEM_MIRA_AESKEY_3,1)));
			break;
		case EXTREQ_AESKEY1 :
			if(protocol==PR_VERBOSE) Comms::txData(port, " %H %H %H %H", (uint8_t)(Utils::readEEPROM(MEM_MIRA_AESKEY_4,1)), (uint8_t)(Utils::readEEPROM(MEM_MIRA_AESKEY_5,1)), (uint8_t)(Utils::readEEPROM(MEM_MIRA_AESKEY_6,1)), (uint8_t)(Utils::readEEPROM(MEM_MIRA_AESKEY_7,1)));
			if(protocol==PR_JOSE) Comms::txData(port, "%>PE1%H%H%H%H&%<", (uint8_t)(Utils::readEEPROM(MEM_MIRA_AESKEY_4,1)), (uint8_t)(Utils::readEEPROM(MEM_MIRA_AESKEY_5,1)), (uint8_t)(Utils::readEEPROM(MEM_MIRA_AESKEY_6,1)), (uint8_t)(Utils::readEEPROM(MEM_MIRA_AESKEY_7,1)));
			break;
		case EXTREQ_AESKEY2 :
			if(protocol==PR_VERBOSE) Comms::txData(port, " %H %H %H %H", (uint8_t)(Utils::readEEPROM(MEM_MIRA_AESKEY_8,1)), (uint8_t)(Utils::readEEPROM(MEM_MIRA_AESKEY_9,1)), (uint8_t)(Utils::readEEPROM(MEM_MIRA_AESKEY_A,1)), (uint8_t)(Utils::readEEPROM(MEM_MIRA_AESKEY_B,1)));
			if(protocol==PR_JOSE) Comms::txData(port, "%>PE2%H%H%H%H&%<", (uint8_t)(Utils::readEEPROM(MEM_MIRA_AESKEY_8,1)), (uint8_t)(Utils::readEEPROM(MEM_MIRA_AESKEY_9,1)), (uint8_t)(Utils::readEEPROM(MEM_MIRA_AESKEY_A,1)), (uint8_t)(Utils::readEEPROM(MEM_MIRA_AESKEY_B,1)));
			break;
		case EXTREQ_AESKEY3 :
			if(protocol==PR_VERBOSE) Comms::txData(port, " %H %H %H %H\n", (uint8_t)(Utils::readEEPROM(MEM_MIRA_AESKEY_C,1)), (uint8_t)(Utils::readEEPROM(MEM_MIRA_AESKEY_D,1)), (uint8_t)(Utils::readEEPROM(MEM_MIRA_AESKEY_E,1)), (uint8_t)(Utils::readEEPROM(MEM_MIRA_AESKEY_F,1)));
			if(protocol==PR_JOSE) Comms::txData(port, "%>PE3%H%H%H%H&%<", (uint8_t)(Utils::readEEPROM(MEM_MIRA_AESKEY_C,1)), (uint8_t)(Utils::readEEPROM(MEM_MIRA_AESKEY_D,1)), (uint8_t)(Utils::readEEPROM(MEM_MIRA_AESKEY_E,1)), (uint8_t)(Utils::readEEPROM(MEM_MIRA_AESKEY_F,1)));
			break;
		/*
		#define SENSOR_BATTERYVOLTAGE	0x3F
		#define SENSOR_PSUVOLTAGE		0x3E
		#define SENSOR_BATTERYCURRENT	0x3D
		#define SENSOR_PSUCURRENT		0x3C

		*/

		case EXTREQ_TIME :
		case SENSOR_TIME :
			if(protocol==PR_VERBOSE) Comms::txData(port,"Time: %D:%D:%D\n",RTC[HOURS],RTC[MINUTES],RTC[SECONDS]);
			if(protocol==PR_JOSE) Comms::txData(port,"%>S0E%h&%<", (uint32_t)(((uint32_t)RTC[HOURS] << 16) | ((uint32_t)RTC[MINUTES] << 8) | RTC[SECONDS]));
			break;
		case EXTREQ_DATE :
		case SENSOR_DATE :
			if(protocol==PR_VERBOSE) Comms::txData(port,"Date: %w%D-%D-%D\n",RTC[DAY],RTC[DATE],RTC[MONTH],RTC[YEAR]);
			if(protocol==PR_JOSE) Comms::txData(port,"%>S0F%h&%<", (uint32_t)(((uint32_t)RTC[YEAR] << 16) | ((uint32_t)RTC[MONTH] << 12) | ((uint32_t)RTC[DATE] << 4) | RTC[DAY]));
			break;
		case EXTREQ_POWERSTATE :
		case SENSOR_POWERSTATE :
			if(protocol==PR_VERBOSE) Comms::txData(port,"Powerstate: 0x%h\n",PowerState);
			if(protocol==PR_JOSE) Comms::txData(port,"%>P00%h&%<", (uint32_t)(PowerState));
			break;
		case EXTREQ_BATTERYVOLTAGE :
		case SENSOR_BATTERYVOLTAGE :
			if(protocol==PR_VERBOSE) Comms::txData(port,"Battery voltage: %f V\n",(float)((float)PSU.BatteryVoltage/(float)(1000)));
			if(protocol==PR_JOSE) Comms::txData(port,"%>P01%h&%<", (uint32_t)(PSU.BatteryVoltage));
			break;
		case EXTREQ_PSUVOLTAGE :
		case SENSOR_PSUVOLTAGE :
			if(protocol==PR_VERBOSE) Comms::txData(port,"Powersupply voltage: %f V\n",(float)((float)PSU.PSUVoltage/(float)(1000)));
			if(protocol==PR_JOSE) Comms::txData(port,"%>P05%h&%<", (uint32_t)(PSU.PSUVoltage));
			break;
		case EXTREQ_BATTERYCURRENT :
		case SENSOR_BATTERYCURRENT :
			if(protocol==PR_VERBOSE) {
				Comms::txData(port,"Battery current: %d mA",PSU.BatteryCurrent);
				if(PSU.BatteryCurrent<0) Comms::txData(port, " (charging)");
				Comms::txData(port,"\n");
			}
			if(protocol==PR_JOSE) Comms::txData(port,"%>P06%h&%<", (uint32_t)(PSU.BatteryCurrent));
			break;
		case EXTREQ_PSUCURRENT :
		case SENSOR_PSUCURRENT :
			if(protocol==PR_VERBOSE) Comms::txData(port,"Powersupply current: %d mA\n",PSU.PSUCurrent);
			if(protocol==PR_JOSE) Comms::txData(port,"%>P07%h&%<", (uint32_t)(PSU.PSUCurrent));
			break;
		case EXTREQ_BATTERYTEMP :
		case SENSOR_BATTERYTEMP :
			if(protocol==PR_VERBOSE) Comms::txData(port,"PSU Temperature: %d degr C\n",(PSU.PSUTemperature-273));
			if(protocol==PR_JOSE) Comms::txData(port,"%>P02%h&%<", ((uint32_t)(PSU.PSUTemperature))*1000);
			break;
		case EXTREQ_COULOMBCOUNT :
		case SENSOR_COULOMBCOUNT :
			if(protocol==PR_VERBOSE) Comms::txData(port,"Coulombcounter: 0x%h \n",(uint32_t)(PSU.BatteryCoulombs));
			if(protocol==PR_JOSE) Comms::txData(port,"%>P03%h&%<", (uint32_t)(PSU.BatteryCoulombs));
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
			if(protocol==PR_VERBOSE) Comms::txData(port,"Accelerometer X: %f G\n",ACC.XAxis);
			if(protocol==PR_JOSE)Comms::txData(port,"%>S1A%h&%<", (uint32_t)(float)((ACC.XAxis*256)+512));
			break;
		case EXTREQ_ACCELEROY :
		case SENSOR_ACCELEROY :
			if(protocol==PR_VERBOSE) Comms::txData(port,"Accelerometer Y: %f G\n",ACC.YAxis);
			if(protocol==PR_JOSE) Comms::txData(port,"%>S1B%h&%<", (uint32_t)(float)((ACC.YAxis*256)+512));
			break;
//		case EXTREQ_ACCELEROZ :
//		case SENSOR_ACCELEROZ :
//			break;
		case EXTREQ_DEVICEANGLE :
		case SENSOR_DEVICEANGLE :
			if(HardwareType==0x1603){
				if(ACC.Degr==ACC.Degr) {
					if(protocol==PR_VERBOSE) Comms::txData(port,"Device angle: %f degr\n",ACC.Degr); 
					if(protocol==PR_JOSE) Comms::txData(port,"%>S2F%h&%<", (uint32_t)((ACC.Degr)*1000));
				} else {
					if(protocol==PR_VERBOSE) Comms::txData(port,"Device lays flat\n"); 
					if(protocol==PR_JOSE) Comms::txData(port,"%>S2F%h&%<", (uint32_t)(0xFFFFFFFF));
				}	
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

		case EXTREQ_DOPFIXSIV :
		case SENSOR_DOPFIXSIV :
			if(protocol==PR_VERBOSE) {
				if (GPS.Fix==0)	Comms::txData(port,"GPS data invalid, trying parameters\n");
				if (GPS.Fix==1)	Comms::txData(port,"GPS no fix, DOP: %f, satelites in view: %d, trying parameters\n",GPS.Dop, GPS.Siv);
				if (GPS.Fix>=2)	Comms::txData(port,"GPS %dD fix, DOP: %f, satelites in view: %d\n",GPS.Fix, GPS.Dop, GPS.Siv);
			}
			if(protocol==PR_JOSE) Comms::txData(port,"%>S40%h&%<", (uint32_t)(DopFixSiv));
			break;

		case EXTREQ_LATITUDE :
		case SENSOR_LATITUDE :
			if(protocol==PR_VERBOSE) {
				if(Latitude!=0xFFFFFFFF) {
					char frac[7];
					snprintf(frac, sizeof frac, "%06lu", (unsigned long)GPS.LatitudeFrac);
					Comms::txData(port,"Latitude: %d.%s degr\n",GPS.LatitudeDegr,frac);
				} else {
					Comms::txData(port,"Latitude parameter not programmed\n");
				}
				
			} 
			if(protocol==PR_JOSE && Latitude!=0xFFFFFFFF) Comms::txData(port,"%>S41%h&%<", (uint32_t)(Latitude));
			break;
		case EXTREQ_LONGITUDE :
		case SENSOR_LONGITUDE :
			if(protocol==PR_VERBOSE) {
				if(Longitude!=0xFFFFFFFF) {
					char frac[7];
					snprintf(frac, sizeof frac, "%06lu", (unsigned long)GPS.LongitudeFrac);
					Comms::txData(port,"Longitude: %d.%s degr\n",GPS.LongitudeDegr,frac);
				} else {
					Comms::txData(port,"Longitude parameter not programmed\n");
				}
			} 
			if(protocol==PR_JOSE && Longitude!=0xFFFFFFFF) Comms::txData(port,"%>S42%h&%<", (uint32_t)(Longitude));
			break;
		case SENSOR_EOM :
			if(protocol==PR_JOSE) Comms::txData(port,"%>PFF%h&%<", (uint32_t)(0));
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
	if(HardwareType==0x1711) {
		A2235H.begin(0x4D,1);				// Switch on GPS if hardwaretype supports this
		Powercontroller.begin(0x60,false);
		AA025E64.begin(0x57,false);
		AA025E64.getID(UnitID);
	}
	
	M41T00S.begin(0x68,false);

	if(PMMode[MEASMODE]==ENABLED) { 
		Utils::powerPM(ENABLED); 
	} else if(PMMode[MEASMODE]==CYCLIC) { 
		Utils::powerPM(CYCLIC);
	} else {
		Utils::powerPM(DISABLED);
	}
	if((HardwareConfig&CONFIG_AUDIOPRESSURE)!=DISABLED) {
	//if((Utils::readEEPROM(MEM_AMPFACTOR,2)!=0xFFFF)) {		// Check if AmpFactor is programmed, if yes, init soundmodule
		PCB150011.begin(0x10,false);
		PCB150011.setAmpFactor(Utils::readEEPROM(MEM_AMPFACTOR,2));
		if(((Utils::readEEPROM(MEM_DEBUGLEDS,1)&0x02)==0x00)) PCB150011.disableLED(); else PCB150011.enableLED();
		PCB150011.clearThirdsSpectrum(LaEQ);
	}
	if((HardwareConfig&CONFIG_EXTTEMPSENSOR)!=DISABLED) {
		DS2482S.begin(0x18,false);
	}
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
	//float *p;
	Utils::ctrlDebugLEDs(LEDI2C, ON);
	uint8_t mode = HORIZONTAL;
	if(HardwareType==0x1711) mode=VERTICAL;
	ADXL345.getSensorData(&ACC, mode);
	//C.XAxis=ACC.XAxis;
	//C.YAxis=ACC.YAxis;
	//C.Degr=ACC.Degr;
	//celeroI=0;
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

void Sensor::getPSUStatus() {
	if(HardwareType==0x1711){
		Utils::ctrlDebugLEDs(LEDI2C, ON);
		Powercontroller.getData(&PSU,false);
		Utils::ctrlDebugLEDs(LEDI2C, OFF);
	}
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

void Sensor::getGPS() {
	//float *p;
	Utils::ctrlDebugLEDs(LEDAsy, ON);
	A2235H.getData(&GPS,false);
	/*
	Serial.print(F("LAT:")); Serial.print(GPS.Latitude, 6); Serial.print(F(", "));
	Serial.print(F("LON:")); Serial.print(GPS.Longitude, 6); Serial.print(F(", "));
	Serial.print(F("FIX:")); Serial.print(GPS.Fix); Serial.print(F(", "));
	Serial.print(F("SIV:")); Serial.print(GPS.Siv); Serial.print(F(", "));
	Serial.print(F("DOP:")); Serial.print(GPS.Dop, 1); Serial.print(F(", "));
	Serial.print(F("Time:")); Serial.print(GPS.Time); Serial.print(F(", "));
	Serial.print(F("Date:")); Serial.print(GPS.Date); Serial.println();
	*/
	Utils::ctrlDebugLEDs(LEDAsy, OFF);
}

void Sensor::askGPS() {
	Utils::ctrlDebugLEDs(LEDAsy, ON);
	if(GPS.Mode==2) GPS.Mode=0; else GPS.Mode++;
	if(GPS.Fix<2) GPS.Mode=0;
	A2235H.queryGPS(GPS.Mode);
	Utils::ctrlDebugLEDs(LEDAsy, OFF);
}

void Sensor::getAudioSpectrum() {
	Utils::ctrlDebugLEDs(LEDI2C, ON);
	if((HardwareConfig&CONFIG_AUDIOPRESSURE)!=DISABLED) PCB150011.getThirdsSpectrum(LaEQ);
	LaEQPeak=LaEQ[125];
	//mms::txData(IF_USB, "Highest audio peak %d dB(A)\n",LaEQPeak);
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
	
	if(HardwareType==0x1711) Sensor::getGPS(); else GPS.Fix=0;
	
	if(GPS.Fix>0) { 
		Latitude=((uint32_t)(GPS.LatitudeDegr)<<20)|((uint32_t)(GPS.LatitudeFrac));
		Longitude=((uint32_t)(GPS.LongitudeDegr)<<20)|((uint32_t)(GPS.LongitudeFrac));
		DopFixSiv=((uint32_t)(GPS.DopDecimal)<<16)|((uint32_t)(GPS.DopFrac)<<12)|((uint32_t)(GPS.Fix)<<8)|((uint32_t)(GPS.Siv));
	} else {
		Latitude = Utils::readEEPROM(MEM_LAT,4);
		Longitude = Utils::readEEPROM(MEM_LON,4);
		DopFixSiv = 0;
		GPS.LatitudeDegr=(Latitude&0x0FF00000)>>20;
		GPS.LatitudeFrac=(Latitude&0x000FFFFF);
		if(Latitude&0x80000000!=0) GPS.LatitudeDegr*=-1;
		GPS.LongitudeDegr=(Longitude&0x0FF00000)>>20;
		GPS.LongitudeFrac=(Longitude&0x000FFFFF);
		if(Longitude&0x80000000!=0) GPS.LongitudeDegr*=-1;
	}
	
	if(HardwareType==0x1711) Sensor::askGPS();

}

void Sensor::checkValues() {
	if(ErrorTimer!=0) {
		//Serial.print(ErrorTimer);
		//Serial.print(" ");
		ErrorTimer--;
		return;
	}
	
	if(Utils::readEEPROM(MEM_PMBOOTMODE, 1)!=DISABLED) {
		if(PM10==0 || PM2_5==0 || PM10>999 || PM2_5>999) ErrorState|=ERRORPM; else ErrorState&= ~ERRORPM;
	}

	if(Utils::readEEPROM(MEM_CO2BOOTMODE, 1)!=DISABLED) {
		if((PowerState&POWERCO2ON)!=0) {
			if(CO2gas==0 || CO2gas>5000) ErrorState|=ERRORCO2; else ErrorState&= ~ERRORCO2;			
		} else {
			ErrorState&= ~ERRORCO2;	
		}
	}
		
	if((HardwareConfig&CONFIG_EXTTEMPSENSOR)!=DISABLED) {
		if(ExtTemperature==0xFFFF || ExtTemperature<-55 || ExtTemperature > 125) ErrorState|=ERROREXTTEMP; else ErrorState&= ~ERROREXTTEMP;
	}
	
	if((HardwareConfig&CONFIG_AUDIOPRESSURE)!=DISABLED) {
		if(LaEQPeak==0) ErrorState|=ERRORAUDIO; else ErrorState&= ~ERRORAUDIO;
	}
	// if((HardwareConfig&CONFIG_POWERCONTROLLER)!=DISABLED) Serial.println(F("External powercontroller enabled"));
	// if((HardwareConfig&CONFIG_LEDMATRICES)!=DISABLED && HardwareConfig==0x1603) Serial.println(F("LED ring enabled")); else Serial.println(F("LED matrices enabled"));
}


/************************************************************************************
*************************************************************************************
* LEDRing class																		*
*************************************************************************************
************************************************************************************/
LEDDemo::LEDDemo(void) {
	
}

void LEDDemo::cursor(float Angle) {
	uint8_t Br = 0x08;
	if(HardwareType==0x1603) if((PowerState&0x80)!=0) Br=0x0C;									// If on mains, high power is available
	if(HardwareType==0x1711) Br=0x0F;			
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
	Utils::disableWiFi(false);
	Utils::disableLoRa(false);
	Utils::disableMiRa(false);
	Utils::initI2C();
	Utils::enableI2C();
    Utils::checkI2C();
    if(IconRingLEDs==ENABLED) Utils::initLEDcontrollers();
	Sensor::begin();
}

void Utils::finishUp() {
	if(IconRingLEDs==ENABLED) LEDRing.startUpRing();
	if(IconRingLEDs==ENABLED) IconRing.startUpRing();
	Utils::setupTimers();
	interrupts();             // enable all interrupts
	Serial.println(F("\nFinished setup\n\nType 'help' + \\n for help"));
	//Demo=0x87;
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
		if(TimerCount==0) {
			TimerCount = TIMERMAX;
			Utils::ctrlDebugLEDs(LEDTmr, ON);
//			Serial.println(TimerCount);			
		} else if(TimerCount > TIMERMAX) {
			if((TimerCount%10)==0) Utils::ctrlDebugLEDs(LEDTmr, ON); else Utils::ctrlDebugLEDs(LEDTmr, OFF); 
			TimerCount--;
		} else {
			TimerCount--;
			Utils::ctrlDebugLEDs(LEDTmr, OFF);
//			Serial.print(TimerCount);			
		}
	}

/******************
* Sensor requests *
******************/
	if(TimerCount==49) Sensor::askExtTemperature();
	if(TimerCount==48) Sensor::getPSUStatus();
	if(TimerCount==45) Sensor::askCO2();
	if(TimerCount==40) Sensor::getUnitTemperature();
	if(TimerCount==35) Sensor::getCO2();
	if(TimerCount==32) Sensor::getRTC();
	if(TimerCount==30) Sensor::getBME280Values();
	if(TimerCount==25) Sensor::askCO2Status();
	if(TimerCount==20) Sensor::getLightIntensity();
	if(TimerCount==18) Sensor::getADXL345Values();
	if(TimerCount==15) Sensor::getCO2Status();
	if(TimerCount== 8) Sensor::getRTC();
	if(TimerCount== 6) Sensor::getPM();
	if(TimerCount== 5) Sensor::getLocation();
	if(TimerCount== 2) Sensor::getExtTemperature();
	if(TimerCount== 1) Sensor::checkValues();
	
	if(TimerCount==43) Sensor::getAudioSpectrum();
	if(TimerCount==33) Sensor::getAudioSpectrum();
	if(TimerCount==23) Sensor::getAudioSpectrum();
	if(TimerCount==13) Sensor::getAudioSpectrum();
	if(TimerCount== 3) Sensor::getAudioSpectrum();

	/*	
	if(TimerCount== 1) Sensor::askExtTemperature();
	if(TimerCount== 2) Sensor::getPSUStatus();
	if(TimerCount== 5) Sensor::askCO2();
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
	if(TimerCount==49) Sensor::getExtTemperature();
	if(TimerCount==50) Sensor::checkValues();
	*/
    
/******************
* Second counters *
******************/
	if(OldSec!=RTC[SECONDS]) {
		if(GPS.Fix>1 && GPS.DateTimeUpdate==1){
			int32_t RTCSOD=(RTC[HOURS]*3600)+(RTC[MINUTES]*60)+RTC[SECONDS];
			int32_t GPSSOD=(GPS.DateTime[HOURS]*3600)+(GPS.DateTime[MINUTES]*60)+GPS.DateTime[SECONDS];
			int32_t DifSOD=GPSSOD-RTCSOD;
			if(DifSOD>2 || DifSOD<-2) {
				M41T00S.setData(GPS.DateTime);
				Sensor::getRTC();
				//Serial.println(F("RTC updated from GPS"));
			}
		}
		
	
		int8_t diff = RTC[SECONDS] - OldSec;
		if(diff<0) diff +=60;
		if(diff>5) diff=1;
		OldSec=RTC[SECONDS];
		SessionTimer+=diff;
		TotalUpTimer+=diff;
		if(CalibrationTimer<diff) CalibrationTimer=0; else CalibrationTimer-=diff;
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
		}
	}

/****************
* Visualisation *
****************/
	#define DCSMAX	100
	int16_t DCS=0;
	if(DemoModes==ENABLED) {
		Sensor::getADXL345Values();
		LEDDemo::cursor(ACC.Degr);
		DCS = (float)(ACC.XAxis * (DCSMAX-2));
		DCS=DCS<0?DCS*-1:DCS*1;	DCS=DCS*-1+DCSMAX;
	
		if(DemoCounter>=DCS) DemoCounter = 0; else DemoCounter+=1;
		if(DCS==DCSMAX && Demo<0x80 && Demo>0) Demo+=0x80;			// If Demo not yet confirmed, confirm it, with device upright

		if(DemoCounter==0){
			if(CursorPos>=5 && CursorPos<=10) {				// Right
				if(Demo>0x80) Demo-=0x80;					// If Demo is confirmed, deconfirm it
				if(Demo<DEMOMAX) Demo+=1; else Demo=0;
			}
			if(CursorPos>=21 && CursorPos<=26) {			// Left
				if(Demo>0x80) Demo-=0x80;					// If Demo is confirmed, deconfirm it
				if(Demo>0) Demo-=1; else Demo=DEMOMAX;
			}
			if(CursorPos>=13 && CursorPos<=18) {			// Down
				Demo=0x0;
			}
		}
	}

/***************
* Actual Demos *
***************/
	if((DemoModes==ENABLED && DCS==DCSMAX) || (DemoModes==DISABLED)) {
		uint8_t Brightness = 0x08;
		if(HardwareType==0x1603) if((PowerState&0x80)!=0) Brightness=0x0C;									// If on mains, high power is available
		if(HardwareType==0x1711) Brightness=0x0F;			

		if(Demo==0x81) Visualisation::showTemperature(HardwareType, Brightness, (uint16_t)Temperature);
		if(Demo==0x82) Visualisation::showHumidity(HardwareType, Brightness, (uint16_t)Humidity);
		if(Demo==0x83) Visualisation::showPressure(HardwareType, Brightness, (uint16_t)Pressure);
		if(Demo==0x84) Visualisation::showCO2(HardwareType, Brightness, CO2gas);
		if(Demo==0x85) Visualisation::showPM10(HardwareType, Brightness, PM10);
		if(Demo==0x86) Visualisation::showPM2_5(HardwareType, Brightness, PM2_5);
		if(Demo==0x87) Visualisation::showDemo(HardwareType, Brightness, LaEQPeak);
	}

	
/***************
* Sensor icons *
***************/
	if(Demo!=DemoVisual) {
		//DemoLED=0;DemoBackup=0;DemoActual=0;
		//Serial.print("Demo:0x"); Serial.println(Demo,HEX); 
		if(HardwareType==0x1711) DebugTimer=DEBUGTIMERMAX;
		
		IconRing.singleIcon(ICONTemperature,ICONOFF,0xFF,0xAF,0xAF, MEMORY);
		IconRing.singleIcon(ICONHumidity,ICONOFF,0xFF,0xAF,0xAF, MEMORY);
		IconRing.singleIcon(ICONPressure,ICONOFF,0xFF,0xAF,0xAF, MEMORY);
		IconRing.singleIcon(ICONCO2,ICONOFF,0xFF,0xAF,0xAF, MEMORY);
		IconRing.singleIcon(ICONPM10,ICONOFF,0xFF,0xAF,0xAF, MEMORY);
		IconRing.singleIcon(ICONPM2_5,ICONOFF,0xFF,0xAF,0xAF, MEMORY);
		IconRing.singleIcon(ICONPM1,ICONOFF,0xFF,0xAF,0xAF, MEMORY);
		IconRing.singleIcon(ICONLocation,ICONOFF,0x00,0xFF,0x00, MEMORY);

		switch(Demo) {
			case 0x01: IconRing.singleIcon(ICONTemperature,ICONHALF,0xFF,0xAF,0xAF, MEMORY); break;
			case 0x02: IconRing.singleIcon(ICONHumidity,ICONHALF,0xFF,0xAF,0xAF, MEMORY); break;
			case 0x03: IconRing.singleIcon(ICONPressure,ICONHALF,0xFF,0xAF,0xAF, MEMORY); break;
			case 0x04: IconRing.singleIcon(ICONCO2,ICONHALF,0xFF,0xAF,0xAF, MEMORY); break;
			case 0x05: IconRing.singleIcon(ICONPM10,ICONHALF,0xFF,0xAF,0xAF, MEMORY); break;
			case 0x06: IconRing.singleIcon(ICONPM2_5,ICONHALF,0xFF,0xAF,0xAF, MEMORY); break;
			case 0x07: IconRing.singleIcon(ICONPM1,ICONHALF,0xFF,0xAF,0xAF, MEMORY); break;
			case 0x80: LEDRing.rangeLED(0x00,0x20,0x00,0x00,0x00,0xFF,UPDATE); break;
			case 0x81: IconRing.singleIcon(ICONTemperature,ICONFULL,0xFF,0xAF,0xAF, MEMORY); break;
			case 0x82: IconRing.singleIcon(ICONHumidity,ICONFULL,0xFF,0xAF,0xAF, MEMORY); break;
			case 0x83: IconRing.singleIcon(ICONPressure,ICONFULL,0xFF,0xAF,0xAF, MEMORY); break;
			case 0x84: IconRing.singleIcon(ICONCO2,ICONFULL,0xFF,0xAF,0xAF, MEMORY); break;
			case 0x85: IconRing.singleIcon(ICONPM10,ICONFULL,0xFF,0xAF,0xAF, MEMORY); break;
			case 0x86: IconRing.singleIcon(ICONPM2_5,ICONFULL,0xFF,0xAF,0xAF, MEMORY); break;
			case 0x87: IconRing.singleIcon(ICONPM1,ICONFULL,0xFF,0xAF,0xAF, MEMORY); break;
			default:
				break;
		}
		DemoVisual=Demo;
		int8_t b=ICONFULL-3;
		if(b<1)b=1;
		if(Demo>0x80) {
			IconRing.singleIcon(ICONLocation,b,0x00,0xBF,0x00, MEMORY);
		}
		IconRing.updateNow();
		Visualisation::reset();
	}
	
	ConnectionState=Utils::checkConnections();
	//Serial.println(ConnectionState,HEX);
	if(HardwareType==0x1711) {
		if((PINJ&0x10)==0) DebugTimer=DEBUGTIMERMAX; 
		if(DebugTimer>0) DebugTimer--;
		if(DebugTimer==1) {
			Utils::disableDebugLEDs(false);
			Utils::disableLED(false);
			PCB150011.disableLED();
		} else if(DebugTimer==(DEBUGTIMERMAX-5)) {
			Utils::enableDebugLEDs(false);
			Utils::enableLED(false);
			PCB150011.enableLED();
		}
	}
	
	
	if(ErrorState==0 && ErrorStateVisual !=0) {
		PowerStateVisual++;		// Force update of LEDs after error handling
		ErrorStateVisual = 0;
	} 
	if(ErrorState==0) {													// If no errors
		/**********************
		* Communication icons *
		**********************/
		if((PowerState!=PowerStateVisual) || (ConnectionState!=ConnectionStateVisual)) {
			if(Demo<0x81 && HardwareType==0x1711) {
				if((ConnectionState & GPSFIX3)==0) IconRing.singleIcon(ICONLocation,12,0xFF,0x00,0x00, UPDATE);
				if((ConnectionState & GPSFIX3)==1) IconRing.singleIcon(ICONLocation,12,0xFF,0x3F,0x00, UPDATE);
				if((ConnectionState & GPSFIX3)==2) IconRing.singleIcon(ICONLocation,12,0xFF,0xBF,0x00, UPDATE);
				if((ConnectionState & GPSFIX3)==3) IconRing.singleIcon(ICONLocation,12,0xFF,0xAF,0xAF, UPDATE);
			}
			if((PowerState & 0x80)!=0) {
				IconRing.singleIcon(ICONStatus,12,0xFF,0xAF,0xAF,UPDATE);
			} else {
				IconRing.singleIcon(ICONStatus,12,0x00,0xBF,0x00,UPDATE);
			}
			if((PowerState & POWERWIFION) != 0) {
				if((ConnectionState & POWERWIFION) != 0) {
					IconRing.singleIcon(ICONWiFi,ICONFULL);
				} else {
					if(HardwareType!=0x1711) IconRing.singleIcon(ICONWiFi,ICONHALF); else IconRing.singleIcon(ICONWiFi,ICONOFF);
				}
			} else {
				IconRing.singleIcon(ICONWiFi,ICONOFF);
			}
			
			if((PowerState & POWERLORAON) != 0) {
				if(ConnectionState & POWERLORAON != 0) {
					IconRing.singleIcon(ICONLoRa,ICONFULL);
				} else {
					IconRing.singleIcon(ICONLoRa,ICONHALF);
				}
			} else {
				IconRing.singleIcon(ICONLoRa,ICONOFF);
			}

			if((PowerState & POWERMIRAON) != 0) {
				IconRing.singleIcon(ICONMiRa,ICONFULL);
			} else {
				IconRing.singleIcon(ICONMiRa,ICONOFF);
			}

			PowerStateVisual=PowerState;
			ConnectionStateVisual=ConnectionState;
		}
	} else if (ErrorState > 0) {
		DebugTimer=DEBUGTIMERMAX;
		Utils::enableLED(false);
		Utils::enableDebugLEDs(false);
		ErrorStateVisual=ErrorState;
		//Demo=0;
		if (TimerCount==(TIMERMAX/2)) {
			IconRing.allOff();
			
			
		} else if (TimerCount==TIMERMAX) {
			IconRing.singleIcon(ICONStatus,15,0xFF,0x00,0x00,MEMORY);
			if((ErrorState&ERRORSENSOR)!=0) IconRing.singleIcon(ICONLocation,15,0x00,0x00,0xFF,MEMORY);
			if((ErrorState&ERRORCONFIG)!=0) IconRing.singleIcon(ICONLocation,15,0xFF,0x00,0x00,MEMORY);
			if((ErrorState&ERRORPM)!=0) { 
				IconRing.singleIcon(ICONPM10,ICONFULL,MEMORY);
				IconRing.singleIcon(ICONPM2_5,ICONFULL,MEMORY);
			}
			if((ErrorState&ERROREXTTEMP)!=0) IconRing.singleIcon(ICONTemperature,ICONFULL,MEMORY);
			if((ErrorState&ERRORCO2)!=0) IconRing.singleIcon(ICONCO2,ICONFULL,MEMORY);
			if((ErrorState&ERRORAUDIO)!=0) IconRing.singleIcon(ICONAUDIO,ICONFULL,MEMORY);
			IconRing.updateNow();
		}
	}
	


/*************************
* Periodacally send data *
* Basetimer = 10*50=500  *
*************************/
	if(BaseTimer==0) return TimerCount;											// If Basetimer==0 then disabled
	if(TxDataCounter==0) TxDataCounter=BaseTimer; else TxDataCounter-=1;
	if(TxDataCounter<=PUSH_MAX) Comms::txSensor(IF_WiFi, PR_JOSE, TxDataCounter);
	if(TxDataCounter==0) PCB150011.clearThirdsSpectrum(LaEQ);
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
	* 5 : OUT :  1  :    : WiFi Request to Send / HW:1711 BTLE link
	* 6 : INP :  0  :    : WiFi Status			/ HW:1711 Ethernet link
	* 7 : INP :  1  :  / : WiFi Link Active		/ HW:1711 Wifi link
	************************************/
	if(HardwareType==0x1603) DDRD = 0x20; else DDRD = 0x00;
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
	* 4 : OUT :  0  : _/ : WiFi Wakeup			// HW:1711 debug button input
	* 5 : OUT :  1  :  / : WiFi Defaults
	* 6 : OUT :  0  :  / : WiFi Reset
	* 7 : OUT :  0  :    : Not used 
	************************************/
	if(HardwareType==0x1603) DDRJ = 0xFF; else DDRJ = 0xEF;
	if(HardwareType==0x1603) PORTJ = 0x24; else PORTJ = 0x34;
	
	/************************************
	* PortK - Development / debugport
	************************************/
	DDRK = 0x00;
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
    PCA9635.reset();                          				// Reset all PCA9635 by reset on ALL CALL address 0x70
    PCA9635.set_sleep(0x0);                   				// Wake oscillators on ALL CALL address 0x70
    delayMicroseconds(500);					  				// Let oscillators stabilize
    PCA9635.set_led_mode(3);     							// Enable PWM controls for all LEDs on ALL CALL address 0x70
	
	if(HardwareType==0x1603) {
		PCA9635.set_driver_mode(0);
		PCA9635.setMode(0x81,0x05);
	}
	if(HardwareType==0x1711) {
		PCA9635.set_driver_mode(1);
		PCA9635.setMode(0x81,0x15);
	}
	
	LEDRing.begin();
	IconRing.begin();
}

/***********************
* Inits I2C controller *
***********************/
void Utils::initI2C() {
	Utils::ctrlDebugLEDs(LEDI2C, ON);
	I2c.begin();
    I2c.timeOut(100);
	if(HardwareType==0x1603) {
		I2c.setSpeed(1);
		Serial.println(F("I2C hardware initialized @ 400kHz"));
	} else if(HardwareType==0x1711) {
		I2c.setSpeed(0);
		Serial.println(F("I2C hardware initialized @ 100kHz"));
	}

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
	uint8_t knownIntDev = 0;
	uint8_t knownExtDev = 0;
	uint8_t expIntDev = 0;
	uint8_t expExtDev = 0;
	if(HardwareType==0x1603) {
		if(HardwareRev==0xA10) expIntDev=17;
		if((HardwareRev==0xB10) || (HardwareRev==0xD10)) expIntDev=17;		// Todo
		if((HardwareRev==0xB20) || (HardwareRev==0xD20)) expIntDev=17;		// Todo
		if((HardwareRev==0xB30) || (HardwareRev==0xD30)) expIntDev=10;
		if((HardwareRev==0xB40) || (HardwareRev==0xD40)) expIntDev=17;		
		if((HardwareRev==0xB50) || (HardwareRev==0xD50)) expIntDev=17;		
		if((HardwareRev==0xB60) || (HardwareRev==0xD60)) expIntDev=17;	
	} else if(HardwareType==0x1711) {
		if(HardwareRev==0xA10) {expIntDev=13; expExtDev=0;}
		if(HardwareRev==0xC10) {expIntDev=13; expExtDev=0;}
		if((HardwareConfig&CONFIG_AUDIOPRESSURE)!=DISABLED) expExtDev++;
		if((HardwareConfig&CONFIG_POWERCONTROLLER)!=DISABLED) expExtDev++;
		if((HardwareConfig&CONFIG_LEDMATRICES)!=DISABLED) expExtDev+=6;

		PCA9635.reset();                          				// Reset all PCA9635 by reset on ALL CALL address 0x70	
		delay(200);
		
	}
	
	Serial.println(F("Scanning I2C devices, one moment"));
	for(uint8_t s = 0; s <= 0x7F; s++) {
		Utils::ctrlDebugLEDs(LEDI2C, ON);
		if(I2c.start()==0) ret = I2c.sendAddress(SLA_W(s));		
		Utils::ctrlDebugLEDs(LEDI2C, OFF);
		Utils::showI2CDevices(s,ret,HardwareType,HardwareConfig);

		if(ret!=0) {
			if(ret==1) {
				Serial.println(F("\nThere is a problem with the bus, could not complete scan"));
				ErrorState|=ERRORSENSOR;
				timeOutDelay = tempTime;
				return;
			}
		} else {
			totalDev++;
			switch (s) {
				case 0x00 : knownIntDev++; break;
				case 0x03 : knownIntDev++; break;
				case 0x10 :	if(HardwareType==0x1711) knownExtDev++; else knownIntDev++;	break;
				case 0x12 :	if(HardwareType==0x1711) knownExtDev++; else knownIntDev++;	break;
				case 0x13 :	if(HardwareType==0x1711) knownExtDev++; else knownIntDev++;	break;
				case 0x14 :	if(HardwareType==0x1711) knownExtDev++; else knownIntDev++;	break;
				case 0x15 :	if(HardwareType==0x1711) knownExtDev++; else knownIntDev++;	break;
				case 0x16 :	if(HardwareType==0x1711) knownExtDev++; else knownIntDev++;	break;
				case 0x17 :	if(HardwareType==0x1711) knownExtDev++; else knownIntDev++;	break;
				case 0x18 :	knownIntDev++; break;
				case 0x1A :	knownIntDev++; break;
				case 0x1D :	knownIntDev++; break;
				case 0x49 :	knownIntDev++; break;
				case 0x4D :	knownIntDev++; break;
				case 0x4E :	knownIntDev++; break;
				case 0x50 :	knownIntDev++; break;
				case 0x57 :	knownIntDev++; break;
				case 0x5C :	knownIntDev++; break;
				case 0x60 :	if(HardwareType==0x1711) knownExtDev++; else knownIntDev++;	break;
				case 0x68 :	knownIntDev++; break;
				case 0x70 :	/*knownIntDev++; */break;
				case 0x76 :	knownIntDev++; break;
				default : break;
			}				
		}
		I2c.stop();
	}
	Serial.print(F("I2C selftest "));

	if(knownIntDev==expIntDev) {
		Serial.print(F("finished, all ")); Serial.print(expIntDev) ; Serial.print(F(" internal devices are available"));
	}
	else if (knownIntDev<expIntDev && totalDev==expIntDev) {
		Serial.print(F("failed, addressing error on ")) ; Serial.print(expIntDev-knownIntDev) ; Serial.print(F(" device(s)"));
		ErrorState|=ERRORSENSOR;
	}
	else if (totalDev<expIntDev) {
		Serial.print(F("failed, missing ")); Serial.print(expIntDev-totalDev); Serial.print(F(" device(s)"));
		ErrorState|=ERRORSENSOR;
	} 
	else if (totalDev==0) {
		Serial.print(F("failed, no devices found, check PCA9516"));
		ErrorState|=ERRORSENSOR;
	}
	else {
		Serial.print(F("failed, some weird I2C problem occured (config wrong?), total: ")); Serial.print(totalDev) ; Serial.print(F(", known: ")); Serial.print(knownIntDev) ; Serial.print(F(", expected: ")); Serial.print(expIntDev) ;
		ErrorState|=ERRORSENSOR;
	}
	//Serial.println();
	if(expExtDev>0){
		if(knownExtDev==expExtDev){
			Serial.print(F(", all ")); Serial.print(expExtDev) ; Serial.print(F(" external devices are available"));
		} else if (knownExtDev<expExtDev){
			Serial.print(F(", missing ")); Serial.print(expExtDev-knownExtDev) ; Serial.print(F(" external device(s)"));
		} else if (knownExtDev>expExtDev){
			Serial.print(F(", found ")); Serial.print(knownExtDev) ; Serial.print(F(" external device(s), but expected ")); Serial.print(knownExtDev);
		}
	}

	Serial.println();
	timeOutDelay = tempTime;
}


/**************************
* Enables I2C hub PCA9516 *
**************************/
void Utils::enableI2C() {
	sbi(PORTE,7);			// HW:1603 ALL / HW:1711 3v3 devices
	sbi(PORTG,0);			// HW:1603 N/C / HW:1711 ext devices
	sbi(PORTG,1);			// HW:1603 N/C / HW:1711 opt devices
	sbi(PORTG,2);			// HW:1603 N/C / HW:1711 5v0 devices
	Serial.println(F("I2C hub is switched on"));
}

/***************************
* Disables I2C hub PCA9516 *
***************************/
void Utils::disableI2C() {
	cbi(PORTE,7);			// HW:1603 ALL / HW:1711 3v3 devices
	cbi(PORTG,0);			// HW:1603 N/C / HW:1711 5v0 devices
	cbi(PORTG,1);			// HW:1603 N/C / HW:1711 ext devices
	cbi(PORTG,2);			// HW:1603 N/C / HW:1711 opt devices
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
* Deasserts WiFi default line *
******************************/
void Utils::deAssertDefaultWiFi() {
	sbi(PORTJ, 5);			// Reset bit 5 on PORTJ
	Serial.println(F("WiFi defaults line is deasserted"));
}

/*******************************
* Enables WiFi module XPCW1002 *
*******************************/
void Utils::enableWiFi(boolean verbose) {
	if((HardwareCommConfig&CONFIG_WiFi)!=DISABLED) {
		if(HardwareType==0x1603) sbi(PORTJ, 6);			// Set bit 6 on PORTJ (XPICOWIFI)
		if(HardwareType==0x1711) cbi(PORTJ, 6);			// Reset bit 6 on PORTJ (ESP32)
		PowerState|=POWERWIFION;
		if(verbose==true) Serial.println(F("WiFi is switched on"));
	} else {
		if(HardwareType==0x1603) cbi(PORTJ, 6);			// Reset bit 6 on PORTJ (XPICOWIFI)
		if(HardwareType==0x1711) sbi(PORTJ, 6);			// Set bit 6 on PORTJ (ESP32)
		PowerState&=POWERWIFIOFF;
		if(verbose==true) Serial.println(F("WiFi is not installed"));
	}
	
}	

/********************************
* Disables WiFi module XPCW1002 *
********************************/
void Utils::disableWiFi(boolean verbose) {
	if(HardwareType==0x1603) cbi(PORTJ, 6);			// Set bit 6 on PORTJ (XPICOWIFI)
	if(HardwareType==0x1711) sbi(PORTJ, 6);			// Reset bit 6 on PORTJ (ESP32)
	PowerState&=POWERWIFIOFF;
	if((HardwareCommConfig&CONFIG_WiFi)!=DISABLED) {
		if(verbose==true) Serial.println(F("WiFi is switched off"));
	} else {
		if(verbose==true) Serial.println(F("WiFi is not installed"));
	}
}

/********************************
* Queries WiFi module XPCW1002 *
********************************/
int Utils::queryWiFi() {
	if(((PORTJ & 0x40)!=0) && (HardwareType==0x1603)) {	// If not 0 then enabled XPICOWIFI
		Utils::enableWiFi();
		return 1;
	} else {
		Utils::disableWiFi();
		return 0;
	}
	if(((PORTJ & 0x40)==0) && (HardwareType==0x1711)) {	// If 0 then enabled ESP32
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
void Utils::enableMiRa(boolean verbose) {
	if((HardwareCommConfig&CONFIG_MiRa)!=DISABLED) {
		sbi(PORTJ, 2);            // Set bit 2 on PORTJ
		PowerState|=POWERMIRAON;
		Serial3.begin(115200);          // Setup MiRa
		while (!Serial3) {
		}
		if(verbose==true) Serial.println(F("MiRa is switched on")); 
	} else {
		cbi(PORTJ, 2);            // Reset bit 2 on PORTJ
		PowerState&=POWERMIRAOFF;
		Serial3.end();          // Setup MiRa
		if(verbose==true) Serial.println(F("MiRa is not installed")); 
	}
}

/******************************
* Disables MiRa module *
******************************/
void Utils::disableMiRa(boolean verbose) {
	cbi(PORTJ, 2);            // Reset bit 2 on PORTJ
	PowerState&=POWERMIRAOFF;
	if((HardwareCommConfig&CONFIG_MiRa)!=DISABLED) {
		if(verbose==true) Serial.println(F("MiRa is switched off")); 
	} else {
		if(verbose==true) Serial.println(F("MiRa is not installed")); 
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
void Utils::enableLoRa(boolean verbose) {
	if((HardwareCommConfig&CONFIG_LoRa)!=DISABLED) {
		sbi(PORTH, 4);			// Set bit 4 on PORTH
		PowerState|=POWERLORAON;
		if(verbose==true) Serial.println(F("LoRa is switched on"));
	} else {
		cbi(PORTH, 4);			// Reset bit 4 on PORTH
		PowerState&=POWERLORAOFF;
		if(verbose==true) Serial.println(F("LoRa is not installed"));
	}
}

/******************************
* Disables LoRa module RN2483 *
******************************/
void Utils::disableLoRa(boolean verbose) {
	cbi(PORTH, 4);			// Reset bit 4 on PORTH
	PowerState&=POWERLORAOFF;
	if((HardwareCommConfig&CONFIG_LoRa)!=DISABLED) {
		if(verbose==true) Serial.println(F("LoRa is switched off"));
	} else {
		if(verbose==true) Serial.println(F("LoRa is not installed"));
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
	switch(GPS.Fix) {
		case 1 : r |= GPSFIX1; break;
		case 2 : r |= GPSFIX2; break;
		case 3 : r |= GPSFIX3; break;
	}

	if(HardwareType==0x1711) {
		if((PIND & 0x40) != 0) r |= POWERWIFION; else r &= POWERWIFIOFF;			// Eth connected
	} else if (HardwareType==0x1603) {
		if((PIND & 0x80) == 0) r |= POWERWIFION; else r &= POWERWIFIOFF;			// Wifi connected
	}
	if((PINE & 0x10) == 0) r |= USBDTRLOW; else r &= USBDTRHIGH;				// Terminal connected
	return r;
}

/********************
* Powers CO2 module *
********************/
void Utils::powerCO2(uint8_t state, boolean verbose) {
	ErrorTimer+=5;
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

/********************
* Powers GPS module *
********************/
void Utils::powerGPS(uint8_t state, boolean verbose) {
	if(state==ENABLED) {
		A2235H.powerOn();
		if(verbose==true) Serial.print(F("GPS receiver is switched on\n"));
	}
	else {
		A2235H.powerOff();
		if(verbose==true) Serial.print(F("GPS receiver is switched off\n"));
	}
}

/*******************
* Powers PM module *
*******************/
void Utils::powerPM(uint8_t state, boolean verbose) {
	if(state==ENABLED) {
		ErrorTimer+=5;
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
		ErrorTimer+=5;
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
	if(Length==2) Data = ((uint16_t)((EEPROM.read(Address))<<8) | EEPROM.read(Address+1));
	if(Length==4) Data = ((uint32_t)(EEPROM.read(Address))<<24 | (uint32_t)(EEPROM.read(Address+1))<<16 | (uint32_t)(EEPROM.read(Address+2))<<8 | EEPROM.read(Address+3));
	Utils::ctrlDebugLEDs(LEDMem, TOGGLE);
	return Data;
}

void Utils::loadAndApplyParameters() {
	//Serial.println((Utils::readEEPROM(MEM_HARDWARETYPE,2)),HEX);
	if((PINJ&0x10)==0) {
		Serial.println("Debug button pressed");
		Utils::defaultParameters(0x1711);
	}

	if(Utils::readEEPROM(MEM_HARDWARETYPE,2)==0xFFFF) {
		Utils::defaultParameters(0);
		Serial.println(F("Parameters defaults loaded!"));
	}
	HardwareType = Utils::readEEPROM(MEM_HARDWARETYPE,2);
	HardwareRev = Utils::readEEPROM(MEM_HARDWAREREV,2);
	HardwareConfig = Utils::readEEPROM(MEM_CONFIGURATION,4);

	
	if((HardwareType != 0x1603) && (HardwareType != 0x1711)) {
		Serial.println(F("*********************************** WARNING ***************************************"));
		Serial.println(F("* This Josuino is wrongly configured and will not function correctly until it is! *"));
		Serial.println(F("***********************************************************************************"));
		ErrorState|=ERRORCONFIG;
		HardwareCommConfig = CONFIG_USB;
	} else {
		Serial.print(F("PCB type "));
		Serial.print(HardwareType,HEX);
		Serial.print(F(", revision "));
		Serial.print(((HardwareRev & 0xFF00) >> 8),HEX);
		Serial.print(F(", variant "));
		Serial.println((HardwareRev & 0x00FF),HEX);
	}
	
	Serial.print(F("Installed communication options: "));
	if (HardwareType==0x1603) {
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
				HardwareCommConfig = CONFIG_USB;
				Serial.println(F("Variant unknown"));
				break;
		}
		if((HardwareCommConfig&CONFIG_USB)!=DISABLED) Serial.print(F("USB"));
		if((HardwareCommConfig&CONFIG_WiFi)!=DISABLED) Serial.print(F(", WiFi"));
		if((HardwareCommConfig&CONFIG_LoRa)!=DISABLED) Serial.print(F(", LoRa"));
		if((HardwareCommConfig&CONFIG_MiRa)!=DISABLED) Serial.print(F(", MiRa"));
	}
	else if (HardwareType==0x1711) {
		switch (HardwareRev) {
			case 0xA10 : HardwareCommConfig = CONFIG_USB + CONFIG_ESP + CONFIG_Pi + CONFIG_MiRa;
				break;
			case 0xC10 : HardwareCommConfig = CONFIG_USB + CONFIG_ESP;
				break;
			default :
				HardwareCommConfig = CONFIG_USB;
				Serial.println(F("Variant unknown"));
				break;
		}
		if((HardwareCommConfig&CONFIG_USB)!=DISABLED) Serial.print(F("USB"));
		if((HardwareCommConfig&CONFIG_WiFi)!=DISABLED) Serial.print(F(", ESP"));
		if((HardwareCommConfig&CONFIG_LoRa)!=DISABLED) Serial.print(F(", Pi"));
		if((HardwareCommConfig&CONFIG_MiRa)!=DISABLED) Serial.print(F(", MiRa"));
	} else {
		if((HardwareCommConfig&CONFIG_USB)!=DISABLED) Serial.print(F("USB"));
	}
	Serial.print(F("\n"));

	
	if((HardwareConfig&CONFIG_AUDIOPRESSURE)!=DISABLED) Serial.println(F("External soundpressure sensor enabled"));
	if((HardwareConfig&CONFIG_EXTTEMPSENSOR)!=DISABLED) Serial.println(F("External temperature sensor enabled"));
	if((HardwareConfig&CONFIG_POWERCONTROLLER)!=DISABLED) Serial.println(F("External powercontroller enabled"));
	if((HardwareConfig&CONFIG_LEDMATRICES)!=DISABLED && HardwareConfig==0x1603) Serial.println(F("LED ring enabled")); else Serial.println(F("LED matrices enabled"));
	

	UnitID[1] = 0;
	UnitID[0] = Utils::readEEPROM(MEM_ID,4);
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

void Utils::defaultParameters(uint16_t hw) {
	Serial.print(F("Loading default parameters"));
	if(hw!=0) {
		Serial.print(F("for hardwaretype: "));
		Serial.print(hw,HEX);
	}
	Serial.println();
	
	if(hw==0x1711){
		Serial.println(F("Release debug button"));
		while((PINJ&0x10)==0) {
			Utils::ctrlDebugLEDs(LEDUSB, TOGGLE);
			Utils::ctrlDebugLEDs(LEDLoRa, TOGGLE);
			delay(200);
		}
		
		Utils::writeEEPROM(MEM_HARDWARETYPE, 0x1711, 2);			// Default hardware type
		Utils::writeEEPROM(MEM_HARDWAREREV, 0x0C10, 2);				// Default hardware type
		Utils::writeEEPROM(MEM_CONFIGURATION,7,4);					// Default configuration
		Utils::writeEEPROM(MEM_UPTIME, 0, 4);						// Total uptime reset
		Utils::writeEEPROM(MEM_BASETIMER, 0x000A, 2);				// Basetimer reset
		Utils::writeEEPROM(MEM_DEMOENABLED, 0xFF, 1);				// Demomodes selectable by accelerometer
	} else {
		Utils::writeEEPROM(MEM_HARDWARETYPE, 0xFFFF, 2);			// Default hardware type
		Utils::writeEEPROM(MEM_HARDWAREREV, 0xFFFF, 2);				// Default hardware type
		Utils::writeEEPROM(MEM_CONFIGURATION,0,4);					// Default configuration
		Utils::writeEEPROM(MEM_UPTIME, 0, 4);						// Total uptime reset
		Utils::writeEEPROM(MEM_BASETIMER, 0xFFFF, 2);				// Basetimer reset
		Utils::writeEEPROM(MEM_DEMOENABLED, 0x00, 1);				// Demomodes selectable by accelerometer
	}

	Utils::writeEEPROM(MEM_DEBUGLEDS, 0xFF, 1);					// DebugLEDs and ringLEDs on
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

	// MiRa defaults
	Utils::writeEEPROM(MEM_MIRA_AESKEY_0, 0x4A, 1);				// AES Key byte 0x0

	Utils::writeEEPROM(MEM_MIRA_AESKEY_1, 0x65, 1);				// AES Key byte 0x1
	Utils::writeEEPROM(MEM_MIRA_AESKEY_2, 0x6E, 1);				// AES Key byte 0x2
	Utils::writeEEPROM(MEM_MIRA_AESKEY_3, 0x6E, 1);				// AES Key byte 0x3
	Utils::writeEEPROM(MEM_MIRA_AESKEY_4, 0x65, 1);				// AES Key byte 0x4
	Utils::writeEEPROM(MEM_MIRA_AESKEY_5, 0x56, 1);				// AES Key byte 0x5
	Utils::writeEEPROM(MEM_MIRA_AESKEY_6, 0x61, 1);				// AES Key byte 0x6
	Utils::writeEEPROM(MEM_MIRA_AESKEY_7, 0x6E, 1);				// AES Key byte 0x7
	Utils::writeEEPROM(MEM_MIRA_AESKEY_8, 0x44, 1);				// AES Key byte 0x8
	Utils::writeEEPROM(MEM_MIRA_AESKEY_9, 0x65, 1);				// AES Key byte 0x9
	Utils::writeEEPROM(MEM_MIRA_AESKEY_A, 0x6E, 1);				// AES Key byte 0xA
	Utils::writeEEPROM(MEM_MIRA_AESKEY_B, 0x42, 1);				// AES Key byte 0xB
	Utils::writeEEPROM(MEM_MIRA_AESKEY_C, 0x6F, 1);				// AES Key byte 0xC
	Utils::writeEEPROM(MEM_MIRA_AESKEY_D, 0x6F, 1);				// AES Key byte 0xD
	Utils::writeEEPROM(MEM_MIRA_AESKEY_E, 0x6D, 1);				// AES Key byte 0xE
	Utils::writeEEPROM(MEM_MIRA_AESKEY_F, 0x21, 1);				// AES Key byte 0xF
	
	Utils::writeEEPROM(MEM_MIRA_NWKKEY, 0x3B96, 2);			// NWK Key
	Utils::writeEEPROM(MEM_MIRA_ROOT, 0x01, 1);				// No root

	// Location reset
	Utils::writeEEPROM(MEM_LAT,0xFFFFFFFF,4);
	Utils::writeEEPROM(MEM_LON,0xFFFFFFFF,4);
}

/********************************
* Show verbose I2C device names *
********************************/
void Utils::showI2CDevices(uint8_t Address, uint8_t Found, uint16_t Hardware, uint32_t Config) {
	if((HardwareConfig&CONFIG_AUDIOPRESSURE)==DISABLED && Address==0x10) if(Found !=0) return; else Found = 2;					// Audio sensor on address 0x10, if not found, and not configured ignore
	if((HardwareConfig&CONFIG_POWERCONTROLLER)==DISABLED && Address==0x60) if(Found !=0) return; else Found = 2;				// Power controller on address 0x60
	if(((HardwareConfig&CONFIG_LEDMATRICES)==DISABLED) && (Address>=0x12 && Address <=0x17)) if(Found!=0) return; else Found=2; // Matrices on address 0x12 ~ 0x17 

	boolean nk = true;
	if(Hardware==0x1711 || Hardware==0xFFFF) {
		switch (Address) {
			case 0x4D :	Serial.print(F("Addr: 0x4D - SC16IS752 UART bridge A+B . .")); nk=false; break;
			case 0x57 :	Serial.print(F("Addr: 0x57 - 24AA025E64 EUI-64 . . . . . .")); nk=false; break;
			case 0x60 :	Serial.print(F("Addr: 0x60 - Intemo Powercontroller. . . .")); nk=false; break;
		}
	}

	if(Hardware==0x1603 || Hardware==0x1711 || Hardware==0xFFFF) {
		switch (Address) {
			case 0x00 :	Serial.print(F("Addr: 0x00 - I2C General Call. . . . . . .")); nk=false; break;
			case 0x03 :	Serial.print(F("Addr: 0x03 - PCA9635 General I2C Reset . .")); nk=false; break;
			case 0x10 :	Serial.print(F("Addr: 0x10 - Intemo Soundpressure sensor .")); nk=false; break;
			case 0x12 :	Serial.print(F("Addr: 0x12 - PCA9635 RGB segment 1/6 . . .")); nk=false; break;
			case 0x13 :	Serial.print(F("Addr: 0x13 - PCA9635 RGB segment 2/6 . . .")); nk=false; break;
			case 0x14 :	Serial.print(F("Addr: 0x14 - PCA9635 RGB segment 3/6 . . .")); nk=false; break;
			case 0x15 :	Serial.print(F("Addr: 0x15 - PCA9635 RGB segment 4/6 . . .")); nk=false; break;
			case 0x16 :	Serial.print(F("Addr: 0x16 - PCA9635 RGB segment 5/6 . . .")); nk=false; break;
			case 0x17 :	Serial.print(F("Addr: 0x17 - PCA9635 RGB segment 6/6 . . .")); nk=false; break;
			case 0x18 :	Serial.print(F("Addr: 0x18 - DS2482S 1wire bridge. . . . .")); nk=false; break;
			case 0x1A :	Serial.print(F("Addr: 0x1A - PCA9635 Icon LEDs . . . . . .")); nk=false; break;
			case 0x1D :	Serial.print(F("Addr: 0x1D - ADXL345 Accelerometer . . . .")); nk=false; break;
			case 0x49 :	Serial.print(F("Addr: 0x49 - TMP112 Temperature sensor . .")); nk=false; break;
			case 0x4E :	Serial.print(F("Addr: 0x4E - SC16IS752 UART bridge C+D . .")); nk=false; break;
			case 0x50 :	Serial.print(F("Addr: 0x50 - AT24C256C 32kx8 memory. . . .")); nk=false; break;
			case 0x5C :	Serial.print(F("Addr: 0x5C - BH1750FVI Light sensor. . . .")); nk=false; break;
			case 0x68 :	Serial.print(F("Addr: 0x68 - M41T00S Real time clock . . .")); nk=false; break;
			case 0x70 :	Serial.print(F("Addr: 0x70 - PCA9635 All Call address. . .")); nk=false; break;
			case 0x76 :	Serial.print(F("Addr: 0x76 - BME280 Temp, RH, hPa sensor .")); nk=false; break;
		}
	}
	
	if(nk==true) {
		if(Found==0) {
			Serial.print(F("Addr: 0x")); 
			if(Address<0x10) Serial.print(F("0"));
			Serial.print(Address,HEX);
			Serial.print(F(" - "));
			Serial.print(F("Unknown device\n")); 
		}
	} else {
		if(Found==0) Serial.println(F(" OK")); 
		else if(Found==2) Serial.println(F(" OK, config wrong!"));
		else Serial.println(F(" Not found!"));
	}
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
	Serial.println(F("Get AES       - Returns configured AES key for MiRa"));
	Serial.println(F("Get NWK       - Returns configured NWK key and isroot for MiRa"));
	Serial.println(F("->> Programming assist commands <<-"));
	Serial.println(F("DebugLEDs On  - Switch debugleds on"));
	Serial.println(F("DebugLEDs Off - Switch debugleds off"));
	Serial.println(F("->> LED commands <<-"));
	Serial.println(F("Show Temp     - Show temperature on LEDs"));
	Serial.println(F("Show Humidity - Show humidity on LEDs"));
	Serial.println(F("Show Pressure - Show pressure on LEDs"));
	Serial.println(F("Show Co2      - Show CO2 on LEDs"));
	Serial.println(F("Show Pm10     - Show PM10 on LEDs"));
	Serial.println(F("Show Pm2.5    - Show PM2.5 on LEDs"));
	Serial.println(F("Show Demo     - Show demo on LEDs"));
	Serial.println(F("->> Sensor value commands <<-"));
	Serial.println(F("Get All       - Returns all data"));
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
