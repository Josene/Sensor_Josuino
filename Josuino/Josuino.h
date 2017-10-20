/************************************************************************************
* Josuino.h - Library for controlling Arduino Based Jose indoor sensor           	*
* Copyright (c) 2014-2016 Antoine van de Cruyssen. All rights reserved             	*
*************************************************************************************
* Rev 1.0 - August 2016																*
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

/*****************************************************************************************************************
* Set fuses: Ext 0xFD, High 0x90, Low 0xFF, Memlock 0x0F														 *
* Burn bootloader:																								 *
* C:\Program Files (x86)\Arduino\hardware\arduino\avr\bootloaders\stk500v2\Mega2560-prod-firmware-2011-06-29.hex *
*****************************************************************************************************************/


#ifndef Josuino_h
#define Josuino_h

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif 

#include "Arduino.h"

#include <I2C.h> 
#include <I2C_Device.h>
#include <EEPROM.h>
#include <PCA9635.h>
#include <TMP112.h>
#include <BME280.h>
#include <BH1750.h>
#include <ADXL345.h>
#include <SC16IS752.h>
#include <T6713.h>
#include <JosenePM.h>
#include <M41T00S.h>
#include <DS2482S.h>
#include <PCB150011.h>
#include <A2235H.h>
#include <Powercontroller.h>
#include <AA025E64.h>
#include <stdarg.h>
#include <LEDRing.h>
#include <IconRing.h>
#include <avr/pgmspace.h>
#include <Visualisation.h>

#define LEDTmr		0x00
#define LEDI2C		0x01
#define LEDMem		0x02
#define LEDAsy		0x03
#define LEDWiFi		0x04
#define LEDMiRa		0x05
#define LEDLoRa		0x06
#define LEDUSB		0x07
#define OFF			0x00
#define ON			0x01

#define UINT8_T		0x01
#define UINT16_T	0x02
#define UINT32_T	0x04
#define BYTE   		0x01
#define INT    		0x02
#define LONG    	0x04
#define RD			false
#define WR			true

#define CMDUNKOWN	0x00
#define CMDSILENT	0x01
#define CMDDONE		0x02

#define FORWARD 	0xFD
#define BACKWARD 	0xFE
#define ALL 		0xFF

#define CYCLIC		0x08
#define ENABLED		0x01
#define DISABLED	0x00

#define IF_USB		0x07
#define IF_WiFi		0x04
#define IF_LoRa		0x06
#define IF_MiRa		0x05

#define PR_JOSE		0x10
#define PR_VERBOSE	0x11

#define SECONDS 	0x00
#define MINUTES  	0x01
#define HOURS  		0x02
#define DAY  		0x03
#define DATE  		0x04
#define MONTH  		0x05
#define YEAR	  	0x06

#define CUR 	0
#define MAX 	31
#define MIN 	62
#define AVG 	93

#define ICONCO2					0x00
#define ICONPM10				0x01
#define ICONPM2_5				0x02
#define ICONPM1					0x03
#define ICONAUDIO				0x03
#define ICONLoRa				0x04
#define ICONPi  				0x04
#define ICONLocation			0x05
#define ICONStatus				0x06
#define ICONMiRa				0x07
#define ICONWiFi				0x08
#define ICONTemperature			0x09
#define ICONHumidity			0x0A
#define ICONPressure			0x0B

#define ICONOFF					0x00
#define ICONFULL				0x0F
#define ICONHALF				0x0A
#define DEBUGTIMERMAX			15000 	// Devide by 50 for seconds

#define ERRORCONNECTION			0x00000001
#define ERRORMEMORY				0x00000002
#define ERRORCONFIG				0x00000004
#define ERRORPM					0x00000100
#define ERRORCO2				0x00000200
#define ERRORAUDIO				0x00000400
#define ERROREXTTEMP			0x00000800

#define ERRORSENSOR				0x20000000
#define ERRORBASEIRQ			0x40000000
#define ERRORBOOTING			0x80000000

#define AUDIOBAND13				0		//20Hz
#define AUDIOBAND14				1		//25Hz
#define AUDIOBAND15				2		//31.5Hz
#define AUDIOBAND16				3		//40Hz
#define AUDIOBAND17				4
#define AUDIOBAND18				5
#define AUDIOBAND19				6
#define AUDIOBAND20				7
#define AUDIOBAND21				8
#define AUDIOBAND22				9
#define AUDIOBAND23				10
#define AUDIOBAND24				11
#define AUDIOBAND25				12
#define AUDIOBAND26				13
#define AUDIOBAND27				14
#define AUDIOBAND28				15
#define AUDIOBAND29				16
#define AUDIOBAND30				17
#define AUDIOBAND31				18
#define AUDIOBAND32				19
#define AUDIOBAND33				20
#define AUDIOBAND34				21
#define AUDIOBAND35				22
#define AUDIOBAND36				23
#define AUDIOBAND37				24
#define AUDIOBAND38				25
#define AUDIOBAND39				26
#define AUDIOBAND40				27
#define AUDIOBAND41				28
#define AUDIOBAND42				29
#define AUDIOBAND43				30		//20kHz

#define MEM_HARDWARETYPE		0x0000
#define MEM_HARDWAREREV			0x0002
#define MEM_CONFIGURATION		0x0004
#define MEM_ID					0x0008
#define MEM_UPTIME				0x000C
#define MEM_BASETIMER			0x0012
#define MEM_DEBUGLEDS			0x0020
#define MEM_DEMOENABLED			0x0021
#define MEM_CO2BOOTMODE			0x0025
#define MEM_PMBOOTMODE			0x0028

#define MEM_CALTIMER			0x0031
#define MEM_AMPFACTOR			0x0036
#define MEM_LAT					0x02F0
#define MEM_LON					0x02F4
#define MEM_PMPOWERINGUP		0x0230
#define MEM_PMPOWEREDUP			0x0232
#define MEM_PMPOWERINGDOWN		0x0234
#define MEM_PMPOWEREDDOWN		0x0236

#define MEM_MIRA_AESKEY_0		0x02C0
#define MEM_MIRA_AESKEY_1		0x02C1
#define MEM_MIRA_AESKEY_2		0x02C2
#define MEM_MIRA_AESKEY_3		0x02C3
#define MEM_MIRA_AESKEY_4		0x02C4
#define MEM_MIRA_AESKEY_5		0x02C5
#define MEM_MIRA_AESKEY_6		0x02C6
#define MEM_MIRA_AESKEY_7		0x02C7
#define MEM_MIRA_AESKEY_8		0x02C8
#define MEM_MIRA_AESKEY_9		0x02C9
#define MEM_MIRA_AESKEY_A		0x02CA
#define MEM_MIRA_AESKEY_B		0x02CB
#define MEM_MIRA_AESKEY_C		0x02CC
#define MEM_MIRA_AESKEY_D		0x02CD
#define MEM_MIRA_AESKEY_E		0x02CE
#define MEM_MIRA_AESKEY_F		0x02CF
#define MEM_MIRA_NWKKEY			0x02D0
#define MEM_MIRA_ROOT  			0x02D2

#define CONFIG_AUDIOPRESSURE	0x00000001
#define CONFIG_EXTTEMPSENSOR 	0x00000002
#define CONFIG_POWERCONTROLLER 	0x00000004
#define CONFIG_LEDMATRICES     	0x00000008

#define CONFIG_USB				0x01		// bit pattern for hardwarecommunication option
#define CONFIG_WiFi				0x02		// bit pattern for hardwarecommunication option
#define CONFIG_ESP				0x02		// bit pattern for hardwarecommunication option
#define CONFIG_LoRa				0x04		// bit pattern for hardwarecommunication option
#define CONFIG_MiRa				0x08		// bit pattern for hardwarecommunication option
#define CONFIG_Pi  				0x04		// bit pattern for hardwarecommunication option

#define POWEREDDOWN				3
#define POWERINGUP				0
#define POWEREDUP				1
#define POWERINGDOWN			2
#define POWERMODE				1
#define MEASMODE				0

#define POWERWIFION				0x00020000		// 0000.0000 0000.0010 0000.0000 0000.0000
#define POWERWIFIOFF			0xFFFDFFFF
#define POWERLORAON				0x00040000		// 0000.0000 0000.0100 0000.0000 0000.0000
#define POWERLORAOFF			0xFFFBFFFF
#define POWERMIRAON				0x00080000		// 0000.0000 0000.1000 0000.0000 0000.0000
#define POWERMIRAOFF			0xFFF7FFFF
#define POWERCO2ON				0x00000800		// 0000.0000 0000.0000 0000.1000 0000.0000
#define POWERCO2OFF				0xFFFFF7FF
#define POWERPMON				0x00004000		// 0000.0000 0000.0010 0100.0000 0000.0000
#define POWERPMOFF				0xFFFFBFFF
#define USBDTRLOW				0x00000400		// 0000.0000 0000.0010 0000.0100 0000.0000
#define USBDTRHIGH				0xFFFFFBFF

#define GPSFIX0   				0xFFFFFFFC		// 1111.1111 1111.1111 1111.1111 1111.1100
#define GPSFIX1  				0x00000001		// 0000.0000 0000.0000 0000.0000 0000.0001
#define GPSFIX2  				0x00000002		// 0000.0000 0000.0000 0000.0000 0000.0010
#define GPSFIX3  				0x00000003		// 0000.0000 0000.0000 0000.0000 0000.0011

#define TIMERMAX 				50
#define PUSH_MAX				0x49

#define SENSOR_ID				0x48
#define SENSOR_ID2				0x47
#define SENSOR_SESSIONTIMER		0x46
#define SENSOR_TOTALUPTIMER		0x45
#define SENSOR_ERRORSTATE		0x44
#define SENSOR_BATTERYTEMP		0x43
#define SENSOR_COULOMBCOUNT		0x42
#define SENSOR_CALTIMER			0x41
#define SENSOR_POWERSTATE		0x40
#define SENSOR_BATTERYVOLTAGE	0x3F
#define SENSOR_PSUVOLTAGE		0x3E
#define SENSOR_BATTERYCURRENT	0x3D
#define SENSOR_PSUCURRENT		0x3C
#define SENSOR_DATE				0x3B
#define SENSOR_TIME				0x3A
#define SENSOR_UNITTEMPERATURE  0x39
#define SENSOR_EXTTEMPERATURE	0x38
#define SENSOR_TEMPERATURE 		0x37
#define SENSOR_HUMIDITY			0x36
#define SENSOR_PRESSURE			0x35
#define SENSOR_LIGHTINTENSITY	0x34
#define SENSOR_CO2				0x33
#define SENSOR_PM10				0x32
#define SENSOR_PM2_5			0x31
#define SENSOR_PM1				0x30
#define SENSOR_ACCELEROX		0x2F
#define SENSOR_ACCELEROY		0x2E
#define SENSOR_DEVICEANGLE		0x2D
#define SENSOR_DOPFIXSIV		0x2C
#define SENSOR_LATITUDE			0x2B
#define SENSOR_LONGITUDE		0x2A

#define SENSOR_AUDIO_OCT0_CUR	0x29
#define SENSOR_AUDIO_OCT0_MAX	0x28
#define SENSOR_AUDIO_OCT0_MIN	0x27
#define SENSOR_AUDIO_OCT0_AVG	0x26
#define SENSOR_AUDIO_OCT1_CUR	0x25
#define SENSOR_AUDIO_OCT1_MAX	0x24
#define SENSOR_AUDIO_OCT1_MIN	0x23
#define SENSOR_AUDIO_OCT1_AVG	0x22
#define SENSOR_AUDIO_OCT2_CUR	0x21
#define SENSOR_AUDIO_OCT2_MAX	0x20
#define SENSOR_AUDIO_OCT2_MIN	0x1F
#define SENSOR_AUDIO_OCT2_AVG	0x1E
#define SENSOR_AUDIO_OCT3_CUR	0x1D
#define SENSOR_AUDIO_OCT3_MAX	0x1C
#define SENSOR_AUDIO_OCT3_MIN	0x1B
#define SENSOR_AUDIO_OCT3_AVG	0x1A
#define SENSOR_AUDIO_OCT4_CUR	0x19
#define SENSOR_AUDIO_OCT4_MAX	0x18
#define SENSOR_AUDIO_OCT4_MIN	0x17
#define SENSOR_AUDIO_OCT4_AVG	0x16
#define SENSOR_AUDIO_OCT5_CUR	0x15
#define SENSOR_AUDIO_OCT5_MAX	0x14
#define SENSOR_AUDIO_OCT5_MIN	0x13
#define SENSOR_AUDIO_OCT5_AVG	0x12
#define SENSOR_AUDIO_OCT6_CUR	0x11
#define SENSOR_AUDIO_OCT6_MAX	0x10
#define SENSOR_AUDIO_OCT6_MIN	0x0F
#define SENSOR_AUDIO_OCT6_AVG	0x0E
#define SENSOR_AUDIO_OCT7_CUR	0x0D
#define SENSOR_AUDIO_OCT7_MAX	0x0C
#define SENSOR_AUDIO_OCT7_MIN	0x0B
#define SENSOR_AUDIO_OCT7_AVG	0x0A
#define SENSOR_AUDIO_OCT8_CUR	0x09
#define SENSOR_AUDIO_OCT8_MAX	0x08
#define SENSOR_AUDIO_OCT8_MIN	0x07
#define SENSOR_AUDIO_OCT8_AVG	0x06
#define SENSOR_AUDIO_OCT9_CUR	0x05
#define SENSOR_AUDIO_OCT9_MAX	0x04
#define SENSOR_AUDIO_OCT9_MIN	0x03
#define SENSOR_AUDIO_OCT9_AVG	0x02

#define SENSOR_EOM				0x01

#define EXTREQ_CTRL				0x5000

#define EXTREQ_ID2				0x50A1
#define EXTREQ_ID				0x50A0
#define EXTREQ_BATTERYTEMP		0x5002
#define EXTREQ_COULOMBCOUNT		0x5003
#define EXTREQ_ERRORSTATE		0x500C
#define EXTREQ_SESSIONTIMER		0x500E
#define EXTREQ_TOTALUPTIMER		0x500F
#define EXTREQ_CALTIMER			0x500A
#define EXTREQ_CO2UTILS			0x5013
#define EXTREQ_NWKKEY			0x50DF
#define EXTREQ_AESKEY0			0x50E0
#define EXTREQ_AESKEY1			0x50E1
#define EXTREQ_AESKEY2			0x50E2
#define EXTREQ_AESKEY3			0x50E3
#define EXTREQ_RESET			0x50EE
#define EXTREQ_ALL  			0x50FF
#define EXTREQ_POWERSTATE		0x5000
#define EXTREQ_BATTERYVOLTAGE	0x5001
#define EXTREQ_PSUVOLTAGE		0x5005
#define EXTREQ_BATTERYCURRENT	0x5006
#define EXTREQ_PSUCURRENT		0x5007
#define EXTREQ_SENSOR			0x5300

#define EXTREQ_TIME				0x530E
#define EXTREQ_DATE				0x530F
#define EXTREQ_UNITTEMPERATURE  0x5311
#define EXTREQ_TEMPERATURE 		0x5312
#define EXTREQ_HUMIDITY			0x5313
#define EXTREQ_PRESSURE			0x5316
#define EXTREQ_LIGHTINTENSITY	0x5314
#define EXTREQ_CO2				0x5323
#define EXTREQ_EXTTEMPERATURE 	0x5325
#define EXTREQ_PM10				0x5329
#define EXTREQ_PM2_5			0x532A
#define EXTREQ_PM1				0x532B
#define EXTREQ_ACCELEROX		0x531A
#define EXTREQ_ACCELEROY		0x531B
#define EXTREQ_ACCELEROZ		0x531C
#define EXTREQ_DEVICEANGLE		0x532F

#define EXTREQ_AUDIO_OCT0_CUR	0x5335
#define EXTREQ_AUDIO_OCT1_CUR	0x5336
#define EXTREQ_AUDIO_OCT2_CUR	0x5337
#define EXTREQ_AUDIO_OCT3_CUR	0x5338
#define EXTREQ_AUDIO_OCT4_CUR	0x5339
#define EXTREQ_AUDIO_OCT5_CUR	0x533A
#define EXTREQ_AUDIO_OCT6_CUR	0x533B
#define EXTREQ_AUDIO_OCT7_CUR	0x533C
#define EXTREQ_AUDIO_OCT8_CUR	0x533D
#define EXTREQ_AUDIO_OCT9_CUR	0x533E

#define EXTREQ_DOPFIXSIV		0x5340
#define EXTREQ_LATITUDE			0x5341
#define EXTREQ_LONGITUDE		0x5342

#define SP2_TARGET_ID_SCLL_JOS	1
#define SP2_TARGET_ID_SCLL_SPL  2
#define SP2_TARGET_ID_SCLL_PWR  3
#define SP2_TARGET_ID_SCLL_ESP  4


//efine TXTIMERMAX				250


const PROGMEM char S35[] = "%>S3500%H%H%H&%<"; 
const PROGMEM char T35[] = "%>T3500%H%H%H&%<"; 
const PROGMEM char U35[] = "%>U3500%H%H%H&%<"; 
const PROGMEM char V35[] = "%>V3500%H%H%H&%<"; 
const PROGMEM char S36[] = "%>S3600%H%H%H&%<"; 
const PROGMEM char T36[] = "%>T3600%H%H%H&%<"; 
const PROGMEM char U36[] = "%>U3600%H%H%H&%<"; 
const PROGMEM char V36[] = "%>V3600%H%H%H&%<"; 
const PROGMEM char S37[] = "%>S3700%H%H%H&%<"; 
const PROGMEM char T37[] = "%>T3700%H%H%H&%<"; 
const PROGMEM char U37[] = "%>U3700%H%H%H&%<"; 
const PROGMEM char V37[] = "%>V3700%H%H%H&%<"; 
const PROGMEM char S38[] = "%>S3800%H%H%H&%<"; 
const PROGMEM char T38[] = "%>T3800%H%H%H&%<"; 
const PROGMEM char U38[] = "%>U3800%H%H%H&%<"; 
const PROGMEM char V38[] = "%>V3800%H%H%H&%<"; 
const PROGMEM char S39[] = "%>S3900%H%H%H&%<"; 
const PROGMEM char T39[] = "%>T3900%H%H%H&%<"; 
const PROGMEM char U39[] = "%>U3900%H%H%H&%<"; 
const PROGMEM char V39[] = "%>V3900%H%H%H&%<"; 
const PROGMEM char S3A[] = "%>S3A00%H%H%H&%<"; 
const PROGMEM char T3A[] = "%>T3A00%H%H%H&%<"; 
const PROGMEM char U3A[] = "%>U3A00%H%H%H&%<"; 
const PROGMEM char V3A[] = "%>V3A00%H%H%H&%<"; 
const PROGMEM char S3B[] = "%>S3B00%H%H%H&%<"; 
const PROGMEM char T3B[] = "%>T3B00%H%H%H&%<"; 
const PROGMEM char U3B[] = "%>U3B00%H%H%H&%<"; 
const PROGMEM char V3B[] = "%>V3B00%H%H%H&%<"; 
const PROGMEM char S3C[] = "%>S3C00%H%H%H&%<"; 
const PROGMEM char T3C[] = "%>T3C00%H%H%H&%<"; 
const PROGMEM char U3C[] = "%>U3C00%H%H%H&%<"; 
const PROGMEM char V3C[] = "%>V3C00%H%H%H&%<"; 
const PROGMEM char S3D[] = "%>S3D00%H%H%H&%<"; 
const PROGMEM char T3D[] = "%>T3D00%H%H%H&%<"; 
const PROGMEM char U3D[] = "%>U3D00%H%H%H&%<"; 
const PROGMEM char V3D[] = "%>V3D00%H%H%H&%<"; 
const PROGMEM char S3E[] = "%>S3E00%H%H%H&%<"; 
const PROGMEM char T3E[] = "%>T3E00%H%H%H&%<"; 
const PROGMEM char U3E[] = "%>U3E00%H%H%H&%<"; 
const PROGMEM char V3E[] = "%>V3E00%H%H%H&%<"; 

const char* const JStr[] PROGMEM = {S35,T35,U35,V35,S36,T36,U36,V36,S37,T37,U37,V37,S38,T38,U38,V38,S39,T39,U39,V39,
									S3A,T3A,U3A,V3A,S3B,T3B,U3B,V3B,S3C,T3C,U3C,V3C,S3D,T3D,U3D,V3D,S3E,T3E,U3E,V3E};

#define J_S35 	0
#define J_T35 	1
#define J_U35 	2
#define J_V35 	3
#define J_S36 	4
#define J_T36 	5
#define J_U36 	6
#define J_V36 	7
#define J_S37 	8
#define J_T37 	9
#define J_U37 	10
#define J_V37 	11
#define J_S38 	12
#define J_T38 	13
#define J_U38 	14
#define J_V38 	15
#define J_S39 	16
#define J_T39 	17
#define J_U39 	18
#define J_V39 	19
#define J_S3A 	20
#define J_T3A 	21
#define J_U3A 	22
#define J_V3A 	23
#define J_S3B 	24
#define J_T3B 	25
#define J_U3B 	26
#define J_V3B 	27
#define J_S3C 	28
#define J_T3C 	29
#define J_U3C 	30
#define J_V3C 	31
#define J_S3D 	32
#define J_T3D 	33
#define J_U3D 	34
#define J_V3D 	35
#define J_S3E 	36
#define J_T3E 	37
#define J_U3E 	38
#define J_V3E 	39

class Comms
{
	public:
		Comms();
		void parseData(uint8_t port, String& cmd); 
		void parseISP(uint8_t port, uint8_t *d);
		static void checkISP(uint8_t dest, uint8_t *d);
		static int parseVerboseProtocol(uint8_t port, String& cmd);
		static int parseJoseProtocol(uint8_t port, String& cmd);
		static void txSensor(uint8_t port, uint8_t protocol, uint16_t sns); 
		static void txMemory(uint8_t port, uint32_t data, uint8_t length, boolean rw);
		static int txData(uint8_t port, char const *str, ...);
		static void txHex32(uint8_t port, uint32_t n);
		static void txHex8(uint8_t port, uint8_t n);
		static uint8_t arrToUint8(uint8_t *d, uint8_t s=0);
		static uint32_t arrToUint32(uint8_t *d, uint8_t s=0);
		static void txDow(uint8_t port, uint8_t n);
		static void txIF(uint8_t port, uint8_t n);
	private:
};

class Sensor
{
	public:
		Sensor();
		static void begin();
		static void getUnitTemperature();
		static void getPSUStatus();
		static void askExtTemperature();
		static void getExtTemperature();
		static void getBME280Values();
		static void getLightIntensity();
		static void getADXL345Values();
		static void askGPS();
		static void getGPS();
		static void askCO2();
		static void getCO2();
		static void askCO2Status();
		static void getCO2Status();
		static void getPM();
		static void askCO2Calibration();
		static void getLocation();
		static void getRTC();
		static void getAudioSpectrum();
		static void checkValues();
	private:
};

class LEDDemo
{
	public:
		LEDDemo();
		static void showPressure(uint8_t mode=0);
		static void showTemperature(uint8_t mode=0);
		static void showHumidity(uint8_t mode=0);
		static void cursor(float Angle);
};


class Utils
{
	public:
		Utils();
		void begin();
		void finishUp();
		static void enableI2C();
		static void disableI2C();
		static void enableLED(boolean verbose=true);
		static void disableLED(boolean verbose=true);
		static void enableDebugLEDs(boolean verbose=true);
		static void disableDebugLEDs(boolean verbose=true);
		static void enableDemos(boolean verbose=true);
		static void disableDemos(boolean verbose=true);
		static int queryWiFi();
		static void enableWiFi(boolean verbose=true);
		static void disableWiFi(boolean verbose=true);
		static void enableLoRa(boolean verbose=true);
		static void disableLoRa(boolean verbose=true);
		static int queryMiRa();
		static void enableMiRa(boolean verbose=true);
		static void disableMiRa(boolean verbose=true);
		static void powerCO2(uint8_t state, boolean verbose=true);
		static void powerGPS(uint8_t state, boolean verbose=true);
		static void powerPM(uint8_t state, boolean verbose=true);
		static void ctrlDebugLEDs(byte mode, byte ctrl);
		static void setupIO();
		static void loadAndApplyParameters();
		static void defaultParameters(uint16_t hw);
		static void initLEDcontrollers();
		static void checkI2C();
		static void initI2C();
		static void checkUSB();
		static uint32_t checkConnections();
		static void assertDefaultWiFi();
		static void deAssertDefaultWiFi();
		static int queryLoRa();
		static void showHelp();
		static void showI2CDevices(uint8_t Address, uint8_t Found, uint16_t Hardware=0xFFFF, uint32_t Config=0);
		static void setupTimers();
		static uint8_t timerIRQ(int Value=-1);
		static uint32_t writeEEPROM(uint16_t Address, uint32_t Data, uint8_t Length=4);
		static uint32_t readEEPROM(uint16_t Address, uint8_t Length=4);
		
	private:
};

#endif
