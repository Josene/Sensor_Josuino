/************************************************************************************
* Josuino_R1-00.ino - Main file for programming Josuino indoor sensor               *
* Copyright (c) 2014-2017 Antoine van de Cruyssen. All rights reserved              *
*************************************************************************************
* Rev 1.00 - February 2017                                                          *
* - Initial public release                                                          *
*************************************************************************************
* Rev 1.20 - April 2017                                                             *
* - Uses library 1.20                                                               *
*************************************************************************************
* Rev 1.30 - June 2017                                                              *
* - Mesh implementation                                                             *
*************************************************************************************
* Rev 1.40 - August 2017                                                            *
* - Added livinglab version                                                         *
************************************************************************************/

#include <I2C.h>
#include <I2C_Device.h>
#include <Josuino.h>

Comms Communication;
Utils Josuino;
LEDRing LEDRing;
IconRing IconRing;
Sensor Sensor;

uint8_t USBarr[100];
uint8_t USBcnt=0;
String USBdata;
String WiFidata;
String MiRadata;
boolean USBdataavail = false;
boolean USBISPavail = false;
boolean WiFidataavail = false;
boolean MiRadataavail = false;
boolean TMRflag = false;

void setup() {
  Josuino.begin();
  Josuino.enableWiFi();
  Josuino.disableLoRa();
  Josuino.disableMiRa();
  Josuino.finishUp();
}

void loop()
{
  if (USBdataavail == true) {
    Communication.parseData(IF_USB, USBdata);
    USBdata = "";
    USBdataavail = false;
  }
  if (USBISPavail == true) {
    Communication.parseISP(IF_USB, USBarr);
    USBISPavail = false;
  }
  if (WiFidataavail == true) {
    Communication.parseData(IF_WiFi, WiFidata);
    WiFidata = "";
    WiFidataavail = false;
  }
  if (MiRadataavail == true) {
    Communication.parseData(IF_MiRa, MiRadata);
    MiRadata = "";
    MiRadataavail = false;
  }
  if (TMRflag == true) {
    Josuino.timerIRQ();
    TMRflag = false;
  }
}

void serialEvent() {
    uint8_t ISPflag = 0;
    uint8_t ISPlen = 0;
    while (Serial.available()) {
      USBarr[USBcnt] = (uint8_t)Serial.read();
      USBcnt++;
    }

    //Serial.println(USBcnt);
    for(uint8_t i=0;i<USBcnt;i++){
        if((USBarr[i]==0x49)&&(USBarr[i+1]==0x53)&&(USBarr[i+2]==0x50)){      // Check for ISP
            ISPflag=i;
            if(USBcnt>16) ISPlen=USBarr[i+11]+13+4;
        }
        if(USBarr[i]==0x03) if(USBcnt>=14 && USBarr[i-13]==0x02) {            // Check for Josene
            for (uint8_t j=i-13;j<USBcnt;j++) USBdata += char(USBarr[j]);
            USBdataavail = true; USBcnt=0;
        }
        if(USBarr[i]==0x0A && ISPflag==0 && i>4) {                            // Check for Verbose
            for (uint8_t j=0;j<i;j++) USBdata += char(USBarr[j]);
            USBdataavail = true; USBcnt=0;        
        }
    }

    if(USBcnt>=((ISPflag-4)+ISPlen) && ISPflag!=0 && ISPlen!=0) {
        USBISPavail = true; USBcnt=0;        
    }
}
 

void serialEvent1() {
  while (Serial1.available()) {
    char WiFichar = (char)Serial1.read();
    WiFidata += WiFichar;
    if (WiFichar == char(0x02)) WiFidata = WiFichar;      // STX
    if (WiFichar == char(0x03)) WiFidataavail = true;     // ETX
    if (WiFichar == char(0x0A)) WiFidataavail = true;     // LF
  }
}

void serialEvent3() {
  while (Serial3.available()) {
    char MiRachar = (char)Serial3.read();
    MiRadata += MiRachar;
    if (MiRachar == char(0x02)) MiRadata = MiRachar;      // STX
    if (MiRachar == char(0x03)) MiRadataavail = true;     // ETX
    if (MiRachar == char(0x0A)) MiRadataavail = true;     // LF
  }
}

ISR(TIMER1_OVF_vect) {
  TMRflag = true;
  TCNT1 = 0xFB1E;            // preload timer
}


