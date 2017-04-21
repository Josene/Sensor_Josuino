/************************************************************************************
* Josuino_R1-00.ino - Main file for programming Josuino indoor sensor               *
* Copyright (c) 2014-2017 Antoine van de Cruyssen. All rights reserved              *
*************************************************************************************
* Rev 1.00 - February 2017                                                          *
* - Initial public release                                                          *
*************************************************************************************
* Rev 1.20 - April 2017                                                             *
* - Uses library 1.20                                                               *
************************************************************************************/


#include <I2C.h>
#include <I2C_Device.h>
#include <Josuino.h>

Comms Communication;
Utils Josuino;
LEDRing LEDRing;
IconRing IconRing;
Sensor Sensor;

String USBdata;
String WiFidata;
boolean USBdataavail = false;
boolean WiFidataavail = false;
boolean TMRflag = false;

uint8_t Test = 0;

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
  if (WiFidataavail == true) {
    Communication.parseData(IF_WiFi, WiFidata);
    WiFidata = "";
    WiFidataavail = false;
  }
  if (TMRflag == true) {
    Test = Josuino.timerIRQ();
    TMRflag = false;
  }
}

void serialEvent() {
  while (Serial.available()) {
    char USBchar = (char)Serial.read();
    USBdata += USBchar;
    if (USBchar == char(0x03)) USBdataavail = true;       // ETX
    if (USBchar == char(0x0A)) USBdataavail = true;       // LF
  }
}

void serialEvent1() {
  while (Serial1.available()) {
    char WiFichar = (char)Serial1.read();
    WiFidata += WiFichar;
    if (WiFichar == char(0x03)) WiFidataavail = true;     // ETX
    if (WiFichar == char(0x0A)) WiFidataavail = true;     // LF
  }
}

ISR(TIMER1_OVF_vect) {
  TMRflag = true;
  TCNT1 = 0xFB1E;            // preload timer
}



