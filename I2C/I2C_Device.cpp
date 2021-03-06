/************************************************************************************
* i2c_device.cpp - Library for I2C communication                                	*
* Copyright (c) 2014-2016 Antoine van de Cruyssen. All rights reserved             	*
*************************************************************************************
* Added functionality of original library written by Eero af Heurlin				*
* http://github.com/rambo															*
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
#include "I2C_Device.h"
//#define I2C_DEVICE_DEBUG


void i2c_device::begin(byte dev_addr, boolean wire_begin)
{
    device_address = dev_addr;
    if (wire_begin)
    {
        I2c.begin();
    }
}

// TODO: Add read and write methods that take pointer to an iterator function

// TODO: Re-write the simplified methods to take advantage of the features of the I2C library
// TODO: Re-write to check for status codes from the I2C library on each send.

boolean i2c_device::read(byte address, byte *target)
{
    return this->read_many(address, 1, target);
}

byte i2c_device::read(byte address)
{
    byte target;
    this->read_many(address, 1, &target);
    return target;
}

boolean i2c_device::read_many(byte address, byte req_num, byte *target)
{
    byte result = I2c.read(device_address, address, req_num, target);
    //if((device_address>=0x12)&&(device_address<=0x17)) {
	//Serial.print(F("Dev 0x"));
	//Serial.print(device_address, HEX);
    //Serial.println(F(" "));
	//}
    if (result > 0)
    {
#ifdef I2C_DEVICE_DEBUG
        Serial.print(F("DEBUG: Read "));
        Serial.print(req_num, DEC);
        Serial.print(F(" bytes from dev 0x"));
        Serial.print(device_address, HEX);
        Serial.print(F(" reg 0x"));
        Serial.print(address, HEX);
        Serial.print(F(" failed, I2c.read returned: "));
        Serial.println(result, DEC);
#endif
        return false;
    }
    return true;
}



/****************************************************
 * Added AvdC 										*
 * Write a single byte without subaddress			*
 ***************************************************/
boolean i2c_device::write_byte(byte value) {
    byte result = I2c.write(device_address, value);
    if (result > 0) {
#ifdef I2C_DEVICE_DEBUG
        Serial.print(F("DEBUG: Writing value 0x "));
        Serial.print(value, HEX);
        Serial.print(F(" bytes to dev 0x"));
        Serial.print(device_address, HEX);
//        Serial.print(F(" reg 0x"));
//        Serial.print(address, HEX);
        Serial.print(F(" failed, I2c.read returned: "));
        Serial.println(result, DEC);
#endif
        return false;
    }
    return true;
}

/****************************************************
 * Added AvdC 										*
 * Read number of bytes without subaddress			*
 ***************************************************/
boolean i2c_device::read_woreg(byte num, byte *target) {
    byte result = I2c.read(device_address, num, target);
    if (result > 0) {
#ifdef I2C_DEVICE_DEBUG
        Serial.print(F("DEBUG: Read "));
        Serial.print(num, DEC);
        Serial.print(F(" bytes from dev 0x"));
        Serial.print(device_address, HEX);
//      Serial.print(F(" reg 0x"));
//      Serial.print(address, HEX);
        Serial.print(F(" failed, I2c.read returned: "));
        Serial.println(result, DEC);
#endif
        return false;
    }
    return true;
}






/**
 * Write a single byte and check the result
 */
boolean i2c_device::write(byte address, byte value)
{
    byte result = I2c.write(device_address, address, value);
    if (result > 0)
    {
#ifdef I2C_DEVICE_DEBUG
        Serial.print(F("DEBUG: Writing value 0x "));
        Serial.print(value, HEX);
        Serial.print(F(" bytes to dev 0x"));
        Serial.print(device_address, HEX);
        Serial.print(F(" reg 0x"));
        Serial.print(address, HEX);
        Serial.print(F(" failed, I2c.read returned: "));
        Serial.println(result, DEC);
#endif
        return false;
    }
    return true;
}

/**
 * Write multiple bytes and check result
 */
boolean i2c_device::write_many(byte address, byte num, void *source)
{
    byte result = I2c.write(device_address, address, (byte*) source, num);
    if (result > 0)
    {
#ifdef I2C_DEVICE_DEBUG
        Serial.print(F("DEBUG: Write "));
        Serial.print(num, DEC);
        Serial.print(F(" bytes to dev 0x"));
        Serial.print(device_address, HEX);
        Serial.print(F(" reg 0x"));
        Serial.print(address, HEX);
        Serial.print(F(" failed, I2c.read returned: "));
        Serial.println(result, DEC);
#endif
        return false;
    }
    return true;
}

/**
 * Performs a masked read-write-modify -cycle, remember that your mask should have 1 on the bits not to modify and 0 on the bits to modify
 * 
 * Operands:
 * 0=OR
 * 1=AND
 * 2=XOR
 *
 */
boolean i2c_device::read_modify_write(byte address, byte mask, byte value, byte operand)
{
    byte tmp;
    if (!this->read_many(address, 1, &tmp))
    {
        return false;
    }
/*
#ifdef I2C_DEVICE_DEBUG
    Serial.print(F("dev 0x"));
    Serial.print(device_address, HEX);
    Serial.print(F(" BEFORE: reg 0x"));
    Serial.print(address, HEX);
    Serial.print(F(" value: 0x"));
    Serial.print(tmp, HEX);
    Serial.print(F("\tB"));
    Serial.println(tmp, BIN);
    Serial.print(F("MASK: B"));
    Serial.print(mask, BIN);
    Serial.print(F("\tVALUE: B"));
    Serial.println(value, BIN);
#endif
*/
    // TODO: These need a re-think, basically: how to set the masked bits to the values in the value byte
    switch (operand)
    {
        case 0:
            tmp = (tmp & mask) | value;
            break;
        case 1:
            tmp = (tmp & mask) & value;
            break;
        case 2:
            tmp = (tmp & mask) ^ value;
            break;

    }
/*
#ifdef I2C_DEVICE_DEBUG
    Serial.print(F("dev 0x"));
    Serial.print(device_address, HEX);
    Serial.print(F(" AFTER: reg 0x"));
    Serial.print(address, HEX);
    Serial.print(F(" value: 0x"));
    Serial.print(tmp, HEX);
    Serial.print(F("\tB"));
    Serial.println(tmp, BIN);
#endif
*/
    return this->write_many(address, 1, &tmp);
}

boolean i2c_device::read_modify_write(byte address, byte mask, byte value)
{
    return this->read_modify_write(address, mask, value, 0);
}

/**
 * This does the reg dumping the naive way just in case the
 * usually supposed-to-be-present register autoincrement does not work as
 * expected
 */
void i2c_device::dump_registers(byte addr_start, byte addr_end)
{
    byte tmp;
    for (byte addr = addr_start; addr <= addr_end; addr++)
    {
        if (!i2c_device::read(addr, &tmp))
        {
            continue;
        }
        Serial.print(F("dev 0x"));
        Serial.print(device_address, HEX);
        Serial.print(F(" reg 0x"));
        Serial.print(addr, HEX);
        Serial.print(F(" value: 0x"));
        Serial.print(tmp, HEX);
        Serial.print(F("\tB"));
        Serial.println(tmp, BIN);
    }
}





