/************************************************************************************
* i2c_device.h - Library for I2C communication        	                        	*
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
// safety againts double-include
#ifndef I2C_Device_h
#define I2C_Device_h
#include <Arduino.h> 
// Defined here for now due to a problem with scope
//#define I2C_DEVICE_DEBUG
#include <I2C.h> 						// Get it from http://github.com/rambo/I2C

class i2c_device
{
    public:
        void begin(byte dev_addr, boolean wire_begin);

        // A Very shorthand helper for reading single byte (NOTE: does not do error-checking!)
        byte read(byte address);
        // Read single byte to a referred target (calls read_many internally)
        boolean read(byte address, byte *target);
        // Read N bytes to a target (usually an array)
        boolean read_many(byte address, byte num, byte *target);

        // Added AvdC
        boolean write_byte(byte value);
        boolean read_woreg(byte num, byte *target);
		
		
        // Write N values from a source (usually an array)
        boolean write(byte address, byte value);

        // Write N values from a source (usually an array)
        boolean write_many(byte address, byte num, void *source);
        // Do a masked read/modify/write operation to an address (defaults to ORing the value)
        boolean read_modify_write(byte address, byte mask, byte value);
        // Do a masked read/modify/write operation to an address
        boolean read_modify_write(byte address, byte mask, byte value, byte operand);
        // Helper to debug state, dumps given register values
        void dump_registers(byte addr_start, byte addr_end);

    protected:
        byte device_address;
};


#endif
// *********** END OF CODE **********
