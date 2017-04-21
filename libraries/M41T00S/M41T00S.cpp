/************************************************************************************
* M41T00S.cpp - Library for M41T00S real time clock                               	*
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

#include "M41T00S.h"

uint8_t M41T00Svalue[7];

// Constructor
M41T00S::M41T00S()
{

}

void M41T00S::begin(uint8_t Address, boolean TWI_Begin) {
    i2c_device::begin(Address, TWI_Begin);
}

uint8_t M41T00S::decToBcd(uint8_t val) {
  return ((val/10*16)+(val%10));
}

uint8_t M41T00S::bcdToDec(uint8_t val) {
  return((val/16*10)+(val%16));
}

void M41T00S::setData(uint8_t *d) {
	for (uint8_t x=0; x<=6; x++){
		d[x]=decToBcd(d[x]);
	}
	i2c_device::write_many(0x00, 7, d);
}

uint8_t *M41T00S::getData() {
	i2c_device::read_many(0x00, 7, M41T00Svalue);

	for (uint8_t x=0; x<=6; x++){
		M41T00Svalue[x]=bcdToDec(M41T00Svalue[x]);
	}

	return M41T00Svalue;
}