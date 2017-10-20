/************************************************************************************
* Visualisation.h - Header for Visualisation 				                    	*
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
#ifndef Visualisation_h
#define Visualisation_h
#include <Arduino.h> 

class Visualisation 
{
    public:
        Visualisation();
		static void showTemperature(uint16_t hw, uint8_t br, uint16_t val);
		static void showHumidity(uint16_t hw, uint8_t br, uint16_t val);
		static void showPressure(uint16_t hw, uint8_t br, uint16_t val);
		static void showCO2(uint16_t hw, uint8_t br, uint16_t val);
		static void showPM10(uint16_t hw, uint8_t br, uint16_t val);
		static void showPM2_5(uint16_t hw, uint8_t br, uint16_t val);
		static void showDemo(uint16_t hw, uint8_t br, uint8_t pk);
		static void reset();
};

#endif
