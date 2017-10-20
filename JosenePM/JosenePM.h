/************************************************************************************
* JosenePM.h - Header for JosenePM PM sensor		          	                   	*
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
#ifndef JosenePM_h
#define JosenePM_h
#include <Arduino.h> 
#include <SC16IS752.h>

class JosenePM : public SC16IS752
{
    public:
        JosenePM();
        void begin(uint8_t UARTI2CAddress, uint8_t UARTChannel);
		uint16_t * getData(boolean dbg=false);
		boolean checkPower();
		void powerOn();
		void powerOff();
};

#endif
