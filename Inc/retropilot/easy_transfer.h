/*
  easy_transfer.h - Simply library for binary package (up to 255 bytes) transfer over U(S)ART, ported for STM32
  Copyright (c) 2020 Florian Brede
*
*  Ported from: EasyTransfer Arduino Library
*			http://www.billporter.info/easytransfer-arduino-library/
*		Brought to you by:
*              Bill Porter
*              www.billporter.info
*This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
<http://www.gnu.org/licenses/>
*
*This work is licensed under the Creative Commons Attribution-ShareAlike 3.0 Unported License.
*To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/3.0/ or
*send a letter to Creative Commons, 444 Castro Street, Suite 900, Mountain View, California, 94041, USA.
******************************************************************/

#ifndef _EASYTRANSFER_H_
#define _EASYTRANSFER_H_

#include "uart.h"

void et_begin(uint8_t *, uint8_t, uart_ring *q);
void et_send_data();
bool et_receive_data();

#endif
