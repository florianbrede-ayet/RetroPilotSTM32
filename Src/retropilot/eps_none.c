/*
  eps_none.c - Mock interface when no eps is installed
  
  The MIT License (MIT)

  Copyright (c) 2021 Florian Brede

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
*/

#include "core.h"
#include "retropilot/globals.h"
#include "retropilot/retropilot_params.h"
#include "retropilot/eps_none.h"
#include "retropilot/easy_transfer.h"
#include "retropilot/can_manager.h"
#include "uart.h"
#include "logger.h"

#if EPS_TYPE==EPS_TYPE_NONE


/* called at initialization */
void eps_setup()
{
}



void eps_debug_torque_left() {
    if (!retropilotParams.DEBUGMODE) return;
}

void eps_debug_torque_right() {
    if (!retropilotParams.DEBUGMODE) return;
}


/* called at > 1kHz */
void eps_loop()
{
  cm_last_recv_module_eps=millis();    
}

#endif