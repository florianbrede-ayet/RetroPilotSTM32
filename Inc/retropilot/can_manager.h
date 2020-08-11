/*
  can_manager.h - Handles sending and receiving of can messages between the different ECUs and OpenPilot, incl. fake messages for stock toyota parts
  
  The MIT License (MIT)

  Copyright (c) 2020 Florian Brede

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

#ifndef _RETROPILOTCANMANAGER_H_
#define _RETROPILOTCANMANAGER_H_

#include "../core.h"
#include "globals.h"



  extern int cm_rx_hist; // tracks the number of received can messages per second for debugging purposes
  extern int cm_tx_hist;
  
  extern unsigned long cm_last_recv;

  // we track can message timings for each module individually. if a module is enabled on the same ecu, the timer is just updated with every can send
  extern unsigned long cm_last_recv_module_inputs;    
  extern unsigned long cm_last_recv_module_vss;
  extern unsigned long cm_last_recv_module_athrottle;
  extern unsigned long cm_last_recv_module_abrake;
  extern unsigned long cm_last_recv_module_eps;

  extern unsigned long cm_last_recv_steer_cmd;
  extern unsigned long cm_last_recv_steer_angle;

  extern unsigned long cm_last_recv_module_error_flag; // updated whenever an ecu sends a heartbeat with a fault state

  void cm_setup();
  void cm_loop_recv();
  void cm_loop_send();


#endif