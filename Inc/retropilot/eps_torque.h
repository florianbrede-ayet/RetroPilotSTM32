/*
  RetropilotVSS.cpp - Handling VSS sensor interrupts, calculating the current wheelspeed and storing it in RetropilotParams
  
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

#ifndef _RETROPILOTEPSTORQUE_H_
#define _RETROPILOTEPSTORQUE_H_


#define MIN_SERIAL_TX_FREQ 100 // this module normally just answers serial messages from the stepper (sent at 50 hz). this is the minimum frequency we send (stop messages) at if the stepper does not respond
#define MIN_CALCULATION_FREQ 10 // we want to update the stepper command calculations at 100+ Hz
#define ANGLE_SYNC_STEP_FREQUENCY 25


#define MIN_TORQUE_THRESHOLD 100.0f    // OP torque commands below this threshold will be ignored (because our eps cannot control torque as fine as the original eps)
#define MAX_COMMANDED_TORQUE 1500.0f   // we clip any OP torque command above this our max. torque 

// the currents applied to the motor for dynamic and hold. note: hold requires less current than dynamic
#define MIN_HOLD_CURRENT 300.0f
#define MIN_DYNAMIC_CURRENT 500.0f
#define MAX_HOLD_CURRENT 1100.0f
#define MAX_DYNAMIC_CURRENT 1500.0f

#define VSS_DEPENDANT_MIN_TORQUE_PERCENT 30.0f      // this is the minimum amount (%) of torque applied (at standstill))
#define VSS_DEPENDANT_MAX_TORQUE_SPEED_KMH 100.0f   // the maximum torque is applied from 100km/h+

#define REDUCE_UNWINDING_TORQUE 0                   // if set to true, we reduce the unwinding torque (since the axle geometry will push the wheels back itself)
                                                    // ATT: depending on the OP implementation, it might already account for this so it's not enabled by default
#define DYNAMIC_CURRENT_UNWINDING_DIVISOR 3.0f      // when unwinding the steering wheel, the dynamic torque will be divided by this factor
#define HOLD_CURRENT_UNWINDING_DIVISOR 1.0f         // when unwinding the steering wheel, the hold torque will be divided by this factor

#define STEERING_ANGLE_SOFT_LIMITATION_0KMH 20.0f   // steering wheel angle from which on the dynamic torque will be reduced by 10% per additional degree at 0 km/h
#define STEERING_ANGLE_SOFT_LIMITATION_100KMH 12.0f // steering wheel angle from which on the dynamic torque will be reduced by 10% per additional degree at 100 km/h


#define EPS_GEARING 220.0f                          // the total gearing between the stepper motor and the steering shaft * microsteps (13.74f*16.0f)



extern int8_t eps_debug_torque_percent;
extern bool eps_stepper_available;                  // set to true if the stepper has no error and is in available state. this is not dependant on temporary stepper error, it's meant to be used internally to check if calibration can progress

void eps_debug_torque_left();
void eps_debug_torque_right();
void eps_setup();
void eps_loop();

#endif