/*
  eps_torque.h - Controlling a closed loop stepper over USART3 (misfittech nano stepper with custom firmware).
  Parses the torque commands from the standard toyota eps LKA_STEERING messages

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
#define ANGLE_SYNC_STEP_FREQUENCY 100


#define MIN_TORQUE_THRESHOLD 50.0f    // OP torque commands below this threshold will be ignored (because our eps cannot control torque as fine as the original eps)
#define MAX_COMMANDED_TORQUE 1500.0f   // we clip any OP torque command above this our max. torque 

// the currents applied to the motor for dynamic and hold. note: hold requires less current than dynamic
#define MIN_HOLD_CURRENT 50.0f
#define MIN_DYNAMIC_CURRENT 50.0f
#define MAX_HOLD_CURRENT 200.0f
#define MAX_DYNAMIC_CURRENT 1000.0f   // 1500ma - we limit this to 1000 max because anything around 900+ constantly might require cooling of the driver

#define TOYOTA_TORQUE_TO_CURRENT_FACTOR 1.0f // we scale the toyota torque by this factor and convert it to stepper current

#define VSS_DEPENDANT_MIN_TORQUE_PERCENT 30.0f      // this is the minimum amount (%) of torque applied (at standstill))
#define VSS_DEPENDANT_MAX_TORQUE_SPEED_KMH 100.0f   // the maximum torque is applied from 100km/h+

#define REDUCE_UNWINDING_TORQUE 0                   // if set to true, we reduce the unwinding torque (since the axle geometry will push the wheels back itself)
                                                    // ATT: depending on the OP implementation, it might already account for this so it's not enabled by default
#define DYNAMIC_CURRENT_UNWINDING_DIVISOR 3.0f      // when unwinding the steering wheel, the dynamic torque will be divided by this factor
#define HOLD_CURRENT_UNWINDING_DIVISOR 1.0f         // when unwinding the steering wheel, the hold torque will be divided by this factor

#define STEERING_ANGLE_SOFT_LIMITATION_0KMH 20.0f   // steering wheel angle from which on the dynamic torque will be reduced by 10% per additional degree at 0 km/h
#define STEERING_ANGLE_SOFT_LIMITATION_100KMH 12.0f // steering wheel angle from which on the dynamic torque will be reduced by 10% per additional degree at 100 km/h
/* #define STEERING_ANGLE_SOFT_LIMITATION_0KMH 45.0f   // steering wheel angle from which on the dynamic torque will be reduced by 10% per additional degree at 0 km/h
#define STEERING_ANGLE_SOFT_LIMITATION_100KMH 45.0f // steering wheel angle from which on the dynamic torque will be reduced by 10% per additional degree at 100 km/h
 */

#define EPS_MAX_ANGLE_ERROR 18.0f                   // the maximum allowed divergence between stepper & angle sensor before the EPS goes into an error state
#define EPS_GEARING 41.0f                        // test gearing for bigger actuation ranges
//#define EPS_GEARING 17.0f                        // the total gearing between the stepper motor and the steering shaft (measured)

#define EPS_STEPPER_RPM   EPS_GEARING/6.0f*20.0f    // rpm = 0 is torque driven positional mode, otherwise "torque driven, positional, speed limited"
                                                    // this results in 20 deg movement per second

#define STEERING_ANGLE_INVERTED -1.0f               // set to 1.0 or -1.0 (-1.0 if the stepper angle is inverted to the actual steering angle, which should be true for most setups)


extern int8_t eps_debug_torque_percent;
extern bool eps_stepper_available;                  // set to true if the stepper has no error and is in available state. this is not dependant on temporary stepper error, it's meant to be used internally to check if calibration can progress

void eps_debug_torque_left();
void eps_debug_torque_right();
void eps_debug_zero_simulated_steering_angle();

void eps_setup();
void eps_loop();

#endif