/*
  actuator_brake.h - This module is controlling a brake actuator through 2 h-bridges (servo + solenoid / clutch) and verifies its position through a potentiometer
  
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

#ifndef _RETROPILOTACTUATORBRAKE_H_
#define _RETROPILOTACTUATORBRAKE_H_

#include "../core.h"

// NOTE: THIS IS THE CONFIGURATION FOR A VDO TEMPOSTAT
#define BRAKE_ACTUATOR_ALLOWED_PERM_ERROR 25 //will allow a difference between targetPressure and currentPressure, otherwise the actuator permanently jerks around
#define BRAKE_ACTUATOR_MIN_POT 20 //measured at actuators lowest position (the initial pulling of the brake pedal ~1000ohms only activates the braking light switch and has no real effect)
#define BRAKE_ACTUATOR_MAX_POT 2500 //measured at actuators highest position (ot the maximum actuation length we want to allow)
#define BRAKE_ACTUATOR_POTI_REFERENCE_RESISTOR 1000 // we measure the resistance of the potentiometer - this is the reference resistor used to cover the ~2.4k ohms potentiometer range

#define BRAKE_ACTUATOR_RAMP_UP_MS -1 // the "pull" time for which duty cycling is active (-1 disables ramping)
#define BRAKE_ACTUATOR_RAMP_DOWN_MS -1 // the "loosen" time for which duty cycling is active (-1 disables ramping)

#define BRAKE_ACTUATOR_DUTY_CYCLE_LENGTH_MS 20 // each complete cycle is 20 ms
#define BRAKE_ACTUATOR_RAMP_UP_INITIAL_DUTY 10 // pull initial duty in % 
#define BRAKE_ACTUATOR_RAMP_DOWN_INITIAL_DUTY 10 // loosen initial duty in %
#define BRAKE_ACTUATOR_RAMP_UP_PER_CYCLE_DUTY 4 // pull duty-per-cycle increment after each completed cycle in absolute percent
#define BRAKE_ACTUATOR_RAMP_DOWN_PER_CYCLE_DUTY 5 // loosen duty-per-cycle increment after each completed cycle in absolute percent

void abrake_setup();
void abrake_loop();
void abrake_debug_up();
void abrake_debug_down();

#endif