/*
  inputs.c - Handling user button inputs (gra on/off, set speed etc.)
  
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

#include "retropilot/globals.h"
#include "retropilot/retropilot_params.h"
#include "retropilot/inputs.h"
#include "retropilot/actuator_throttle.h"
#include "retropilot/actuator_brake.h"
#if EPS_TYPE == EPS_TYPE_TORQUE
#include "retropilot/eps_torque.h"
#elif EPS_TYPE == EPS_TYPE_POSITION
#include "retropilot/eps_position.h"
#endif
#include "core.h"

int buttonstate1;
int lastbuttonstate1;
unsigned long debounceTime1 = 0;

int buttonstate2;
int lastbuttonstate2;
unsigned long debounceTime2 = 0;

int buttonstate3;
int lastbuttonstate3;
unsigned long debounceTime3 = 0;

int buttonstate4;
int lastbuttonstate4;
unsigned long debounceTime4 = 0;


uint8_t ccLastSetSpeed = 90;


unsigned long lastClutchPressedTime = 0L;
unsigned long lastBrakePressedTime = 0L;


uint8_t debug_actuator = 0;


/* called at initialization */
void inputs_setup() {
    buttonstate1 = digitalRead(BUTTON_1_PORT, BUTTON_1_PIN);
    buttonstate2 = digitalRead(BUTTON_2_PORT, BUTTON_2_PIN);
    buttonstate3 = digitalRead(BUTTON_3_PORT, BUTTON_3_PIN);
    buttonstate4 = digitalRead(BUTTON_4_PORT, BUTTON_4_PIN);
    
    if (!buttonstate2 && !buttonstate3 && buttonstate1) {
        retropilotParams.DEBUGMODE=true;

        #if !MODULE_THROTTLE_ACTUATOR || !MODULE_BRAKE_ACTUATOR || !MODULE_EPS
        logger_e("DEBUG MODE ACTIVATED WITHOUT REQUIRED MODULES ON THIS ECU (ACTUATOR THROTTLE, ACTUATOR BRAKE, EPS)!\n");
        while(1) {} // deadlock to trigger IWDG reset
        #endif
    }
    else if (!buttonstate2 && !buttonstate3 && !buttonstate1) {
        retropilotParams.NO_CLUCH_BRAKE_MODE=true;
    }

    ccLastSetSpeed = DEFAULT_SET_SPEED;
}

/* called at > 1kHz */
void inputs_loop() {
    if (!retropilotParams.NO_CLUCH_BRAKE_MODE) {
        int stat = digitalRead(CLUTCH_CANCEL_PORT, CLUTCH_CANCEL_PIN);
        if (stat) {
            lastClutchPressedTime = millis();
        }
        retropilotParams.OP_CLUTCH_PRESSED = stat || millis()-lastClutchPressedTime<CLUTCH_RELEASE_GRACE_TIME_MS;

        stat = digitalRead(BRAKE_CANCEL_PORT, BRAKE_CANCEL_PIN);
        if (stat) {
            lastBrakePressedTime = millis();
        }
        retropilotParams.OP_BRAKE_PRESSED = stat || millis()-lastBrakePressedTime<BRAKE_RELEASE_GRACE_TIME_MS;
    }
    else {
        retropilotParams.OP_CLUTCH_PRESSED = false;
        retropilotParams.OP_BRAKE_PRESSED = false;
    }
    
    // READING BUTTONS AND SWITCHES
    buttonstate1 = digitalRead(BUTTON_1_PORT, BUTTON_1_PIN);
    buttonstate2 = digitalRead(BUTTON_2_PORT, BUTTON_2_PIN);
    buttonstate3 = digitalRead(BUTTON_3_PORT, BUTTON_3_PIN);
    buttonstate4 = digitalRead(BUTTON_4_PORT, BUTTON_4_PIN);
    
    if (buttonstate1 != lastbuttonstate1) {
        debounceTime1=millis();
    }
    if (buttonstate2 != lastbuttonstate2) {
        debounceTime2=millis();
    }
    if (buttonstate3 != lastbuttonstate3) {
        debounceTime3=millis();
    }
    if (buttonstate4 != lastbuttonstate4) {
        debounceTime4=millis();
    }

    lastbuttonstate1 = buttonstate1;
    lastbuttonstate2 = buttonstate2;
    lastbuttonstate3 = buttonstate3;
    lastbuttonstate4 = buttonstate4;


    if (buttonstate1==LOW && debounceTime1!=0 && (millis()-debounceTime1>=50L || millis()<debounceTime1)) {
        debounceTime1=0;
        if (retropilotParams.OP_ON == true)
        {
            retropilotParams.OP_ON = false;
        }
        else if(retropilotParams.OP_ON == false && !retropilotParams.DEBUGMODE)
        {
            retropilotParams.OP_ON = true;
            retropilotParams.ccSetSpeed = ccLastSetSpeed;
            retropilotParams.OP_LKAS_ENABLED = DEFAULT_LKAS_STATE;
        }

        if (retropilotParams.DEBUGMODE) {
            debug_actuator = debug_actuator >= 2 ? 0 : debug_actuator+1;
        }
    }


    #if DEBUG_SIMULATE_STEERING_ANGLE_SENSOR
    if (buttonstate2==LOW && buttonstate3==LOW) {
        if (debounceTime2!=0 || debounceTime3!=0) {
            debounceTime2=0;
            debounceTime3=0;
            eps_debug_zero_simulated_steering_angle();
        }
    }
    #endif

    if (buttonstate2==LOW && debounceTime2!=0 && (millis()-debounceTime2>=50L || millis()<debounceTime2)) {
        debounceTime2=0;
        if (!retropilotParams.DEBUGMODE && retropilotParams.ccSetSpeed>30) {
            retropilotParams.ccSetSpeed -= 5;
            ccLastSetSpeed=retropilotParams.ccSetSpeed;
        }

        if (retropilotParams.DEBUGMODE) {
            if (debug_actuator==0) {
                athrottle_debug_down();
            } else if (debug_actuator==1) {
                abrake_debug_down();
            } else if (debug_actuator==2) {
                eps_debug_torque_left();
            }
        }
    }

    

    if (buttonstate3==LOW && debounceTime3!=0 && (millis()-debounceTime3>=50L || millis()<debounceTime3)) {
        debounceTime3=0;
        if (!retropilotParams.DEBUGMODE && retropilotParams.ccSetSpeed<150) {
            retropilotParams.ccSetSpeed += 5;
            ccLastSetSpeed=retropilotParams.ccSetSpeed;
        }

        if (retropilotParams.DEBUGMODE) {
            if (debug_actuator==0) {
                athrottle_debug_up();
            } else if (debug_actuator==1) {
                abrake_debug_up();
            } else if (debug_actuator==2) {
                eps_debug_torque_right();
            }
        }
    }

    if (buttonstate4==LOW && debounceTime4!=0 && (millis()-debounceTime4>=50L || millis()<debounceTime4)) {
        debounceTime4=0;
        if (!retropilotParams.DEBUGMODE && retropilotParams.OP_ON) {
            retropilotParams.OP_LKAS_ENABLED = !retropilotParams.OP_LKAS_ENABLED;
        }
    }

    /* if (retropilotParams.DEBUGMODE && buttonstate3==LOW) {
        startActuation(actuatorTargetPosition+50);
        delay(10);
    } */
}