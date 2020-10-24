/*
  actuator_throttle.c - This module is controlling a throttle actuator through 2 h-bridges (servo + solenoid / clutch) and verifies its position through a potentiometer
  
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
#include "retropilot/actuator_throttle.h"


  unsigned long at_last_actuation_start = 0;
  unsigned long at_last_poti_position_update = 0;
  
  int at_actuator_target_position = 0;
  int at_actuator_poti_position = 0;

  int at_actuator_direction=0;
  unsigned long at_actuator_duty_cycle_start=0;


/* called at initialization */
void athrottle_setup()
{
    digitalWrite(THROTTLE_ACTUATOR_M_IN1_PORT, THROTTLE_ACTUATOR_M_IN1_PIN, LOW);
    digitalWrite(THROTTLE_ACTUATOR_M_IN2_PORT, THROTTLE_ACTUATOR_M_IN2_PIN, LOW);
    digitalWrite(THROTTLE_ACTUATOR_S_IN3_PORT, THROTTLE_ACTUATOR_S_IN3_PIN, LOW);
    digitalWrite(THROTTLE_ACTUATOR_S_IN4_PORT, THROTTLE_ACTUATOR_S_IN4_PIN, LOW);
    digitalWrite(THROTTLE_ACTUATOR_M_ENA_PORT, THROTTLE_ACTUATOR_M_ENA_PIN, LOW);
    digitalWrite(THROTTLE_ACTUATOR_M_ENB_PORT, THROTTLE_ACTUATOR_M_ENB_PIN, LOW);
}


int athrottle_read_poti() {
  int potRaw = adcRead(THROTTLE_ACTUATOR_POTI_PIN);
  if(potRaw>=0) 
  {
    float buffer=potRaw * 3.3f;
    float Vout = (buffer)/4096.0; // 12 bit precision
    if (Vout==0)
      buffer=4096;
    else
      buffer = (3.3f/Vout) - 1;
    return (int)(THROTTLE_ACTUATOR_POTI_REFERENCE_RESISTOR * buffer);
  }
  return 0;
}




/**
 * This is called whenever a setpoint is reached.
 * Does NOT disconnect the solenoid!
 * */
void athrottle_stop() {
    digitalWrite(THROTTLE_ACTUATOR_M_ENA_PORT, THROTTLE_ACTUATOR_M_ENA_PIN, 0);  //stop Motor
}

/**
 * This is called ~ 50/s (whenever a 0x200 can package is received) to startActuation (if stopped) and set a new setpoint.
 * The function will check if our current cycle is in the same direction as the new cycle. 
 * If so, just update the endpoints to make sure no new ramp up/down phase begins.
 * */
void athrottle_start(int mTargetPosition) {
  if (mTargetPosition<THROTTLE_ACTUATOR_MIN_POT) mTargetPosition=THROTTLE_ACTUATOR_MIN_POT;
  if (mTargetPosition>THROTTLE_ACTUATOR_MAX_POT) mTargetPosition=THROTTLE_ACTUATOR_MAX_POT;

  // within THROTTLE_ACTUATOR_ALLOWED_PERM_ERROR, stop this actuation sequence
  if (ABS(at_actuator_poti_position - mTargetPosition) < THROTTLE_ACTUATOR_ALLOWED_PERM_ERROR) {
    at_actuator_target_position=mTargetPosition; 
    return athrottle_stop();
  }

  // continue current actuation, just update endpoint
  if (mTargetPosition>at_actuator_poti_position && at_actuator_direction==1) {
    at_actuator_target_position=mTargetPosition;
    return;
  }

  // continue current actuation, just update endpoint
  if (mTargetPosition<at_actuator_poti_position && at_actuator_direction==-1) {
    at_actuator_target_position=mTargetPosition;
    return;
  }
  
  at_actuator_target_position=mTargetPosition;
  if (mTargetPosition<at_actuator_poti_position) at_actuator_direction=1;
  else at_actuator_direction=-1;

  at_actuator_duty_cycle_start=millis();
}

/**
 * This is called at least 500/s to check the current actuator position, desired setpoint and actuate through the L298 H-BRIDGE
 * Supports ramping up/down and smoothing out movements close to setpoints to make the driving experience softer
 * */
void athrottle_update() {

  if (at_actuator_direction==0) return;

  if (!retropilotParams.ALLOW_THROTTLE || ABS(at_actuator_poti_position - at_actuator_target_position) < THROTTLE_ACTUATOR_ALLOWED_PERM_ERROR)
      return athrottle_stop();


  if (at_actuator_poti_position<at_actuator_target_position) {
    digitalWrite(THROTTLE_ACTUATOR_M_IN1_PORT, THROTTLE_ACTUATOR_M_IN1_PIN, HIGH); digitalWrite(THROTTLE_ACTUATOR_M_IN2_PORT, THROTTLE_ACTUATOR_M_IN2_PIN, LOW); //motor left (== pull wire)
  }
  else {
    digitalWrite(THROTTLE_ACTUATOR_M_IN1_PORT, THROTTLE_ACTUATOR_M_IN1_PIN, LOW); digitalWrite(THROTTLE_ACTUATOR_M_IN2_PORT, THROTTLE_ACTUATOR_M_IN2_PIN, HIGH); //motor right (== loosen wire)
  }   


  unsigned long currentDutyCycleMs = millis()-at_actuator_duty_cycle_start;

  bool isOffDuty=false;

  if (currentDutyCycleMs>(at_actuator_direction==1 ? THROTTLE_ACTUATOR_RAMP_UP_MS : THROTTLE_ACTUATOR_RAMP_DOWN_MS)) {
    // nothing to do - we're outside of the ramp phases  
  }
  else {
    int currentCyclePosition = currentDutyCycleMs % THROTTLE_ACTUATOR_DUTY_CYCLE_LENGTH_MS;
    int currentCycleNum = currentDutyCycleMs / THROTTLE_ACTUATOR_DUTY_CYCLE_LENGTH_MS;

    if (currentCyclePosition>(at_actuator_direction==1 ? THROTTLE_ACTUATOR_RAMP_UP_INITIAL_DUTY : THROTTLE_ACTUATOR_RAMP_DOWN_INITIAL_DUTY) + currentCycleNum * (at_actuator_direction==1 ? THROTTLE_ACTUATOR_RAMP_UP_PER_CYCLE_DUTY : THROTTLE_ACTUATOR_RAMP_DOWN_PER_CYCLE_DUTY)) {
      isOffDuty=true;
    }
  }
  // "PWM simulation" through on/off duty cycle phases
  if (isOffDuty) {
    digitalWrite(THROTTLE_ACTUATOR_M_ENA_PORT, THROTTLE_ACTUATOR_M_ENA_PIN, 0);  //stop motor - off-duty cycle position - we wait fpr the next currentCycleNum before we pull/loosen a little bit again
  } else {
    digitalWrite(THROTTLE_ACTUATOR_M_ENA_PORT, THROTTLE_ACTUATOR_M_ENA_PIN, 1);  // we run at full speed
  }
}


void athrottle_debug_up() {
  if (retropilotParams.DEBUGMODE)
    athrottle_start(at_actuator_target_position+250);
}
void athrottle_debug_down() {
  if (retropilotParams.DEBUGMODE)
    athrottle_start(at_actuator_target_position-250);
}


/* called at ~ 1kHz */
void athrottle_loop()
{
    if (millis()-at_last_poti_position_update>10) {
      at_actuator_poti_position = athrottle_read_poti();
    }

    // calculating GAS_CMD_PERCENT into actuatorTargetPosition 
    if (!retropilotParams.DEBUGMODE && millis()-at_last_actuation_start>40) {
        at_last_actuation_start=millis();
        athrottle_start((int)(((retropilotParams.GAS_CMD_PERCENT / 100) * (THROTTLE_ACTUATOR_MAX_POT - THROTTLE_ACTUATOR_MIN_POT)) + THROTTLE_ACTUATOR_MIN_POT));
    }

    if (!retropilotParams.ALLOW_THROTTLE && !retropilotParams.DEBUGMODE) {
        athrottle_stop();
        digitalWrite(THROTTLE_ACTUATOR_S_IN3_PORT, THROTTLE_ACTUATOR_S_IN3_PIN, LOW); //open solenoid (both in3 AND enb must be high to close the solenoid for safety)
        digitalWrite(THROTTLE_ACTUATOR_M_ENB_PORT, THROTTLE_ACTUATOR_M_ENB_PIN, LOW); //open solenoid (both in3 AND enb must be high to close the solenoid for safety)
    }
    else { // we're not braking, changing gears, have an OP brake request or having canbus problems: we can actuate
        digitalWrite(THROTTLE_ACTUATOR_S_IN3_PORT, THROTTLE_ACTUATOR_S_IN3_PIN, HIGH); // close solenoid (both in3 AND enb must be high to close the solenoid for safety)
        digitalWrite(THROTTLE_ACTUATOR_M_ENB_PORT, THROTTLE_ACTUATOR_M_ENB_PIN, HIGH); // close solenoid (both in3 AND enb must be high to close the solenoid for safety)
    }

    athrottle_update();
}