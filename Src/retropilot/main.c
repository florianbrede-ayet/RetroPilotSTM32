/*
  retropilot_main.c - This is the main entrypoint for the retropilot ecu.
  Configuration goes to globals.h and the individual module headers, retropilot_setup(); must be called after board setup.
  retropilot_loop(); should be called at a frequency of 1khz or higher.
  
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

#include "retropilot/retropilot_params.h"
#include "retropilot/can_manager.h"
#include "retropilot/inputs.h"
#include "retropilot/vss.h"
#include "retropilot/actuator_throttle.h"
#include "retropilot/actuator_brake.h"
#include "can_handler.h"
#include "logger.h"

#if EPS_TYPE == EPS_TYPE_TORQUE
#include "retropilot/eps_torque.h"
#elif EPS_TYPE == EPS_TYPE_POSITION
#include "retropilot/eps_position.h"
#endif

// DISPLAY / STATISTICS related variables
unsigned long lastMainLoop=0;
unsigned long lastMainLoopCurrentSecondMillis=0;

int lastMainLoopPerSecond=0; // tracks the number of loops() processed per second. target is around 500/s (if can network is active!)
int lastMainLoopCurrentSecond=0;

bool has_can_lowlevel_error = false;

void module_setup() {
  cm_setup();
  
  #if MODULE_INPUTS
  inputs_setup();
  #endif

  #if MODULE_VSS
  vss_setup();
  #endif
  #if MODULE_THROTTLE_ACTUATOR
  athrottle_setup();    
  #endif
  #if MODULE_BRAKE_ACTUATOR
  abrake_setup();
  #endif
  #if MODULE_EPS
  eps_setup();
  #endif
}


#if DEBUG
unsigned long last_debug_gpio=0;
volatile unsigned int vss_irq_cnt=0;

void debug_gpio() {
  if (millis()-last_debug_gpio<500) return;
  last_debug_gpio = millis();


  logger("GPIO  |  SYS_LOOP: %-4d/s   BTN1: %d   BTN2: %d   BTN3: %d   BTN4: %d   CLU: %d   BRA: %d   GAS: %d   VSS: %d   PO1: %d   PO2: %d   CTX: %d   CRX: %d\n",
    lastMainLoopPerSecond,
    digitalRead(BUTTON_1_PORT, BUTTON_1_PIN),
    digitalRead(BUTTON_2_PORT, BUTTON_2_PIN),
    digitalRead(BUTTON_3_PORT, BUTTON_3_PIN),
    digitalRead(BUTTON_4_PORT, BUTTON_4_PIN),
    digitalRead(CLUTCH_CANCEL_PORT, CLUTCH_CANCEL_PIN),
    digitalRead(BRAKE_CANCEL_PORT, BRAKE_CANCEL_PIN),
    digitalRead(THROTTLE_CANCEL_PORT, THROTTLE_CANCEL_PIN),
    vss_irq_cnt,
    adcRead(THROTTLE_ACTUATOR_POTI_PIN),
    adcRead(BRAKE_ACTUATOR_POTI_PIN),
    cm_tx_hist,
    cm_rx_hist
    );

}

extern float eps_current_stepper_angle; // current stepper angle received from the closed loop driver
unsigned long last_debug_state=0;

void debug_state() {
  if (millis()-last_debug_state<200) return;
  last_debug_state = millis();


  logger("STATE  |  ERROR: %d   ON: %d   A_STEER: %d   A_BRAKE: %d   A_GAS: %d   TORQUE:   %d   BRAKE_PER: %d   GAS_PER: %d   BRAKE_POTI: %d   GAS_POTI: %d   STEER_ANG: %d   STEPPER: %d   EPS_TOYOTA_STATUS: %d\n",
    retropilotParams.UNRECOVERABLE_CONFIGURATION_ERROR || retropilotParams.OP_FAULTY_ECU || retropilotParams.OP_ERROR_CAN ? 1 : 0,
    retropilotParams.OP_ON,
    retropilotParams.ALLOW_STEERING, retropilotParams.ALLOW_BRAKE, retropilotParams.ALLOW_STEERING,
    (int)retropilotParams.OP_COMMANDED_TORQUE, (int)retropilotParams.BRAKE_CMD_PERCENT, (int)retropilotParams.GAS_CMD_PERCENT,
    adcRead(THROTTLE_ACTUATOR_POTI_PIN),
    adcRead(BRAKE_ACTUATOR_POTI_PIN),
    (int)(retropilotParams.currentSteeringAngle*100),
    (int)(eps_current_stepper_angle/EPS_GEARING*100),
    retropilotParams.OP_EPS_TOYOTA_STAUS_FLAG
    );

}
#endif

extern uint8_t debug_actuator;
extern int ab_actuator_target_position;
extern int ab_actuator_poti_position;
extern int at_actuator_target_position;
extern int at_actuator_poti_position;
unsigned long last_debug_general=0;

void debug_general() {
  if (millis()-last_debug_general<500) return;
  last_debug_general = millis();

  logger("ACTU  |  DEB_ACT: %s   AT_TAR: %-4d   AT_POT: %-4d   AB_TAR: %-4d   AB_POT: %-4d   EPS_AVAIL: %d/%d   EPS_TORQUE: %d%\n",
    debug_actuator==0 ? "THR" : debug_actuator==1 ? "BRA" : "EPS",
    at_actuator_target_position,
    at_actuator_poti_position,
    ab_actuator_target_position,
    ab_actuator_poti_position,
    #if EPS_TYPE != EPS_TYPE_NONE && EPS_TYPE != EPS_TYPE_STOCK
    retropilotParams.OP_EPS_TEMPORARY_ERROR ? 0 : 1, eps_stepper_available ? 1 : 0, eps_debug_torque_percent
    #else
    0, 0, 0
    #endif
    );
  
}

void retropilot_loop_safety() {

  retropilotParams.OP_FAULTY_ECU = (millis()-cm_last_recv_module_error_flag<1000);

  retropilotParams.OP_ERROR_CAN = ((!retropilotParams.DEBUGMODE && 
                                    (
                                      millis()-cm_last_recv_pedal_safety > 100 ||
                                      millis()-cm_last_recv_panda_safety > 100 ||
                                      millis()-cm_last_recv > 100 || 
                                      millis()-cm_last_recv_module_inputs > 100 || 
                                      millis()-cm_last_recv_module_vss > 100 || 
                                      millis()-cm_last_recv_module_athrottle > 100 || 
                                      millis()-cm_last_recv_module_abrake > 100)
                                    ));
  #if EPS_TYPE != EPS_NONE
  if (!retropilotParams.DEBUGMODE && millis()-cm_last_recv_steer_angle > 50) 
    retropilotParams.OP_ERROR_LKAS=true;
  else if (!retropilotParams.DEBUGMODE && millis()-cm_last_recv_module_eps > 100) 
    retropilotParams.OP_ERROR_LKAS=true;
  else if (!retropilotParams.DEBUGMODE && millis()-cm_last_recv_steer_cmd > 100) 
    retropilotParams.OP_ERROR_LKAS=true;
  else
    retropilotParams.OP_ERROR_LKAS=false;
  #endif

  if (has_can_lowlevel_error)
    retropilotParams.OP_ERROR_CAN=true;


  retropilotParams.ALLOW_THROTTLE = retropilotParams.OP_ON && !retropilotParams.UNRECOVERABLE_CONFIGURATION_ERROR && !retropilotParams.OP_FAULTY_ECU && !retropilotParams.OP_ERROR_CAN && !retropilotParams.OP_BRAKE_PRESSED && !retropilotParams.OP_CLUTCH_PRESSED && retropilotParams.BRAKE_CMD_PERCENT==0;
  retropilotParams.ALLOW_BRAKE    = retropilotParams.OP_ON && !retropilotParams.OP_WHEELLOCK_DETECTED &&!retropilotParams.UNRECOVERABLE_CONFIGURATION_ERROR && !retropilotParams.OP_FAULTY_ECU && !retropilotParams.OP_ERROR_CAN && !retropilotParams.OP_CLUTCH_PRESSED && !retropilotParams.OP_GAS_PRESSED && retropilotParams.BRAKE_CMD_PERCENT>0;
  retropilotParams.ALLOW_STEERING = retropilotParams.OP_ON && !retropilotParams.OP_ERROR_LKAS && !retropilotParams.UNRECOVERABLE_CONFIGURATION_ERROR && !retropilotParams.OP_FAULTY_ECU && !retropilotParams.OP_ERROR_CAN && retropilotParams.OP_LKAS_ENABLED;
    
  // whenever there is any actual error, make sure to send OP_ON=false on the canbus so openpilot knows it's not supposed to try engaging
  if (retropilotParams.UNRECOVERABLE_CONFIGURATION_ERROR || retropilotParams.OP_FAULTY_ECU || retropilotParams.OP_ERROR_CAN) {   
    retropilotParams.OP_ON=false;
    retropilotParams.OP_LKAS_ENABLED=false;
  }

  // TODO: implement angle sensor and eps specific safety checks
}


void module_loop() { 

  cm_loop_recv();
  
  retropilot_loop_safety(); // checks pedal states, verifies can connection / disables operation for each ecu whenever an error state is detected
  
  cm_loop_send();
  
  #if MODULE_INPUTS
  inputs_loop();
  #endif
  
  #if MODULE_VSS
  vss_loop();
  #endif

  #if MODULE_THROTTLE_ACTUATOR
  athrottle_loop();
  #endif

  #if MODULE_BRAKE_ACTUATOR
  abrake_loop();
  #endif

  #if MODULE_EPS
  eps_loop();
  #endif


  #if DEBUG
  debug_gpio();
  debug_state();
  #endif

  if (retropilotParams.DEBUGMODE)
    debug_general();

  lastMainLoop=millis();

  if (lastMainLoop-lastMainLoopCurrentSecondMillis>=1000) {
    lastMainLoopPerSecond=lastMainLoopCurrentSecond;
    lastMainLoopCurrentSecond=0;
    lastMainLoopCurrentSecondMillis=lastMainLoop;

    if (check_can_errors()) {
      has_can_lowlevel_error=true;
    }
    else {
      has_can_lowlevel_error=false;
    }

  }
  lastMainLoopCurrentSecond++;
}


void module_timer(TIM_HandleTypeDef *htim) {
}

void module_interrupt(uint16_t GPIO_Pin) {
  #if MODULE_VSS
  if(GPIO_Pin == VSS_INT_Pin)
  {
    vss_interrupt();
    #if DEBUG
    vss_irq_cnt++;
    #endif
  }
  #endif
}
