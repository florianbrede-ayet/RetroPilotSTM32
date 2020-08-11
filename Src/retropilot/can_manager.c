/*
  can_manager.c - Handles sending and receiving of can messages between the different ECUs and OpenPilot, incl. fake messages for stock toyota parts  

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


#include "retropilot/canhelper.h"
#include "can_handler.h"
#include "retropilot/globals.h"
#include "retropilot/retropilot_params.h"

unsigned long cm_last_send_100hz=0L;
unsigned long cm_last_send_50hz=0L;
unsigned long cm_last_send_33hz=0L;

int cm_rx_cnt=0;
int cm_rx_hist=0; // tracks the number of received can messages per second for debugging purposes

int cm_tx_cnt=0;
int cm_tx_hist=0; // tracks the number of sent can messages per second for debugging purposes

unsigned long cm_last_update_msg_curr_sec=0;

uint8_t cm_counter=0;


unsigned long cm_last_recv=0L;

// we track can message timings for each module individually. if a module is enabled on the same ecu, the timer is just updated with every can send
unsigned long cm_last_recv_module_inputs=0L;    
unsigned long cm_last_recv_module_vss=0L;
unsigned long cm_last_recv_module_athrottle=0L;
unsigned long cm_last_recv_module_abrake=0L;
unsigned long cm_last_recv_module_eps=0L;

unsigned long cm_last_recv_steer_cmd=0L;
unsigned long cm_last_recv_steer_angle=0L;

unsigned long cm_last_recv_module_error_flag = 0; // updated whenever an ecu sends a heartbeat with a fault state




/* called at initialization */
void cm_setup()
{

}




void cm_send_fake_toyota_can_messages(bool trigger100Hz, bool trigger50Hz, bool trigger33Hz) {

    if (!trigger33Hz) return;

    uint8_t buf[8];
    
    // last but not least: the fake messages are sent by the main ("inputs" ECU)
    #if MODULE_INPUTS
    //0x3b7 msg ESP_CONTROL
    canhelper_reset_buffer(buf);
    buf[7] = 0x08;
    can_send_message(0, 0x3b7, buf);
    cm_tx_cnt++;


    //0x620 msg STEATS_DOORS
    canhelper_reset_buffer(buf);
    buf[0] = 0x10;
    buf[3] = 0x1d;
    buf[4] = 0xb0;
    buf[5] = 0x40;
    can_send_message(0, 0x620, buf);
    cm_tx_cnt++;

    // 0x3bc msg GEAR_PACKET
    canhelper_reset_buffer(buf);
    buf[5] = 0x80;
    can_send_message(0, 0x3bc, buf);
    cm_tx_cnt++;

    //0x224 msg fake brake module
    canhelper_reset_buffer(buf);
    buf[7] = 0x8;
    can_send_message(0, 0x224, buf);
    cm_tx_cnt++;

    //0x260 fake STEER_TORQUE_SENSOR  
    canhelper_reset_buffer(buf);
    buf[0] = 0x08;
    buf[1] = 0xff;
    buf[2] = 0xfb;
    buf[5] = 0xff;
    buf[6] = 0xdc;
    buf[7] = 0x47;
    can_send_message(0, 0x260, buf);
    cm_tx_cnt++;

    // 0x2c1 msg GAS_PEDAL
    canhelper_reset_buffer(buf);
    buf[0] = (!retropilotParams.gas_pedal_state << 3) & 0x08;
    can_send_message(0, 0x2c1, buf);
    cm_tx_cnt++;

    //0x614 msg steering_levers
    canhelper_reset_buffer(buf);
    buf[0] = 0x29;
    buf[2] = 0x01;
    buf[3] = ((retropilotParams.blinker_left << 5) & 0x20) | ((retropilotParams.blinker_right << 4) & 0x10);
    buf[6] = 0x76;
    canhelper_put_toyota_checksum(buf, 0x614);
    can_send_message(0, 0x614, buf);
    cm_tx_cnt++;
    #endif
}





void cm_send_standard_toyota_can_messages(bool trigger100Hz, bool trigger50Hz, bool trigger33Hz) {
    
    if (!trigger50Hz) return;

    uint8_t buf[8];    
    // SENDING_CAN_MESSAGES
    #if MODULE_INPUTS
    //0x1d2 msg PCM_CRUISE
    canhelper_reset_buffer(buf);
    buf[0] = ((retropilotParams.OP_ON << 5) & 0x20) | ((!retropilotParams.gas_pedal_state << 4) & 0x10);
    buf[6] = (retropilotParams.OP_ON << 7) & 0x80;
    canhelper_put_toyota_checksum(buf, 0x1d2);
    can_send_message(0, 0x1d2, buf);
    cm_tx_cnt++;

    //0x1d3 msg PCM_CRUISE_2
    canhelper_reset_buffer(buf);
    buf[1] = ((retropilotParams.OP_ON << 7) & 0x80) | 0x28;
    buf[2] = retropilotParams.ccSetSpeed;
    canhelper_put_toyota_checksum(buf, 0x1d3);
    can_send_message(0, 0x1d3, buf);
    cm_tx_cnt++;
    #endif

    #if MODULE_VSS
    //0xaa msg defaults 1a 6f WHEEL_SPEEDS
    canhelper_reset_buffer(buf);
    uint16_t wheelspeed = 0x1a6f + (retropilotParams.vssAvgSpeedKMH * 100);
    buf[0] = (wheelspeed >> 8) & 0xFF;
    buf[1] = (wheelspeed >> 0) & 0xFF;
    buf[2] = (wheelspeed >> 8) & 0xFF;
    buf[3] = (wheelspeed >> 0) & 0xFF;
    buf[4] = (wheelspeed >> 8) & 0xFF;
    buf[5] = (wheelspeed >> 0) & 0xFF;
    buf[6] = (wheelspeed >> 8) & 0xFF;
    buf[7] = (wheelspeed >> 0) & 0xFF;
    can_send_message(0, 0xaa, buf);
    cm_tx_cnt++;
    #endif

    #if MODULE_EPS
    //0x262 fake EPS_STATUS
    canhelper_reset_buffer(buf);
    buf[3] = 0x3;
    buf[4] = 0x6c;
    can_send_message(0, 0x262, buf);
    cm_tx_cnt++;
    #endif
}




void cm_send_retropilot_can_messages(bool trigger100Hz, bool trigger50Hz, bool trigger33Hz) {
    
    if (!trigger100Hz) return;

    uint8_t buf[8];    

    canhelper_reset_buffer(buf);

    #if MODULE_INPUTS
        cm_last_recv_module_inputs=millis();
    		buf[0] |= 1 << 0; // inputs heartbeat flag
    		buf[1] |= retropilotParams.UNRECOVERABLE_CONFIGURATION_ERROR ? 0 : 1 << 0; // inputs status flag (1 == working)

        buf[2] |= (retropilotParams.OP_ON             ? 1 : 0) << 0; // DETAILED inputs flags
        buf[2] |= (retropilotParams.OP_BRAKE_PRESSED  ? 1 : 0) << 1; // DETAILED inputs flags
        buf[2] |= (retropilotParams.OP_CLUTCH_PRESSED ? 1 : 0) << 2; // DETAILED inputs flags
        buf[2] |= (retropilotParams.OP_GAS_PRESSED    ? 1 : 0) << 3; // DETAILED inputs flags
        buf[2] |= (retropilotParams.OP_LKAS_ENABLED   ? 1 : 0) << 4; // DETAILED inputs flags
        // "5" is taken by VSS to send WHEELLOCKS!
        buf[2] |= (retropilotParams.OP_FAULTY_ECU     ? 1 : 0) << 6; // DETAILED inputs flags (this one is just for reference, each ECU sets this error on its own whenever any ecu sends a "0" status flag)
        buf[2] |= (retropilotParams.OP_ERROR_CAN      ? 1 : 0) << 7; // DETAILED inputs flags (this one is just for reference, each ECU sets this error on its own)

        // some data for debugging purposes
        buf[4] = retropilotParams.ccSetSpeed;
        buf[5] = (int)retropilotParams.vssAvgSpeedKMH;

    #endif
    #if MODULE_VSS
        cm_last_recv_module_vss=millis();
    		buf[0] |= 1 << 1; // actuator heartbeat flag
    		buf[1] |= retropilotParams.UNRECOVERABLE_CONFIGURATION_ERROR ? 0 : 1 << 1; // actuator status flag (1 == working)
        buf[2] |= (retropilotParams.OP_WHEELLOCK_DETECTED   ? 1 : 0) << 5; // DETAILED inputs flags
    #endif
    #if MODULE_THROTTLE_ACTUATOR
        cm_last_recv_module_athrottle=millis();
    		buf[0] |= 1 << 2; // throttle heartbeat flag
    		buf[1] |= retropilotParams.UNRECOVERABLE_CONFIGURATION_ERROR ? 0 : 1 << 2; // throttle status flag (1 == working)
    #endif
    #if MODULE_BRAKE_ACTUATOR
        cm_last_recv_module_abrake=millis();
    		buf[0] |= 1 << 3; // brake heartbeat flag
    		buf[1] |= retropilotParams.UNRECOVERABLE_CONFIGURATION_ERROR ? 0 : 1 << 3; // brake status flag (1 == working)
    #endif
    #if MODULE_EPS
        cm_last_recv_module_eps=millis();
    		buf[0] |= 1 << 4; // eps heartbeat flag
    		buf[1] |= retropilotParams.UNRECOVERABLE_CONFIGURATION_ERROR ? 0 : 1 << 4; // eps status flag (1 == working)
        buf[3] = 1; // DETAILED eps driver status (0 == steering angle synchronization, 1 == working, 2 == serial error, 3 == motor error, 4 == encoder error, 5 == calibration error, 6 == steering angle synchronization error)
    #endif
    
    buf[6]=++cm_counter;

    canhelper_put_toyota_checksum(buf, 0x112);

    can_send_message(0, 0x112, buf);
    cm_tx_cnt++;

}

/* called at ~ 1kHz */
void cm_loop_recv() {
  CAN_STD_Msg rxMsg;

  while (can_receive_message(0, &rxMsg))
  {
    cm_last_recv=millis();

    cm_rx_cnt++;

    switch (rxMsg.id) {
      case 0x200: { // COMMA PEDAL GAS COMMAND from OP
        float GAS_CMD = (rxMsg.buf[0] << 8 | rxMsg.buf[1] << 0); 
        // scale GAS_CMD into GAS_CMD_PERCENT
        if (GAS_CMD >= OP_MIN_GAS_COMMAND) {
          GAS_CMD = (GAS_CMD>OP_MAX_GAS_COMMAND ? OP_MAX_GAS_COMMAND : GAS_CMD);
        }
        else {
          GAS_CMD = OP_MIN_GAS_COMMAND;
        }
        retropilotParams.GAS_CMD_PERCENT = ((100/(OP_MAX_GAS_COMMAND - OP_MIN_GAS_COMMAND)) * (GAS_CMD - OP_MIN_GAS_COMMAND));
        break; 
      }
      case 0x343: { // standard toyota ACC_CONTROL (we want to extract the brake requests from ACCEL_CMD here)
        float BRAKE_CMD = ((rxMsg.buf[0] << 8 | rxMsg.buf[1] << 0) * -1); 

        if (BRAKE_CMD >= OP_MIN_BRAKE_COMMAND) {
          BRAKE_CMD = (BRAKE_CMD>OP_MAX_BRAKE_COMMAND ? OP_MAX_BRAKE_COMMAND : BRAKE_CMD);
        }
        else {
          BRAKE_CMD = OP_MIN_BRAKE_COMMAND;
        }
        retropilotParams.BRAKE_CMD_PERCENT = ((100/(OP_MAX_BRAKE_COMMAND - OP_MIN_BRAKE_COMMAND)) * (BRAKE_CMD - OP_MIN_BRAKE_COMMAND));
        break;
      }

      case 0xaa: { // WHEELSPEEDS message for all ecus except vss (which is sending 0xaa)
        #if MODULE_VSS
          retropilotParams.UNRECOVERABLE_CONFIGURATION_ERROR = true;  // there must not be more than one VSS module, if we receive this message as vss ecu, something is wrong
        #endif
        retropilotParams.vssAvgSpeedKMH = canhelper_parse_be_float(rxMsg.buf, -67.67, 0.01, 7, 16);
        break;
      }
    
      case 0x112: { // retropilot status message
        if (canhelper_verify_toyota_checksum(rxMsg.buf, 0x112)) {
          if (rxMsg.buf[0] >> 0 & 1) { // message includes INPUTS heartbeat
            cm_last_recv_module_inputs = millis();
            #if MODULE_INPUTS
              retropilotParams.UNRECOVERABLE_CONFIGURATION_ERROR = true;  // collision with another inputs module on this can bus
            #endif
            if (!(rxMsg.buf[1] >> 0 & 1)) { // if the inputs status flag is not "0", the module is in a fault state
              cm_last_recv_module_error_flag = millis(); // will trigger "FAULTY_ECU" in safety
            }

            // parse the status data from the inputs module
            retropilotParams.OP_ON = rxMsg.buf[2] >> 0 & 1;
            retropilotParams.OP_BRAKE_PRESSED = rxMsg.buf[2] >> 1 & 1;
            retropilotParams.OP_CLUTCH_PRESSED = rxMsg.buf[2] >> 2 & 1;
            retropilotParams.OP_GAS_PRESSED = rxMsg.buf[2] >> 3 & 1;
            retropilotParams.OP_LKAS_ENABLED = rxMsg.buf[2] >> 4 & 1;

            retropilotParams.ccSetSpeed = rxMsg.buf[4];
            // for wheelspeeds, we parse the more accurate WHEELSPEEDS message

          }

          if (rxMsg.buf[0] >> 1 & 1) { // message includes VSS heartbeat
            cm_last_recv_module_vss = millis();
            #if MODULE_VSS
              retropilotParams.UNRECOVERABLE_CONFIGURATION_ERROR = true;  // collision with another vss module on this can bus
            #endif
            if (!(rxMsg.buf[1] >> 1 & 1)) { // if the vss status flag is not "0", the module is in a fault state
              cm_last_recv_module_error_flag = millis(); // will trigger "FAULTY_ECU" in safety
            }
            // parse the status data from the vss module
            retropilotParams.OP_WHEELLOCK_DETECTED = rxMsg.buf[2] >> 5 & 1;
          }

          if (rxMsg.buf[0] >> 2 & 1) { // message includes throttle heartbeat
            cm_last_recv_module_athrottle = millis();
            #if MODULE_THROTTLE_ACTUATOR
              retropilotParams.UNRECOVERABLE_CONFIGURATION_ERROR = true;  // collision with another throttle module on this can bus
            #endif
            if (!(rxMsg.buf[1] >> 2 & 1)) { // if the vss status flag is not "0", the module is in a fault state
              cm_last_recv_module_error_flag = millis(); // will trigger "FAULTY_ECU" in safety
            }
          }

          if (rxMsg.buf[0] >> 3 & 1) { // message includes brake heartbeat
            cm_last_recv_module_abrake = millis();
            #if MODULE_BRAKE_ACTUATOR
              retropilotParams.UNRECOVERABLE_CONFIGURATION_ERROR = true;  // collision with another brake module on this can bus
            #endif
            if (!(rxMsg.buf[1] >> 3 & 1)) { // if the vss status flag is not "0", the module is in a fault state
              cm_last_recv_module_error_flag = millis(); // will trigger "FAULTY_ECU" in safety
            }
          }

           if (rxMsg.buf[0] >> 4 & 1) { // message includes eps heartbeat
            cm_last_recv_module_eps = millis();
            #if MODULE_EPS
              retropilotParams.UNRECOVERABLE_CONFIGURATION_ERROR = true;  // collision with another brake module on this can bus
            #endif
            if (!(rxMsg.buf[1] >> 4 & 1)) { // if the EPS status flag is not "0", the module is in a fault state
              cm_last_recv_module_error_flag = millis(); // will trigger "FAULTY_ECU" in safety
            }

            if (rxMsg.buf[3] != 1) { // if the detailed EPS status flag is not "0", we currently just assume a fault state (and don't allow engaging)
              cm_last_recv_module_error_flag = millis(); // will trigger "FAULTY_ECU" in safety
            }
          }
          
        }

        // TODO: update params, update last can receives, verify that no ecu receives messages from its own kind (otherwise unrecoverable error)
        break;
      }
      
      case 0x2e4: { // STEERING_LKA (steer requests from openpilot)
        if (canhelper_verify_toyota_checksum(rxMsg.buf, 0x2e4)) {
          cm_last_recv_steer_cmd = millis();
          retropilotParams.OP_STEER_REQUEST    = canhelper_parse_be_byte(rxMsg.buf, 0, 1, 0, 1);
          retropilotParams.OP_COMMANDED_TORQUE = canhelper_parse_be_int(rxMsg.buf, 0, 1, 15, 16);
        }
        break;
      }

      case 0x25: { // STEER_ANGLE_SENSOR (coming from the TSS steering angle sensor)
        cm_last_recv_steer_angle = millis();
        float steerAngle    = canhelper_parse_be_float(rxMsg.buf, 0, 1.5f, 3, 12);
        float steerFraction = canhelper_parse_be_float(rxMsg.buf, 0, 0.1f, 39, 4);
        retropilotParams.currentSteeringAngle = steerAngle+steerFraction;
        break;
      }
      
    } 
  }
}


/* called at ~ 1kHz */
void cm_loop_send()
{
    bool trigger100Hz = false;
    if (millis()-cm_last_send_100hz>=10) {
        trigger100Hz=true;
        cm_last_send_100hz=millis();
    }
    
    bool trigger50Hz = false;
    if (millis()-cm_last_send_50hz>=20) {
        trigger50Hz=true;
        cm_last_send_50hz=millis();
    }
    
    bool trigger33Hz = false;
    if (millis()-cm_last_send_33hz>=30) {
        trigger33Hz=true;
        cm_last_send_33hz=millis();
    }

    cm_send_fake_toyota_can_messages(trigger100Hz, trigger50Hz, trigger33Hz);
    cm_send_standard_toyota_can_messages(trigger100Hz, trigger50Hz, trigger33Hz);
    cm_send_retropilot_can_messages(trigger100Hz, trigger50Hz, trigger33Hz);

    if (millis()-cm_last_update_msg_curr_sec>=1000) {
      cm_rx_hist=cm_rx_cnt;
      cm_rx_cnt=0;
      cm_last_update_msg_curr_sec=millis();

      cm_tx_hist=cm_tx_cnt;
      cm_tx_cnt=0;
    }
}