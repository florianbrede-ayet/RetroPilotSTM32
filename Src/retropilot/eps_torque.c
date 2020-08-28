/*
  eps_torque.c - Controlling a closed loop stepper over USART3 (misfittech nano stepper with custom firmware).
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

#include "core.h"
#include "retropilot/globals.h"
#include "retropilot/retropilot_params.h"
#include "retropilot/eps_torque.h"
#include "retropilot/easy_transfer.h"
#include "uart.h"
#include "logger.h"


#if DEBUG_SIMULATE_STEERING_ANGLE_SENSOR
extern unsigned long cm_last_recv_steer_angle; // from can_manager, must be updated here if we enable simulated steering angle sensor
bool is_simulated_steering_angle_zeroed;
float simulated_steering_angle_zero_offset;
#endif 

unsigned long eps_last_serial_out=0;
uint8_t eps_serial_package[8];

bool eps_received_serial_message = false;
bool eps_stepper_available = false;  // set to true if the stepper has no error and is in available state. this is not dependant on temporary stepper error, it's meant to be used internally to check if calibration can progress
bool eps_steering_angle_synchronization_valid = false; // set to true after 2 consecutive successful synchronizations

uint8_t eps_stepper_status=0; // DETAILED eps driver status (0 == steering angle synchronization, 1 == working, 2 == serial error, 3 == motor error, 4 == encoder error, 5 == calibration error, 6 == steering angle synchronization error)

unsigned long eps_last_temporary_error = 0;
unsigned long eps_last_serial_write = 0;
unsigned long eps_last_serial_message_received=0;

int8_t eps_debug_torque_percent = 0;

float eps_current_stepper_angle = 0.0f; // current stepper angle received from the closed loop driver

// these are calculated in "calculateOutputsFromTorque" or "calculateOutputsFromPosition" and applied through serial to the closed loop driver
int8_t eps_calculated_stepper_stop = 0;
float eps_calculated_stepper_target_angle = 0.0f;    // the absolute (stepper) target angle sent to the closed loop driver
float eps_calculated_stepper_dynamic_current = 0.0f;
float eps_calculated_stepper_hold_current = 0.0f;

uint32_t eps_unwinding_counter=0;


// steering & stepper angle synchronization specific
/* unsigned long eps_last_sync_time = 0;
unsigned long eps_last_successful_sync = 0;
int8_t eps_sync_errors=0;
int8_t eps_sync_success=0;
bool eps_is_syncing=false;
float eps_sync_start_stepper_angle=0.0f;
float eps_sync_start_steering_angle=0.0f;
unsigned long eps_sync_start=0;
int8_t eps_target_sync_step_counter=-1;
float eps_sync_target_steering_angle=0;
float syncTargetStepperAngle=0;
float eps_target_sync_steering_angle_sum=0;
float eps_target_sync_stepper_angle_sum=0;
int8_t eps_sync_sub_retries=0;
 */

// new steering angle synchronization:
extern unsigned long cm_last_recv_steer_angle;

unsigned long eps_last_sync_time = 0;
unsigned long eps_last_successful_sync = 0;
int8_t eps_sync_errors=0;
float eps_steering_angle_offset=0;


typedef struct SERIAL_DATA_STRUCTURE{
    uint8_t statusMask;
    int32_t currentAngle;
    int32_t targetAngle;
    uint16_t dynamicCurrent;
    int16_t holdCurrent;
	uint8_t counter;
	uint8_t checksum;
    // ^--> 120 bytes + 4 bytes overhead == 124 bytes * 40/s * 2 = 9920 bytes/s = ~80.000 bps
} SERIAL_DATA_STRUCTURE;

 SERIAL_DATA_STRUCTURE eps_serial_data_struct;

uint8_t msg_counter=0;


/* called at initialization */
void eps_setup()
{
    uart_ring *ur;    
    ur = serial_get_ring_by_number(3);
    et_begin((uint8_t*)&eps_serial_data_struct, sizeof(eps_serial_data_struct), ur);
}



void eps_receive() {
    eps_received_serial_message=false;
    while (et_receive_data()) {
        if (eps_serial_data_struct.currentAngle==INT32_MAX) {
            logger_e("received loopback eps usart3 package - check external serial connection!\n");
        }
        else {
             /* logger("CNT: %d  eS: %d  eC: %d  eE: %d  eM: %d  EN: %d  cAng: %d  tAng: %d  currD: %d  currH: %d\n", 
                eps_serial_data_struct.counter, eps_serial_data_struct.statusMask >> 0 & 1, eps_serial_data_struct.statusMask >> 1 & 1,
                eps_serial_data_struct.statusMask >> 2 & 1, eps_serial_data_struct.statusMask >> 3 & 1, eps_serial_data_struct.statusMask >> 4 & 1,
                eps_serial_data_struct.currentAngle, eps_serial_data_struct.targetAngle, eps_serial_data_struct.dynamicCurrent, eps_serial_data_struct.holdCurrent);  */

            eps_current_stepper_angle = eps_serial_data_struct.currentAngle;  
            eps_received_serial_message = true;
      }
    }

  	if (eps_received_serial_message) {
        if (millis()-eps_last_serial_message_received>50) {
            logger_e("slow stepper frame %dms!\n",(int)(millis()-eps_last_serial_message_received));
        }
  		eps_last_serial_message_received=millis();
        
        if (eps_serial_data_struct.statusMask >> 0 & 1) {  // serial error
            eps_last_temporary_error=millis();
            if (eps_stepper_status!=6) eps_stepper_status=2;
        }
        else if(eps_serial_data_struct.statusMask >> 3 & 1 ) { // motor error
            eps_last_temporary_error=millis();
            if (eps_stepper_status!=6) eps_stepper_status=3;
        }
        else
            eps_stepper_available = true;

        if (eps_serial_data_struct.statusMask >> 1 & 1 ||  // stepper calibration table error
            eps_serial_data_struct.statusMask >> 2 & 1) {   // encoder error
            retropilotParams.OP_EPS_UNRECOVERABLE_ERROR = true;
            if (eps_stepper_status!=6) eps_stepper_status=4;
        }


        // calculate the current EPS output torque to send to openpilot
        if ((eps_serial_data_struct.statusMask >> 4 & 1) == 1) {
            if (ABS(eps_serial_data_struct.currentAngle-eps_serial_data_struct.targetAngle)>0.5f*EPS_GEARING) {
                if (eps_serial_data_struct.currentAngle-eps_serial_data_struct.targetAngle>0)
                    retropilotParams.OP_EPS_ACTUAL_TORQUE=STEERING_ANGLE_INVERTED*-eps_serial_data_struct.dynamicCurrent;
                else
                    retropilotParams.OP_EPS_ACTUAL_TORQUE=STEERING_ANGLE_INVERTED*eps_serial_data_struct.dynamicCurrent;
            }
            else {
                if (retropilotParams.currentSteeringAngle>0.5f)
                    retropilotParams.OP_EPS_ACTUAL_TORQUE=eps_serial_data_struct.holdCurrent;
                else if (retropilotParams.currentSteeringAngle<-0.5f)
                    retropilotParams.OP_EPS_ACTUAL_TORQUE=-eps_serial_data_struct.holdCurrent;
                else 
                    retropilotParams.OP_EPS_ACTUAL_TORQUE=0;
            }
        }
        else
            retropilotParams.OP_EPS_ACTUAL_TORQUE=0;

    }
    else {
        if (millis()-eps_last_serial_message_received>120) {
            eps_last_temporary_error=millis();
            eps_stepper_available = false;
            if (eps_stepper_status!=6) eps_stepper_status=2;
        }
        else if (millis()-eps_last_serial_message_received>1000) {
            retropilotParams.OP_EPS_UNRECOVERABLE_ERROR = true;
            if (eps_stepper_status!=6) eps_stepper_status=2;
        }
    }

    if (retropilotParams.OP_EPS_UNRECOVERABLE_ERROR) eps_stepper_available = false;

	return;
}



void eps_calculate_outputs_from_torque() {
    if (eps_received_serial_message || millis()-eps_last_serial_write>=MIN_CALCULATION_FREQ) {

        float commandedTorque = STEERING_ANGLE_INVERTED*retropilotParams.OP_COMMANDED_TORQUE;   // must be inverted depending on gearing and stepper wiring
        eps_calculated_stepper_target_angle=eps_current_stepper_angle;
        
        eps_calculated_stepper_stop = 1;
        eps_calculated_stepper_hold_current = MIN_HOLD_CURRENT;
        eps_calculated_stepper_dynamic_current = MIN_DYNAMIC_CURRENT;

        if (commandedTorque>MIN_TORQUE_THRESHOLD) {
            eps_calculated_stepper_dynamic_current = MAX(MIN_DYNAMIC_CURRENT, MIN(MAX_DYNAMIC_CURRENT, MIN_DYNAMIC_CURRENT+(ABS(commandedTorque)-MIN_TORQUE_THRESHOLD)/MAX_COMMANDED_TORQUE*(MAX_DYNAMIC_CURRENT-MIN_DYNAMIC_CURRENT)));
            eps_calculated_stepper_hold_current = MAX(MIN_HOLD_CURRENT, MIN(MAX_HOLD_CURRENT, MIN_HOLD_CURRENT+(ABS(commandedTorque)-MIN_TORQUE_THRESHOLD)/MAX_COMMANDED_TORQUE*(MAX_HOLD_CURRENT-MIN_HOLD_CURRENT)));
            
            #if REDUCE_UNWINDING_TORQUE     // depends on the implementation in OP
            if (retropilotParams.STEERING_ANGLE_INVERTED*currentSteeringAngle<-2) { // unwinding with reduced torque
                if (eps_unwinding_counter<20) { // ~250ms maximum duration allowed for the last 4° unwinding (misaligned axle or uneven road) - see below
                    eps_calculated_stepper_dynamic_current = eps_calculated_stepper_dynamic_current / DYNAMIC_CURRENT_UNWINDING_DIVISOR;
                    eps_calculated_stepper_hold_current = eps_calculated_stepper_hold_current / HOLD_CURRENT_UNWINDING_DIVISOR;
                }
                if (retropilotParams.STEERING_ANGLE_INVERTED*currentSteeringAngle<-4) eps_unwinding_counter+=2;
                else if (retropilotParams.STEERING_ANGLE_INVERTED*currentSteeringAngle<-6) eps_unwinding_counter+=1;
            }
            else eps_unwinding_counter=0;
            #endif

            // VSS dependant torque limit
            float torqueScale = MIN(100.0f, VSS_DEPENDANT_MIN_TORQUE_PERCENT + (100.0f-VSS_DEPENDANT_MIN_TORQUE_PERCENT) / VSS_DEPENDANT_MAX_TORQUE_SPEED_KMH * retropilotParams.vssAvgSpeedKMH);
            if (retropilotParams.DEBUGMODE) {
                torqueScale=100.0f;
            }
            eps_calculated_stepper_dynamic_current = MAX(MIN_DYNAMIC_CURRENT, MIN(MAX_DYNAMIC_CURRENT, eps_calculated_stepper_dynamic_current/100.0f*torqueScale));

            // reduce dynamic current at steering angle limit edges 10% per degree over STEERING_ANGLE_SOFT_LIMITATION
            float steeringAngleSoftLimit = STEERING_ANGLE_SOFT_LIMITATION_100KMH + (STEERING_ANGLE_SOFT_LIMITATION_0KMH-STEERING_ANGLE_SOFT_LIMITATION_100KMH)/100.0f*(100.0f-MIN(100.0f, retropilotParams.vssAvgSpeedKMH));
            if (STEERING_ANGLE_INVERTED*retropilotParams.currentSteeringAngle>steeringAngleSoftLimit)
                eps_calculated_stepper_dynamic_current = MIN(eps_calculated_stepper_dynamic_current, MAX(0.0f, eps_calculated_stepper_dynamic_current/100*(100-10*(STEERING_ANGLE_INVERTED*retropilotParams.currentSteeringAngle-steeringAngleSoftLimit))));
            
            eps_calculated_stepper_stop = 0;
            eps_calculated_stepper_target_angle = eps_current_stepper_angle + EPS_GEARING * 10; // 10° of steering angle is our new target position offset - only restricted for safety (maximum error on connection failure). should be significant enough to have the stepper PID detect a worthy position error
            
            eps_debug_torque_percent = eps_calculated_stepper_dynamic_current*100.0f/MAX_DYNAMIC_CURRENT;
        }
        else if (commandedTorque<-MIN_TORQUE_THRESHOLD) {
            eps_calculated_stepper_dynamic_current = MAX(MIN_DYNAMIC_CURRENT, MIN(MAX_DYNAMIC_CURRENT, MIN_DYNAMIC_CURRENT+(ABS(commandedTorque)-MIN_TORQUE_THRESHOLD)/MAX_COMMANDED_TORQUE*(MAX_DYNAMIC_CURRENT-MIN_DYNAMIC_CURRENT)));
            eps_calculated_stepper_hold_current = MAX(MIN_HOLD_CURRENT, MIN(MAX_HOLD_CURRENT, MIN_HOLD_CURRENT+(ABS(commandedTorque)-MIN_TORQUE_THRESHOLD)/MAX_COMMANDED_TORQUE*(MAX_HOLD_CURRENT-MIN_HOLD_CURRENT)));

            #if REDUCE_UNWINDING_TORQUE     // depends on the implementation in OP, if it already accounts for unwinding, this should be disabled
            if (STEERING_ANGLE_INVERTED*retropilotParams.currentSteeringAngle>2) { // unwinding with reduced torque
                if (eps_unwinding_counter<20) { // ~250ms maximum duration allowed for the last 4° unwinding (misaligned axle or uneven road) - see below
                    eps_calculated_stepper_dynamic_current = eps_calculated_stepper_dynamic_current / DYNAMIC_CURRENT_UNWINDING_DIVISOR;
                    eps_calculated_stepper_hold_current = eps_calculated_stepper_hold_current / HOLD_CURRENT_UNWINDING_DIVISOR;
                }
                if (STEERING_ANGLE_INVERTED*retropilotParams.currentSteeringAngle>6) eps_unwinding_counter+=1;
                else if (STEERING_ANGLE_INVERTED*retropilotParams.currentSteeringAngle>4) eps_unwinding_counter+=2;
            }
            else eps_unwinding_counter=0;
            #endif

            // VSS dependant torque limit
            float torqueScale = MIN(100.0f, VSS_DEPENDANT_MIN_TORQUE_PERCENT + (100.0f-VSS_DEPENDANT_MIN_TORQUE_PERCENT) / VSS_DEPENDANT_MAX_TORQUE_SPEED_KMH * retropilotParams.vssAvgSpeedKMH);
            if (retropilotParams.DEBUGMODE) {
                torqueScale=100.0f;
            }
            eps_calculated_stepper_dynamic_current = MAX(MIN_DYNAMIC_CURRENT, MIN(MAX_DYNAMIC_CURRENT, eps_calculated_stepper_dynamic_current/100.0f*torqueScale));


            // reduce dynamic current at steering angle limit edges 10% per degree over STEERING_ANGLE_SOFT_LIMITATION
            float steeringAngleSoftLimit = STEERING_ANGLE_SOFT_LIMITATION_100KMH + (STEERING_ANGLE_SOFT_LIMITATION_0KMH-STEERING_ANGLE_SOFT_LIMITATION_100KMH)/100.0f*(100.0f-MIN(100.0f, retropilotParams.vssAvgSpeedKMH));
            if (STEERING_ANGLE_INVERTED*retropilotParams.currentSteeringAngle<-steeringAngleSoftLimit)
                eps_calculated_stepper_dynamic_current = MIN(eps_calculated_stepper_dynamic_current, MAX(0.0f, eps_calculated_stepper_dynamic_current/100.0f*(100.0f-10.0f*(-STEERING_ANGLE_INVERTED*retropilotParams.currentSteeringAngle-steeringAngleSoftLimit))));

            eps_calculated_stepper_stop = 0;
            eps_calculated_stepper_target_angle = eps_current_stepper_angle - EPS_GEARING * 10; // 10° of steering angle is our new target position offset - only restricted for safety (maximum error on connection failure). should be significant enough to have the stepper PID detect a worthy position error

            eps_debug_torque_percent = -eps_calculated_stepper_dynamic_current*100.0f/MAX_DYNAMIC_CURRENT;
        }
        
    }
}


void eps_send() {
    if (eps_received_serial_message || millis()-eps_last_serial_write>=MIN_SERIAL_TX_FREQ) {
		eps_last_serial_write=millis();

		eps_serial_data_struct.statusMask=0;
        if ((!retropilotParams.DEBUGMODE && !retropilotParams.ALLOW_STEERING) || (!retropilotParams.DEBUGMODE && !retropilotParams.OP_LKAS_ENABLED) || retropilotParams.OP_EPS_UNRECOVERABLE_ERROR || retropilotParams.OP_EPS_TEMPORARY_ERROR) {
		    eps_serial_data_struct.statusMask |= 0 << 0; // enable
            eps_serial_data_struct.statusMask |= 1 << 1; // stop
        }
        else {
    		eps_serial_data_struct.statusMask |= 1 << 0; // enable
            eps_serial_data_struct.statusMask |= 0 << 1; // stop
        }
		eps_serial_data_struct.currentAngle = INT32_MAX;   // unused
		eps_serial_data_struct.targetAngle = eps_calculated_stepper_target_angle;
		eps_serial_data_struct.dynamicCurrent = eps_calculated_stepper_dynamic_current;
		eps_serial_data_struct.holdCurrent = eps_calculated_stepper_hold_current;

        eps_serial_data_struct.counter = ++msg_counter;

         /* logger("OUT:  stat: %d  tAng: %d  currD: %d  currH: %d\n",  eps_serial_data_struct.statusMask,
                (int32_t)eps_serial_data_struct.targetAngle, (int)eps_serial_data_struct.dynamicCurrent, eps_serial_data_struct.holdCurrent ); */

		eps_serial_data_struct.checksum = eps_serial_data_struct.statusMask+eps_serial_data_struct.currentAngle+eps_serial_data_struct.targetAngle+eps_serial_data_struct.dynamicCurrent+eps_serial_data_struct.holdCurrent+eps_serial_data_struct.counter;
		et_send_data(eps_serial_data_struct);
	}
}




void eps_synchronize_angle_sensor() {
    if (millis()<3000) return;
    if (millis()-eps_last_sync_time<ANGLE_SYNC_STEP_FREQUENCY) return;

    if (!eps_stepper_available) {
        return;
    }

    if (eps_sync_errors>=6) {
        retropilotParams.OP_EPS_UNRECOVERABLE_ERROR=true;
        eps_stepper_status=6;
        return;
    }    


    if (!eps_received_serial_message) {
        if (millis()-eps_last_serial_message_received>=200) {
            eps_sync_errors++;
            logger_e("EPS sync error #1\n");
        }
        return;
    }
    if (millis()-cm_last_recv_steer_angle>=200) {
        if (millis()-eps_last_successful_sync>=200) {
           eps_sync_errors++;
            logger_e("EPS sync error #2\n");
        }
        eps_last_sync_time=millis();
        return;
    }
    
    if (!eps_steering_angle_synchronization_valid) {
        eps_steering_angle_synchronization_valid=true;
        eps_steering_angle_offset=STEERING_ANGLE_INVERTED*(eps_current_stepper_angle/EPS_GEARING)-retropilotParams.currentSteeringAngle;
        eps_last_successful_sync=millis();
        eps_last_sync_time=millis();
        logger("EPS offset calculated: %d\n", (int)(eps_steering_angle_offset*100));
        return;
    }

    float current_eps_steering_angle_offset=-(eps_current_stepper_angle/EPS_GEARING)-retropilotParams.currentSteeringAngle;

    if (ABS(current_eps_steering_angle_offset-eps_steering_angle_offset)>=EPS_MAX_ANGLE_ERROR) {
        eps_sync_errors++;
        logger_e("EPS sync error: %d\n", (int)(ABS(current_eps_steering_angle_offset-eps_steering_angle_offset)*100));
    }
    else {
        eps_sync_errors=MAX(0, eps_sync_errors-1);
        eps_last_successful_sync=millis();
    }
    
    eps_last_sync_time=millis();
}



void eps_debug_torque_left() {
    if (!retropilotParams.DEBUGMODE) return;
    retropilotParams.OP_COMMANDED_TORQUE = retropilotParams.OP_COMMANDED_TORQUE - 250;
    if (retropilotParams.OP_COMMANDED_TORQUE<-MAX_COMMANDED_TORQUE) retropilotParams.OP_COMMANDED_TORQUE=-MAX_COMMANDED_TORQUE;
}

void eps_debug_torque_right() {
    if (!retropilotParams.DEBUGMODE) return;
    retropilotParams.OP_COMMANDED_TORQUE = retropilotParams.OP_COMMANDED_TORQUE + 250;
    if (retropilotParams.OP_COMMANDED_TORQUE>MAX_COMMANDED_TORQUE) retropilotParams.OP_COMMANDED_TORQUE=MAX_COMMANDED_TORQUE;
}


void eps_debug_zero_simulated_steering_angle() {
    #if DEBUG_SIMULATE_STEERING_ANGLE_SENSOR
    if (millis()-eps_last_serial_message_received<150) {
        is_simulated_steering_angle_zeroed=true;
        simulated_steering_angle_zero_offset=eps_current_stepper_angle;
    }
    #endif
}

/* called at > 1kHz */
void eps_loop()
{
    eps_receive();

    #if DEBUG_SIMULATE_STEERING_ANGLE_SENSOR
        if (!is_simulated_steering_angle_zeroed) {
            eps_last_temporary_error=millis();
        } else if (millis()-eps_last_serial_message_received<150) 
            cm_last_recv_steer_angle = millis();

        retropilotParams.currentSteeringAngle=STEERING_ANGLE_INVERTED*(eps_current_stepper_angle-simulated_steering_angle_zero_offset)/EPS_GEARING;
    #else
    /*if (retropilotParams.DEBUGMODE) {
        retropilotParams.currentSteeringAngle=-eps_current_stepper_angle/EPS_GEARING;
    }*/
    #endif


    if (millis()-eps_last_temporary_error<500 || !eps_steering_angle_synchronization_valid) {
        retropilotParams.OP_EPS_TEMPORARY_ERROR=true;
    }
    else {
        retropilotParams.OP_EPS_TEMPORARY_ERROR=false;
        if (eps_stepper_available && !retropilotParams.OP_EPS_UNRECOVERABLE_ERROR) eps_stepper_status=1;
    }

    // update the status flag sent to openpilot in the 
    if (retropilotParams.OP_EPS_UNRECOVERABLE_ERROR)
        retropilotParams.OP_EPS_TOYOTA_STAUS_FLAG = 99; // steering permanently not available
    else if (retropilotParams.OP_EPS_TEMPORARY_ERROR)
        retropilotParams.OP_EPS_TOYOTA_STAUS_FLAG = 9;  // toyotas "steering temporary not available #1"
    else if (retropilotParams.OP_ON && retropilotParams.OP_LKAS_ENABLED && retropilotParams.OP_STEER_REQUEST==1)
        retropilotParams.OP_EPS_TOYOTA_STAUS_FLAG = 5; // active
    else
        retropilotParams.OP_EPS_TOYOTA_STAUS_FLAG = 1; // standby

    if (!retropilotParams.OP_ON || !retropilotParams.OP_LKAS_ENABLED)   // this allows us to drive ACC only even if no eps is installed or if it's faulty
        retropilotParams.OP_EPS_TOYOTA_STAUS_FLAG = 1; // standby



    #if DEBUG_SIMULATE_STEERING_ANGLE_SENSOR
    if (is_simulated_steering_angle_zeroed) eps_synchronize_angle_sensor();
    #else
    eps_synchronize_angle_sensor();
    #endif

    eps_calculate_outputs_from_torque();

    eps_send();
    
}