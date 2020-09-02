#ifndef _RETROPILOTPARAMS_H_
#define _RETROPILOTPARAMS_H_

#include "../core.h"

typedef struct {
    bool DEBUGMODE;
    bool NO_CLUCH_BRAKE_MODE;

    bool UNRECOVERABLE_CONFIGURATION_ERROR;

    bool ALLOW_THROTTLE;
    bool ALLOW_BRAKE;
    bool ALLOW_STEERING;

    bool OP_ERROR_LKAS;                 // set to true in safety if there is any problem with the eps communication. separate because it's not supposed to "break" ACC functionality
    float GAS_CMD_PERCENT;
    float BRAKE_CMD_PERCENT;
    

    uint8_t ccSetSpeed;
    int gas_pedal_state; // we always send 0 since we handle override behaviour here and not in OP
    int brake_pedal_state; // we always send 0 since we handle override behaviour here and not in OP
    bool blinker_left;
    bool blinker_right;

    float vssAvgSpeedKMH;   // current v_ego, calculated by the vss sensor and streamed through WHEELSPEEDS by the vss module
    float currentSteeringAngle; // current steering angle, taken from the steering angle sensor messages

    // these are extracted from STEERING_LKA openpilot is streaming
    uint8_t OP_STEER_REQUEST;
    float   OP_COMMANDED_TORQUE;
    // this is extracted from our modified STEERING_IPAS message
    float   OP_COMMANDED_TARGET_ANGLE;
    
    // "OP_" data shared from the inputs module to other ecus through the retropilot status can message
    bool OP_ON;
    bool OP_BRAKE_PRESSED;
    bool OP_CLUTCH_PRESSED;
    bool OP_GAS_PRESSED;
    bool OP_LKAS_ENABLED;

    // shared by the VSS sensor over can - triggers for non-abs cars when the VSS sensor detects a wheellock
    bool OP_WHEELLOCK_DETECTED;

    // "OP_" params set by each module based on the received data and status
    bool OP_ERROR_CAN; // if no messages are received or a particular retropilot ecu doesn't send heartbeats
    bool OP_FAULTY_ECU; // if any ecu sent a "fault" status over the last 1 second

    bool OP_EPS_UNRECOVERABLE_ERROR;    // set to true by the eps if its in an unrecoverable error state (e.g. calibration, encoder)
    bool OP_EPS_TEMPORARY_ERROR;        // set to true by the eps if there's a temporary issue with the stepper or it's not synchronized correctly yet
    int OP_EPS_ACTUAL_TORQUE;           // holds the current torque applied by the EPS - this is sent to openpilot in the STEER_TORQUE_SENSOR message
    uint8_t OP_EPS_TOYOTA_STAUS_FLAG;   // holds the current toyota eps compatible status of our stepper eps - sent to op in the EPS_STATUS message
    
} RetropilotParams;

extern RetropilotParams retropilotParams;

#endif