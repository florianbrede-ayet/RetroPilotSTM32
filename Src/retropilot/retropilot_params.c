#include "retropilot/retropilot_params.h"

RetropilotParams retropilotParams = {
    .DEBUGMODE = false,
    .NO_CLUCH_BRAKE_MODE = false,

    .UNRECOVERABLE_CONFIGURATION_ERROR = false,

    .ALLOW_THROTTLE = false,
    .ALLOW_BRAKE = false,
    .ALLOW_STEERING = false,

    .OP_ERROR_LKAS = false,

    .GAS_CMD_PERCENT = 0,
    .BRAKE_CMD_PERCENT = 0,
    
    .ccSetSpeed = 90,
    .gas_pedal_state = 0, // we always send 0 since we handle override behaviour here and not in OP
    .brake_pedal_state = 0, // we always send 0 since we handle override behaviour here and not in OP
    .blinker_left = true,
    .blinker_right = true,

    .vssAvgSpeedKMH = 0,   // current v_ego, calculated by the vss sensor and streamed through WHEELSPEEDS by the vss module
    .currentSteeringAngle = 0, // current steering angle, taken from the steering angle sensor messages
    .currentSteeringRate = 0,  // current steering rate, taken from steering angle sensor messages or calculated at 10Hz

    // these are extracted from STEERING_LKA openpilot is streaming
    .OP_STEER_REQUEST    = 0,
    .OP_COMMANDED_TORQUE = 0,
    .OP_COMMANDED_TARGET_ANGLE = 0,
    
    // "OP_" data shared from the inputs module to other ecus through the retropilot status can message
    .OP_ON = false,
    .OP_BRAKE_PRESSED = true,
    .OP_CLUTCH_PRESSED = true,
    .OP_GAS_PRESSED = false,
    .OP_LKAS_ENABLED = false,

    // shared by the VSS sensor over can - triggers for non-abs cars when the VSS sensor detects a wheellock
    .OP_WHEELLOCK_DETECTED = false,

    // "OP_" params set by each module based on the received data and status
    .OP_ERROR_CAN = false, // if no messages are received or a particular retropilot ecu doesn't send heartbeats
    .OP_FAULTY_ECU = false, // if any ecu sent a "fault" status over the last 1 second

    .OP_EPS_UNRECOVERABLE_ERROR = false,    // set to true by the eps if its in an unrecoverable error state (e.g. calibration, encoder)
    .OP_EPS_TEMPORARY_ERROR = false,        // set to true by the eps if there's a temporary issue with the stepper or it's not synchronized correctly yet
    .OP_EPS_ACTUAL_TORQUE = 0,              // holds the current torque applied by the EPS - this is sent to openpilot in the STEER_TORQUE_SENSOR message
    .OP_EPS_TOYOTA_STAUS_FLAG = 2,          // holds the current toyota eps compatible status of our stepper eps - sent to op in the EPS_STATUS message
};
