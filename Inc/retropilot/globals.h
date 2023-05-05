#ifndef _GLOBALS_H_
#define _GLOBALS_H_

#include "../main.h"


#define VERSION 4
#define ENABLE_OLED 0
#define OLED_REFRESH_MS 100


#define DEBUG_SIMULATE_STEERING_ANGLE_SENSOR    0   // DEBUG ONLY!  if set to 1, the encoder will replace a physical TSS steering angle sensor (zero sensor by pressing +/- on the keypad while disengaged - required before able to engage)

#define CAR_WITHOUT_ABS_BRAKES    1    // set to 1 if the car does not have antilock-braking. if enabled, braking will be interrupted within ~100ms (depending on VSS sensor) if a wheel-lock is detected

#define STEER_ANGLE_SENSOR_BOSCH_2007 1
#define STEER_ANGLE_SENSOR_COROLLA 0

// define the type of eps controlling we want (torque controlled, position controlled or no eps)
#define EPS_TYPE_NONE             0
#define EPS_TYPE_TORQUE           1
#define EPS_TYPE_POSITION         2
#define EPS_TYPE_STOCK            3 // stock means an actual toyota eps is installed

//#define EPS_TYPE    EPS_TYPE_STOCK
#define EPS_TYPE    EPS_TYPE_NONE



#define MODULE_INPUTS             1
#define MODULE_VSS                1
#define MODULE_THROTTLE_ACTUATOR  1
#define MODULE_BRAKE_ACTUATOR     1
#define MODULE_EPS                1


/* RETROPILOT STM32 V1.0

This ECU can be used to control a BMW cruise throttle actuator (BMW/VDO "8 369 027" / "408.201/013/001") over CAN and will also handle the cruise part (speed signal, buttons, cruise state). 

original research: https://github.com/Lukilink

COMPONENTS:
- STM32F405 Retropilot ECU
- 0.96"/1.3" 128x64 OLED DISPLAY I2C
- (3/4)+ Button Keypad (e.g. https://www.amazon.de/gp/product/B079JWDQBW)
- 1x ODB Female Port (e.g. https://www.amazon.de/gp/product/B07TZMXQLK)
- various molex connectors
- 1x 30mm 5V Fan




RETROPILOT STM32F405 ECU PINOUT:


J_INOUT1

KEYPAD 2 / ON		J_INOUT1 1			PA0
KEYPAD 4 / SPD-		J_INOUT1 2			PA1
KEYPAD 3 / SPD+		J_INOUT1 3			PA2
KEYPAD 5 / LKAS		J_INOUT1 4			PA3
CLUTCH / HALL		J_INOUT1 5			PB12
BRAKE / HALL		J_INOUT1 6			PB13
[GAS / HALL			J_INOUT1 7			PB14] FUTURE!
KEYPAD 1 / HALLGND	J_INOUT1 8			GND


J_VSS_SENSOR1

SPD SENSOR +5V whi	J_VSS_SENSOR1 1			+5V
SPD SENSOR GND bla	J_VSS_SENSOR1 2			GND
SPD SENSOR GND gre	J_VSS_SENSOR1 3			PB4


J_SERIAL_DEBUG1

PROGRAMMER TX 	RX	J_SERIAL_DEBUG1 1		PA10
PROGRAMMER RX	TX	J_SERIAL_DEBUG1 2		PA9


J_SERIAL_3

EPS TX 		RX	J_SERIAL_3 1		USART3_RX	PB11
EPS RX		TX	J_SERIAL_3 2		USART3 TX	PB10


J_CAN_1

+12V				J_CAN_1 1	+12V
GND					J_CAN_1 2	GND
CAN1 L 				J_CAN_1 3	TJA1040-1 CAN L 	--> used to connect angle sensor, steering mcu and RADAR 2 (black) + RADAR 6 (purple) (cable black & blue)
CAN1 H 				J_CAN_1 4	TJA1040-1 CAN H		--> used to connect angle sensor, steering mcu and RADAR 3 (white) + RADAR 5 (green) (cable yellow & green)


J_CAN_2

+12V				J_CAN_2 CAN2 1	+12V
GND					J_CAN_2 CAN2 2	GND
CAN2 L 				J_CAN_2	TJA1040-2 CAN L
CAN2 H 				J_CAN_2	TJA1040-2 CAN H


L298-1

M_IN1 				PC12
M_IN2 				PC11
M_ENA 				PC6
S_IN3 				PC10
S_IN4 				PC9
M_ENB				PC7


J_ACTUATOR1

ACTUATOR 4 red		L298-1 OUT1			J_ACTUATOR1 1
ACTUATOR 1 ora	 	L298-1 OUT2			J_ACTUATOR1 2
ACTUATOR 2 bla		L298-1 OUT3			J_ACTUATOR1 3
ACTUATOR 3 yel		L298-1 OUT4			J_ACTUATOR1 4
ACTUATOR 7 whi		+3.3V				J_ACTUATOR1 5
ACTUATOR 6 gre		RES1KOHM / PA7		J_ACTUATOR1 6


L298-2

M_IN1 				PA15
M_IN2 				PB1
M_ENA 				PA6
S_IN3 				PB9
S_IN4 				PB15
M_ENB				PA8


J_ACTUATOR2

ACTUATOR 4 red		L298-2 OUT1			J_ACTUATOR2 1
ACTUATOR 1 ora	 	L298-2 OUT2			J_ACTUATOR2 2
ACTUATOR 2 bla		L298-2 OUT3			J_ACTUATOR2 3
ACTUATOR 3 yel		L298-2 OUT4			J_ACTUATOR2 4
ACTUATOR 7 whi		+3.3V				J_ACTUATOR2 5
ACTUATOR 6 gre		RES1KOHM / PB0		J_ACTUATOR2 6


TJA1040-1

MCP RX				PA11
MCP TX				PA12


TJA1040-2

MCP RX				PB5
MCP TX				PB6


J_I2C_DISP1		[OPTION: 0.96 OLED DISPLAY I2C]

PIN_OLED_GND	J_I2C_DISP 1	GND
PIN_OLED_VCC	J_I2C_DISP 2	+5V
PIN_OLED_SCL 	J_I2C_DISP 3	PB8
PIN_OLED_SDA 	J_I2C_DISP 4	PB7

*/




//////////////////// PIN CONFIG ////////////////////

// CLUTCH & BRAKE PEDAL PINS
#define CLUTCH_CANCEL_PIN SW_CLUTCH_Pin      // pulled to GND through default open hall sensor when CLUTCH pedal is NOT pressed
#define CLUTCH_CANCEL_PORT SW_CLUTCH_GPIO_Port
#define BRAKE_CANCEL_PIN SW_BRAKE_Pin       // pulled to GND through default open hall sensor when BRAKE pedal is NOT pressed
#define BRAKE_CANCEL_PORT SW_BRAKE_GPIO_Port      
#define THROTTLE_CANCEL_PIN SW_GAS_Pin    // pulled to GND through default open hall sensor when THROTTLE pedal is NOT pressed
#define THROTTLE_CANCEL_PORT SW_GAS_GPIO_Port   

// BUTTONS AND SWITCHES (PULLDOWN INPUT)
#define BUTTON_1_PIN  KEYPAD_BTN_1_Pin  // ON/OFF
#define BUTTON_1_PORT KEYPAD_BTN_1_GPIO_Port
#define BUTTON_2_PIN  KEYPAD_BTN_2_Pin  // SPD-
#define BUTTON_2_PORT KEYPAD_BTN_2_GPIO_Port
#define BUTTON_3_PIN  KEYPAD_BTN_3_Pin  // SPD+
#define BUTTON_3_PORT KEYPAD_BTN_3_GPIO_Port
#define BUTTON_4_PIN  KEYPAD_BTN_4_Pin  // currently unused / LKAS later
#define BUTTON_4_PORT KEYPAD_BTN_4_GPIO_Port

// THROTTLE ACTUATOR POSITION POTENTIOMETER PIN
#define THROTTLE_ACTUATOR_POTI_PIN      Actuator_1_Poti_Pin       // connect the potentiometer of your car's throttle actuator

// THROTTLE ACTUATOR H BRIDGE PINS
#define THROTTLE_ACTUATOR_M_IN1_PIN     L2981_IN1_Pin // motor direction (IN1-H, IN2-L = left, IN1-L, IN2-H = right)
#define THROTTLE_ACTUATOR_M_IN1_PORT    L2981_IN1_GPIO_Port
#define THROTTLE_ACTUATOR_M_IN2_PIN     L2981_IN2_Pin // motor direction (IN1-H, IN2-L = left, IN1-L, IN2-H = right)
#define THROTTLE_ACTUATOR_M_IN2_PORT    L2981_IN2_GPIO_Port
#define THROTTLE_ACTUATOR_S_IN3_PIN     L2981_IN3_Pin // SOLENOID / actuator clutch drive pin (for safety, the solenoid will only engage if both ENB + IN3 are high)
#define THROTTLE_ACTUATOR_S_IN3_PORT    L2981_IN3_GPIO_Port
#define THROTTLE_ACTUATOR_S_IN4_PIN     L2981_IN4_Pin // SOLENOID / actuator clutch drive pin (for safety, the solenoid will only engage if both ENB + IN3 are high)
#define THROTTLE_ACTUATOR_S_IN4_PORT    L2981_IN4_GPIO_Port
#define THROTTLE_ACTUATOR_M_ENA_PIN     L2981_ENA_Pin // 255 is run / LOW is stopp   // motor speed
#define THROTTLE_ACTUATOR_M_ENA_PORT    L2981_ENA_GPIO_Port
#define THROTTLE_ACTUATOR_M_ENB_PIN     L2981_ENB_Pin // SOLENOID / actuator clutch enable pin (for safety, the solenoid will only engage if both ENB + IN3 are high)
#define THROTTLE_ACTUATOR_M_ENB_PORT    L2981_ENB_GPIO_Port

// BRAKE ACTUATOR POSITION POTENTIOMETER PIN
#define BRAKE_ACTUATOR_POTI_PIN         Actuator_2_Poti_Pin       // connect the potentiometer of your car's brake actuator

// BRAKE ACTUATOR H BRIDGE PINS
#define BRAKE_ACTUATOR_M_IN1_PIN     L2982_IN1_Pin // motor direction (IN1-H, IN2-L = left, IN1-L, IN2-H = right)
#define BRAKE_ACTUATOR_M_IN1_PORT    L2982_IN1_GPIO_Port
#define BRAKE_ACTUATOR_M_IN2_PIN     L2982_IN2_Pin // motor direction (IN1-H, IN2-L = left, IN1-L, IN2-H = right)
#define BRAKE_ACTUATOR_M_IN2_PORT    L2982_IN2_GPIO_Port
#define BRAKE_ACTUATOR_S_IN3_PIN     L2982_IN3_Pin // SOLENOID / actuator clutch drive pin (for safety, the solenoid will only engage if both ENB + IN3 are high)
#define BRAKE_ACTUATOR_S_IN3_PORT    L2982_IN3_GPIO_Port
#define BRAKE_ACTUATOR_S_IN4_PIN     L2982_IN4_Pin // SOLENOID / actuator clutch drive pin (for safety, the solenoid will only engage if both ENB + IN3 are high)
#define BRAKE_ACTUATOR_S_IN4_PORT    L2982_IN4_GPIO_Port
#define BRAKE_ACTUATOR_M_ENA_PIN     L2982_ENA_Pin // 255 is run / LOW is stopp   // motor speed
#define BRAKE_ACTUATOR_M_ENA_PORT    L2982_ENA_GPIO_Port
#define BRAKE_ACTUATOR_M_ENB_PIN     L2982_ENB_Pin // SOLENOID / actuator clutch enable pin (for safety, the solenoid will only engage if both ENB + IN3 are high)
#define BRAKE_ACTUATOR_M_ENB_PORT    L2982_ENB_GPIO_Port


//#define VSS_HALL_SENSOR_INTERRUPT_PIN PB4


#define DEFAULT_LKAS_STATE false    // whenever engaging openpilot, this is the default status for LKAS (false = no steering)
#define DEFAULT_SET_SPEED 90        // the default set speed for acc when the (input) ECU is powered up


// SAFETY
#define CLUTCH_RELEASE_GRACE_TIME_MS 500    // amount of ms to delay before pulling throttle after shifting for example
#define BRAKE_RELEASE_GRACE_TIME_MS  50     // amount of ms to delay before pulling throttle after braking either automatically or manually
#define GAS_RELEASE_GRACE_TIME_MS  50       // amount of ms to delay before allowing auto-brake after manually pressing the gas/throttle pedal


// OPENPILOT / TOYOTA MESSAGE CONFIG 
#define OP_MAX_GAS_COMMAND 2000.0f //the max value which comes from OP on CAN ID 0x200 (actually 2074, it's being clipped)
#define OP_MIN_GAS_COMMAND 0.0f //the min value which comes from OP on CAN ID 0x200

#define OP_IGNORE_BRAKE_COMMAND_BELOW 0.5f // values below this are not treated as brake cmd (for example because engine braking is sufficient) (extracted from 0x343 GAS_CMD message in m/s)
#define OP_MIN_BRAKE_COMMAND 0.5f   // this is no braking (extracted from 0x343 GAS_CMD message in m/s)
#define OP_MAX_BRAKE_COMMAND 2.0f // this is full braking (extracted from 0x343 GAS_CMD message in m/s)



#define DISPLAY_OFFSET_X 0
#define DISPLAY_OFFSET_Y 5
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 56
#define DISPLAY_BORDER 0


#endif