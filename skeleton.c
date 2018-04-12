#include <stdlib.h>
#include <stdbool.h>
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

/* You can define the ports used here */
#define COLOR_PORT NXT_PORT_S1
#define SONAR_PORT NXT_PORT_S2

#define STEER_MOTOR NXT_PORT_A
#define  LEFT_MOTOR NXT_PORT_B
#define RIGHT_MOTOR NXT_PORT_C

DeclareCounter(SysTimerCnt);

DeclareTask(BackgroundAlways);
DeclareTask(Display);
DeclareTask(ReadSensors);
DeclareTask(MotorControl);

DeclareEvent(AdjustMotorEvent);
DeclareEvent(SteerMotorEvent);
DeclareEvent(StopMotorEvent);

// Global variables used to calculate averages that are displayed
int cnt = 0;
int angle_sum = 0;
U16 light_sum = 0;
U16 sonar_sum = 0;
U16 sensor_cnt = 0;

// Useful enums for managing the logic of the vehicle
enum COURSE_FSM {
	START           = 0,
	FIRST_CURVE     = 1,
	FIRST_DOTTED    = 2,
	FIRST_CORNER    = 3,
	OBSTACLE        = 4,
	SECOND_CORNER   = 5,
	SECOND_DOTTED   = 6,
	SECOND_CURVE    = 7,
	FINISH          = 8,
};

enum SPEED {
	FORWARD_FULL  = -100,
	FORWARD_HALF  = -75,
	FORWARD_STOP  = -50,
	COMPLETE_STOP = 0,
	REVERSE_STOP  = 50,
	REVERSE_HALF  = 75,
	REVERSE_FULL  = 100,
};

enum STEERING_ANGLE {
	LEFT_HARD = -60,
	LEFT_SOFT = -30,
	STRAIGHT = 0,
	RIGHT_SOFT = 30,
	RIGHT_HARD = 60,
};

// A controls set by the sensor task and read by the motor task
int speed = FORWARD_HALF;
int steer = 0;

//----------------------------------------------------------------------------+
// nxtOSEK hooks                                                              |
//----------------------------------------------------------------------------+
void ecrobot_device_initialize() {
	ecrobot_init_nxtcolorsensor(COLOR_PORT, NXT_LIGHTSENSOR_BLUE);
	ecrobot_init_sonar_sensor(SONAR_PORT);
}
void ecrobot_device_terminate() {
	ecrobot_term_nxtcolorsensor(COLOR_PORT);
	ecrobot_term_sonar_sensor(SONAR_PORT);
	nxt_motor_set_speed(LEFT_MOTOR, COMPLETE_STOP, 1);
	nxt_motor_set_speed(RIGHT_MOTOR, COMPLETE_STOP, 1);
	nxt_motor_set_speed(STEER_MOTOR, COMPLETE_STOP, 1);
}
void user_1ms_isr_type2() {
	StatusType ercd;
	ercd = SignalCounter(SysTimerCnt); /* Increment OSEK Alarm Counter */
	if (ercd != E_OK) {
		ShutdownOS(ercd);
	}
}

//----------------------------------------------------------------------------+
// BackgroundAlways aperiodic task while(1), priority 1                       |
//----------------------------------------------------------------------------+
TASK(BackgroundAlways) {
	while(1) {
		ecrobot_process_bg_nxtcolorsensor();
	}
}

//----------------------------------------------------------------------------+
// Display periodic every 500ms, priority 2                                   |
//----------------------------------------------------------------------------+
TASK(Display) {
	// Calculate the sum
	int angle = angle_sum / sensor_cnt;
	U16 light = light_sum / sensor_cnt;
	U16 sonar = sonar_sum / sensor_cnt;
	angle_sum = light_sum = sonar_sum = sensor_cnt = 0;
	
	display_clear(0);
	display_goto_xy(0, 0);
	display_string("Devin and John");
	display_string("\nClock: ");
	display_int(cnt, 7);
	display_string("\nLight: ");
	display_int(light, 7);
	display_string("\nSonar: ");
	display_int(sonar, 7);
	display_string("\nSteer: ");
	display_int(steer, 7);
	display_string("\nAngle: ");
	display_int(angle, 7);
	display_update();

	TerminateTask();
}

//----------------------------------------------------------------------------+
// TestColorSensor periodic every 50ms, priority 3                            |
//----------------------------------------------------------------------------+
TASK(ReadSensors) {
	// Increment the displayed clock
	cnt++;
	
	// Increment the counter used to average readings in the display task
	sensor_cnt++;
	
	// Read the light sensor
	U16 light = ecrobot_get_nxtcolorsensor_light(COLOR_PORT);
	light_sum += light;
	
	// Read the proximity sensor
	S32 sonar = ecrobot_get_sonar_sensor(SONAR_PORT);
	sonar_sum += sonar;
	
	// Read the angle
	int angle = nxt_motor_get_count(STEER_MOTOR);
	angle_sum += angle;
	
	// FSM Logic
	switch (cnt % 160) {
		case 1:
			steer = STRAIGHT;
			speed = FORWARD_FULL;
			SetEvent(MotorControl, AdjustMotorEvent);
			break;
		case 40:
			steer = LEFT_HARD;
			speed = FORWARD_HALF;
			SetEvent(MotorControl, AdjustMotorEvent);
			break;
		case 80:
			steer = RIGHT_HARD;
			break;
		case 120:
			steer = STRAIGHT;
			SetEvent(MotorControl, StopMotorEvent);
			break;
	}
	
	
	
	
	//if (light < 250) {
	//if (sonar < 30) {
	
	//SetEvent(MotorControl, AdjustMotorEvent);
	//SetEvent(MotorControl, StopMotorEvent);
	
	TerminateTask();
}

//----------------------------------------------------------------------------+
// MotorControl aperiodic task while(1), event-driven, priority 4             |
//----------------------------------------------------------------------------+
TASK(MotorControl) {
	while(1) {
		WaitEvent(SteerMotorEvent | AdjustMotorEvent | StopMotorEvent);
		
		EventMaskType eMask = 0;
		GetEvent(MotorControl, &eMask);
		
		if (eMask & SteerMotorEvent) {
			ClearEvent(SteerMotorEvent);
			
			int angle = nxt_motor_get_count(STEER_MOTOR);
			int delta_angle = steer - angle;
			
			int direction = delta_angle < 0 ? -1 : 1;
			delta_angle *= direction;
			
			if (delta_angle == 0) {
				nxt_motor_set_speed(STEER_MOTOR, COMPLETE_STOP, 1);
			}
			else {
				int speed = delta_angle > 15 ? 75 : 50;
				nxt_motor_set_speed(STEER_MOTOR, speed * direction, 0);
			}
		}
		
		if (eMask & AdjustMotorEvent) {
			ClearEvent(AdjustMotorEvent);
			
			nxt_motor_set_speed(LEFT_MOTOR, speed, 0);
			nxt_motor_set_speed(RIGHT_MOTOR, speed, 0);
		}
		
		if (eMask & StopMotorEvent) {
			ClearEvent(StopMotorEvent);
			
			nxt_motor_set_speed(LEFT_MOTOR, COMPLETE_STOP, 1);
			nxt_motor_set_speed(RIGHT_MOTOR, COMPLETE_STOP, 1);
		}
	}
	
	TerminateTask();
}
