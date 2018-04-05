#include <stdlib.h>
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

/* You can define the ports used here */
#define COLOR_PORT NXT_PORT_S1
#define SONAR_PORT NXT_PORT_S3
#define TOUCH_PORT NXT_PORT_S4

#define STEER_MOTOR NXT_PORT_A
#define  LEFT_MOTOR NXT_PORT_B
#define RIGHT_MOTOR NXT_PORT_C

DeclareCounter(SysTimerCnt);
DeclareTask(BackgroundAlways);
DeclareTask(Display);

DeclareTask(ReadSensors);
DeclareTask(MotorControl);

DeclareEvent(StartMotorEvent);
DeclareEvent(StopMotorEvent);

int cnt = 0;
U16 light_sum = 0;
U16 sonar_sum = 0;
U16 sensor_cnt = 0;

U8 touch_debounce = 0;

//----------------------------------------------------------------------------+
// nxtOSEK hooks                                                              |
//----------------------------------------------------------------------------+
void ecrobot_device_initialize() {
	ecrobot_init_nxtcolorsensor(COLOR_PORT, NXT_LIGHTSENSOR_NONE);
	ecrobot_init_sonar_sensor(SONAR_PORT);
}
void ecrobot_device_terminate() {
	ecrobot_term_nxtcolorsensor(COLOR_PORT);
	ecrobot_term_sonar_sensor(SONAR_PORT);
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
	U16 light = light_sum/sensor_cnt;
	U16 sonar = sonar_sum/sensor_cnt;
	sonar_sum = light_sum = sensor_cnt = 0;
	
	display_clear(0);
	display_goto_xy(0, 0);
	display_string("Devin and John\n");
	display_string("Clock:\n   ");
	display_int(cnt, 7);
	display_string("\nLight sensor:\n    ");
	display_int(light, 5);
	display_string("\nSonar sensor:\n    ");
	display_int(sonar, 5);
	display_update();

	TerminateTask();
}

//----------------------------------------------------------------------------+
// TestColorSensor periodic every 10ms, priority 4                            |
//----------------------------------------------------------------------------+
TASK(ReadSensors) {
	// Increment the displayed clock
	cnt++;
	
	// Increment the counter used to average readings in the display task
	sensor_cnt++;
	
	// Read the light sensor
	light_sum += ecrobot_get_nxtcolorsensor_light(COLOR_PORT);
	
	// Read the proximity sensor
	S32 sonar = ecrobot_get_sonar_sensor(SONAR_PORT);
	sonar_sum += sonar;
	
	if (sonar > 7) {
		SetEvent(MotorControl, StopMotorEvent);
	}
	
	// Read the touch sensor
	U8 touch = ecrobot_get_touch_sensor(TOUCH_PORT);
	if (touch != touch_debounce) {
		if (touch) {
			SetEvent(MotorControl, StartMotorEvent);
		}
		else {
			SetEvent(MotorControl, StopMotorEvent);
		}
		
		touch_debounce = touch;
	}
	
	TerminateTask();
}

//----------------------------------------------------------------------------+
// MotorControl aperiodic task while(1), event-driven, priority 3             |
//----------------------------------------------------------------------------+
//#define SPEEDRUN
TASK(MotorControl) {
#ifdef SPEEDRUN
	nxt_motor_set_speed(LEFT_MOTOR, -100, 0);
	nxt_motor_set_speed(RIGHT_MOTOR, -100, 0);
#else
	while(1) {
		WaitEvent(StartMotorEvent | StopMotorEvent);
		
		EventMaskType eMask = 0;
		GetEvent(MotorControl, &eMask);
		
		if (eMask & StartMotorEvent) {
			ClearEvent(StartMotorEvent);
			
			nxt_motor_set_speed(LEFT_MOTOR, -75, 0);
			nxt_motor_set_speed(RIGHT_MOTOR, -75, 0);
		}
		
		if (eMask & StopMotorEvent) {
			ClearEvent(StopMotorEvent);
			
			nxt_motor_set_speed(LEFT_MOTOR, 0, 1);
			nxt_motor_set_speed(RIGHT_MOTOR, 0, 1);
		}
	}
#endif
	TerminateTask();
}
