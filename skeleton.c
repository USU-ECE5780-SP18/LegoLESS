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

int cnt = 0;
U16 av_cnt = 0;
U16 av_sum = 0;

//----------------------------------------------------------------------------+
// nxtOSEK hooks                                                              |
//----------------------------------------------------------------------------+
void ecrobot_device_initialize() {
	ecrobot_init_nxtcolorsensor(COLOR_PORT, NXT_LIGHTSENSOR_NONE);
}
void ecrobot_device_terminate() {
	ecrobot_term_nxtcolorsensor(COLOR_PORT);
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
	U16 av = av_sum/av_cnt;
	av_sum = av_cnt = 0;
	
	display_clear(0);
	display_goto_xy(0, 0);
	display_string("Devin and John\n");
	display_string("Light sensor:\n    ");
	display_int(av, 5);
	display_string("\nClock:\n   ");
	display_int(cnt, 7);
	display_update();

	TerminateTask();
}

//----------------------------------------------------------------------------+
// TestColorSensor periodic every 10ms, priority 4                            |
//----------------------------------------------------------------------------+
TASK(ReadSensors) {
	// Increment the displayed clock
	cnt++;
	
	// Read the light sensor
	av_sum += ecrobot_get_nxtcolorsensor_light(COLOR_PORT);
	av_cnt++;
	
	// Read the touch sensor
	
	TerminateTask();
}

//----------------------------------------------------------------------------+
// MotorControl aperiodic task while(1), event-driven, priority 3             |
//----------------------------------------------------------------------------+
#define SPEEDRUN
TASK(MotorControl) {
#ifdef SPEEDRUN
	nxt_motor_set_speed(LEFT_MOTOR, -100, 0);
	nxt_motor_set_speed(RIGHT_MOTOR, -100, 0);
#else
	
#endif
	TerminateTask();
}
