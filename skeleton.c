#include <stdlib.h>
#include <stdbool.h>
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

/* You can define the ports used here */
#define COLOR_PORT NXT_PORT_S1
#define SONAR_PORT NXT_PORT_S4

#define STEER_MOTOR NXT_PORT_A
#define  LEFT_MOTOR NXT_PORT_B
#define RIGHT_MOTOR NXT_PORT_C

DeclareCounter(SysTimerCnt);

DeclareTask(BackgroundAlways);
DeclareTask(Display);
DeclareTask(ReadSensors);
DeclareTask(MotorControl);
DeclareTask(LineFollower);

DeclareEvent(AdjustMotorEvent);
DeclareEvent(SteerMotorEvent);
DeclareEvent(StopMotorEvent);

DeclareEvent(LineUpdateEvent);
DeclareEvent(ObjectDetectedEvent);
DeclareEvent(TurnCompleteEvent);
DeclareEvent(DriveCompleteEvent);
DeclareEvent(SteerCompleteEvent);

// Global variables used to calculate averages that are displayed
int cnt = 0;
int angle_sum = 0;
U16 light_sum = 0;
U16 sonar_sum = 0;
U16 sensor_cnt = 0;

bool on_line = true;
bool obstacle = false;
U32 line_rev_count = 0;

// Useful enums for managing the logic of the vehicle
enum DRIVE_DIRECTION {
	FORWARD = -1,
	REVERSE = 1,
};

enum SPEED {
	FASTEST = 100,
	SLOWEST = 60,
	STOPPED = 0,
	
	SPEED_0 = 60,
	SPEED_0_5 = 65,
	SPEED_1 = 70,
	SPEED_2 = 80,
	SPEED_3 = 90,
	SPEED_4 = 100,
};

enum STEERING_ANGLE {
	LEFT_HARD = -60,
	LEFT_SOFT = -30,
	STRAIGHT = 0,
	RIGHT_SOFT = 30,
	RIGHT_HARD = 60,
};

// A controls set by the sensor task and read by the motor task
volatile int velocity = FORWARD * SPEED_0;

volatile bool steer_signal_event = false;
volatile int steer_target = 0;

volatile unsigned int drive_counter = 0;

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
	nxt_motor_set_speed(LEFT_MOTOR, STOPPED, 1);
	nxt_motor_set_speed(RIGHT_MOTOR, STOPPED, 1);
	nxt_motor_set_speed(STEER_MOTOR, STOPPED, 1);
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
	display_int(steer_target, 7);
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
	//sonar = ecrobot_get_sonar_sensor(SONAR_PORT);
	sonar_sum += sonar;
	
	// Read the angle
	int angle = nxt_motor_get_count(STEER_MOTOR);
	angle_sum += angle;
	
	if (light < 250) {
		line_rev_count = nxt_motor_get_count(LEFT_MOTOR);
		if (!on_line) {
			on_line = true;
			SetEvent(LineFollower, LineUpdateEvent);
		}
	}
	else if (on_line) {
		on_line = false;
		SetEvent(LineFollower, LineUpdateEvent);
	}
	
	if (sonar < 30) {
		if (!obstacle) {
			obstacle = true;
			SetEvent(LineFollower, ObjectDetectedEvent);
		}
	}
	else if (obstacle) {
		obstacle = false;
	}
	
	TerminateTask();
}

//----------------------------------------------------------------------------+
// MotorControl aperiodic task while(1), event-driven, priority 5             |
//----------------------------------------------------------------------------+
TASK(MotorControl) {
	while(1) {
		WaitEvent(SteerMotorEvent | AdjustMotorEvent | StopMotorEvent);
		
		EventMaskType eMask = 0;
		GetEvent(MotorControl, &eMask);
		
		if (eMask & SteerMotorEvent) {
			ClearEvent(SteerMotorEvent);
			
			int current_angle = nxt_motor_get_count(STEER_MOTOR);
			int delta_angle = steer_target - current_angle;
			int direction = delta_angle < 0 ? -1 : 1;
			delta_angle *= direction;
			
			// Signal that steering is complete
			if (delta_angle == 0) {
				nxt_motor_set_speed(STEER_MOTOR, STOPPED, 1);
				
				if (steer_signal_event) {
					steer_signal_event = false;
					SetEvent(LineFollower, SteerCompleteEvent);
				}
			}
			
			// Adjust the steering motor
			else {
				int speed = delta_angle > 15 ? 75 : 50;
				nxt_motor_set_speed(STEER_MOTOR, speed * direction, 0);
			}
			
			// Signal the drive_counter timer when completed 
			if (drive_counter > 0) {
				if (--drive_counter == 0) {
					SetEvent(LineFollower, DriveCompleteEvent);
				}
			}
		}
		
		if (eMask & AdjustMotorEvent) {
			ClearEvent(AdjustMotorEvent);
			
			nxt_motor_set_speed(LEFT_MOTOR, velocity, 0);
			nxt_motor_set_speed(RIGHT_MOTOR, velocity, 0);
		}
		
		if (eMask & StopMotorEvent) {
			ClearEvent(StopMotorEvent);
			
			nxt_motor_set_speed(LEFT_MOTOR, STOPPED, 1);
			nxt_motor_set_speed(RIGHT_MOTOR, STOPPED, 1);
		}
	}
	
	TerminateTask();
}

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

//----------------------------------------------------------------------------+
// Steer: Does an in-place turn and returns out when finished turning         |
//----------------------------------------------------------------------------+
inline void Steer(int angle) {
	// Stop to simplify logic (should be a no-op but safety first)
	SetEvent(MotorControl, StopMotorEvent);
	
	// Clear any previous signal (should be a no-op but safety first)
	ClearEvent(SteerCompleteEvent);
	
	steer_target = angle;
	steer_signal_event = true;
	
	WaitEvent(SteerCompleteEvent);
	ClearEvent(SteerCompleteEvent);
}

//----------------------------------------------------------------------------+
// DriveForTime: Drive until finding the line or hitting the timeout          |
// returns true: If the line is found before the time runs out                |
//----------------------------------------------------------------------------+
inline bool DriveForTime(int speed, int direction, unsigned int timeout) {
	// Clear any previous driving command (should be a no-op but safety first)
	drive_counter = 0;
	ClearEvent(DriveCompleteEvent);
	
	// Set the global variables to drive_counter for the speed and duration desired
	velocity = speed * direction;
	drive_counter = timeout;
	
	// Get the motor's going
	SetEvent(MotorControl, AdjustMotorEvent);
	
	// Wait for the timer or line found
	while (1) {
		WaitEvent(DriveCompleteEvent | LineUpdateEvent);
		
		EventMaskType eMask = 0;
		GetEvent(LineFollower, &eMask);
		
		if (eMask & LineUpdateEvent && !on_line) {
			ClearEvent(LineUpdateEvent);
			continue; 
		}
		
		// Stop now that we've hit our timer or found the line
		SetEvent(MotorControl, StopMotorEvent);
		
		drive_counter = 0;
		ClearEvent(DriveCompleteEvent);
		ClearEvent(LineUpdateEvent);
		
		return eMask & LineUpdateEvent ? true : false;
	}
}

//----------------------------------------------------------------------------+
// LineFollower aperiodic task while(1), event-driven, priority 4             |
// Assumptions:                                                               |
//   1: The wheels are straight to begin with                                 |
//   2: The car is over the line to begin with (and relatively straight)      |
//----------------------------------------------------------------------------+
TASK(LineFollower) {
	//int course_state = START;
	
	velocity = SPEED_0_5 * FORWARD;
	SetEvent(MotorControl, AdjustMotorEvent);
	
	while (1) {
		WaitEvent(LineUpdateEvent);
		ClearEvent(LineUpdateEvent);
		
		if (!on_line) {
			DriveForTime(SPEED_0_5, FORWARD, 500);
		}
	}
	
	TerminateTask();
	return;
	while (1) {
		Steer(LEFT_SOFT);
		DriveForTime(SPEED_2, FORWARD, 10);
		DriveForTime(STOPPED, FORWARD, 20);
		
		Steer(STRAIGHT);
		DriveForTime(SPEED_2, FORWARD, 10);
		DriveForTime(STOPPED, FORWARD, 20);
		
		Steer(RIGHT_SOFT);
		DriveForTime(SPEED_2, FORWARD, 10);
		DriveForTime(STOPPED, FORWARD, 20);
		
		Steer(STRAIGHT);
		DriveForTime(SPEED_2, FORWARD, 10);
		DriveForTime(STOPPED, FORWARD, 20);
	}
	
	TerminateTask();
}
