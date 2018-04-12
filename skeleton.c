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
DeclareTask(LineFollower);

DeclareEvent(AdjustMotorEvent);
DeclareEvent(SteerMotorEvent);
DeclareEvent(StopMotorEvent);

DeclareEvent(LineFoundEvent);
DeclareEvent(LineLostEvent);
DeclareEvent(ObjectDetectedEvent);
DeclareEvent(TurnCompleteEvent);
DeclareEvent(DriveCompleteEvent);

// Global variables used to calculate averages that are displayed
int cnt = 0;
int angle_sum = 0;
U16 light_sum = 0;
U16 sonar_sum = 0;
U16 sensor_cnt = 0;

bool on_line = 0;
bool obstacle = 0;

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
volatile int steer = 0;
volatile unsigned int drive = 0;

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
	//sonar = ecrobot_get_sonar_sensor(SONAR_PORT);
	sonar_sum += sonar;
	
	// Read the angle
	int angle = nxt_motor_get_count(STEER_MOTOR);
	angle_sum += angle;
	
	// FSM Logic
	/*switch (cnt % 200) {
		case 1:
			velocity = FORWARD * SPEED_0;
			SetEvent(MotorControl, AdjustMotorEvent);

			steer = STRAIGHT;
			break;
		case 41:
			steer = RIGHT_HARD;
			break;
		case 61:
			steer = LEFT_HARD;
			break;
		case 81:
			steer = STRAIGHT;
			break;
		case 121:
			steer = LEFT_HARD;
			break;
		case 141:
			steer = RIGHT_HARD;
			break;
		case 161:
			steer = STRAIGHT;

			SetEvent(MotorControl, StopMotorEvent);
			break;
	}*/
	
	if (light < 250) {
		if (!on_line) {
			on_line = true;
			SetEvent(LineFollower, LineFoundEvent);
		}
	}
	else if (on_line) {
		on_line = false;
		SetEvent(LineFollower, LineLostEvent);
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
			int delta_angle = steer - current_angle;
			int direction = delta_angle < 0 ? -1 : 1;
			delta_angle *= direction;
			
			if (delta_angle == 0) {
				nxt_motor_set_speed(STEER_MOTOR, STOPPED, 1);
			}
			else {
				int speed = delta_angle > 15 ? 75 : 50;
				nxt_motor_set_speed(STEER_MOTOR, speed * direction, 0);
			}
			
			if (drive > 0) {
				if (--drive == 0) {
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
// LineFollower aperiodic task while(1), event-driven, priority 4             |
// Assumptions:                                                               |
//   1: The wheels are straight to begin with                                 |
//   2: The car is over the line to begin with (and relatively straight)      |
//----------------------------------------------------------------------------+
TASK(LineFollower) {
	int course_state = START;
	
	while (1) {
		drive = 10;
		velocity = FORWARD * SPEED_3;
		SetEvent(MotorControl, AdjustMotorEvent);
		
		WaitEvent(DriveCompleteEvent);
		ClearEvent(DriveCompleteEvent);
		
		SetEvent(MotorControl, StopMotorEvent);
		
		drive = 30;
		WaitEvent(DriveCompleteEvent);
		ClearEvent(DriveCompleteEvent);
	}
	
	TerminateTask();
	
	while(1) {
		WaitEvent(ObjectDetectedEvent | TurnCompleteEvent | LineFoundEvent | LineLostEvent);
		
		EventMaskType eMask = 0;
		GetEvent(LineFollower, &eMask);
		
		if (eMask & ObjectDetectedEvent) {
			ClearEvent(ObjectDetectedEvent);
			SetEvent(MotorControl, StopMotorEvent);
		}
		
		if (eMask & TurnCompleteEvent) {
			ClearEvent(TurnCompleteEvent);
		}
		
		if (eMask & LineFoundEvent) {
			ClearEvent(LineFoundEvent);
			
			velocity = FORWARD * SPEED_0;
			SetEvent(MotorControl, AdjustMotorEvent);
		}
		
		if (eMask & LineLostEvent) {
			//Perform line finding algorithm
			ClearEvent(LineLostEvent);
			
			// Make an in-place turn
			SetEvent(MotorControl, StopMotorEvent);
			
			// clear existing events so we in fact wait for the upcoming turn to finish
			ClearEvent(TurnCompleteEvent);
			steer = RIGHT_SOFT;
			
			// Wait for the turn
			WaitEvent(TurnCompleteEvent);
			ClearEvent(TurnCompleteEvent);

			// clear existing events so we in fact drive for the desired time
			drive = 0;
			ClearEvent(DriveCompleteEvent);
			
			// drive for 50ms
			drive = 5;
			SetEvent(MotorControl, AdjustMotorEvent);
			
			// Wait for the drive to finish
			WaitEvent(DriveCompleteEvent);
			ClearEvent(DriveCompleteEvent);
			
			SetEvent(MotorControl, StopMotorEvent);
			
		}
	}
	
	TerminateTask();
}
