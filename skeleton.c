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

typedef struct {
	int min;
	int max;
	int sum;
	int	cnt;
} DispStat;

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
DispStat steer = { 0 };
DispStat drive = { 0 };
DispStat light = { 0 };
DispStat sonar = { 0 };
volatile int debug = 0;

volatile bool on_line = true;
volatile bool obstacle = false;
volatile U32 line_rev_count = 0;

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
	LEFT_HARD = -75,
	LEFT_SOFT = -30,
	LEFT_BUMP = -20,
	STRAIGHT = 0,
	RIGHT_BUMP = 20,
	RIGHT_SOFT = 30,
	RIGHT_HARD = 75,
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
	int steer_avg = steer.sum / steer.cnt;
	int drive_avg = drive.max; // drive.sum / drive.cnt;
	int light_avg = light.sum / light.cnt;
	int sonar_avg = sonar.sum / sonar.cnt;
	steer.cnt = drive.cnt = light.cnt = sonar.cnt = 0;
	
	display_clear(0);
	display_goto_xy(0, 0);
	display_string("Devin and John");
	display_string("\nLight: ");
	display_int(light_avg, 7);
	display_string("\nSonar: ");
	display_int(sonar_avg, 7);
	display_string("\nSteer: ");
	display_int(steer_avg, 7);
	display_string("\nDrive: ");
	display_int(drive_avg, 7);
	display_string("\nDebug: ");
	display_int(debug, 7);
	display_update();

	TerminateTask();
}

inline void RecordStat(DispStat* stat, int val) {
	stat->sum += val;
	if (stat->cnt++ == 0) {
		stat->min = val;
		stat->max = val;
		stat->sum = val;
	}
	else if (val < stat->min) {
		stat->min = val;
	}
	else if (val > stat->max) {
		stat->max = val;
	}
}

//----------------------------------------------------------------------------+
// TestColorSensor periodic every 50ms, priority 3                            |
//----------------------------------------------------------------------------+
TASK(ReadSensors) {
	// Read the light sensor
	U16 light_now = ecrobot_get_nxtcolorsensor_light(COLOR_PORT);
	RecordStat(&light, light_now);
	
	// Read the proximity sensor
	S32 sonar_now = ecrobot_get_sonar_sensor(SONAR_PORT);
	RecordStat(&sonar, sonar_now);
	
	// Read the steer angle
	int steer_now = nxt_motor_get_count(STEER_MOTOR);
	RecordStat(&steer, steer_now);
	
	// Read the drive revolution count
	int drive_now = nxt_motor_get_count(LEFT_MOTOR);
	RecordStat(&drive, drive_now);
	
	if (light_now < 250) {
		line_rev_count = drive_now;
		if (!on_line) {
			on_line = true;
			SetEvent(LineFollower, LineUpdateEvent);
		}
	}
	else if (on_line) {
		on_line = false;
		SetEvent(LineFollower, LineUpdateEvent);
	}
	
	if (sonar_now < 30) {
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
			if (delta_angle < 2) {
				nxt_motor_set_speed(STEER_MOTOR, STOPPED, 1);
				
				if (steer_signal_event) {
					steer_signal_event = false;
					SetEvent(LineFollower, SteerCompleteEvent);
				}
			}
			
			// Adjust the steering motor
			else {
				int speed = 60;
				if (delta_angle > 15) {
					speed = 70;
				}
				if (delta_angle > 30) {
					speed = 80;
				}
				
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
	steer_signal_event = false;
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

//inline void SearchAngle(

//----------------------------------------------------------------------------+
// LineFollower aperiodic task while(1), event-driven, priority 4             |
// Assumptions:                                                               |
//   1: The wheels are straight to begin with                                 |
//   2: The car is over the line to begin with (and relatively straight)      |
//----------------------------------------------------------------------------+
TASK(LineFollower) {
	int angle = STRAIGHT;
	
	while (1) {
		debug = 1;
		velocity = FORWARD * SPEED_4;
		SetEvent(MotorControl, AdjustMotorEvent);
		
		WaitEvent(LineUpdateEvent);
		ClearEvent(LineUpdateEvent);

		debug = 2;
		
		SetEvent(MotorControl, StopMotorEvent);
		
		if (!on_line) {
			debug = 3;

			int seek_angle;
			int iteration = 0;
			while (1) {
				debug = 4;
				++iteration;
				
				seek_angle = angle + iteration * LEFT_BUMP;
				if (seek_angle < LEFT_HARD) {
					seek_angle = LEFT_HARD; 
				}
				Steer(seek_angle);
				debug = 4;
				
				if (DriveForTime(SPEED_4, FORWARD, 30)) {
					angle = seek_angle;
					break;
				}
				if (DriveForTime(SPEED_4, REVERSE, 31)) {
					break;
				}
				
				seek_angle = angle + iteration * RIGHT_BUMP;
				if (seek_angle > RIGHT_HARD) {
					seek_angle = RIGHT_HARD; 
				}
				Steer(seek_angle);
				debug = 5;
				
				if (DriveForTime(SPEED_4, FORWARD, 30)) {
					angle = seek_angle;
					break;
				}
				if (DriveForTime(SPEED_4, REVERSE, 31)) {
					break;
				}
			}
			
		}
	}
	
	TerminateTask();
}
