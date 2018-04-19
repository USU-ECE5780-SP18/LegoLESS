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

typedef struct { int now; int min; int max; int sum; int cnt; } DispStat;

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
enum DRIVE_MAGNITUDE {
	FASTEST = 100,
	SLOWEST = 60,
	STOPPED = 0,
	
	SPEED_0 = 60,
	SPEED_1 = 70,
	SPEED_2 = 80,
	SPEED_3 = 90,
	SPEED_4 = 100,
};

enum STEER_DIRECTION {
	LEFT = -1,
	RIGHT = 1,
};
enum STEER_MAGNITUDE {
	STRAIGHT = 0,
	BUMP = 20,
	SOFT = 45,
	HARD = 75,
};

// A controls set by the sensor task and read by the motor task
volatile int velocity = FORWARD * SPEED_0;
volatile unsigned int drive_counter = 0;

volatile bool steer_signal_event = false;
volatile int target_angle = 0;

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
// BackgroundAlways - aperiodic task while(1), priority 1                     |
//----------------------------------------------------------------------------+
TASK(BackgroundAlways) {
	while(1) {
		ecrobot_process_bg_nxtcolorsensor();
	}
}

//----------------------------------------------------------------------------+
// Display - periodic every 500ms, priority 2                                 |
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

//----------------------------------------------------------------------------+
// RecordStat: Tracks {avg, min, max} over period of TASK(Display)            |
//----------------------------------------------------------------------------+
inline void RecordStat(DispStat* stat, int val) {
	stat->now = val;
	if (stat->cnt++ == 0) {
		stat->min = val;
		stat->max = val;
		stat->sum = val;
	}
	else {
		stat->sum += val;
		if (val < stat->min) {
			stat->min = val;
		}
		else if (val > stat->max) {
			stat->max = val;
		}
	}
}

//----------------------------------------------------------------------------+
// ReadSensors - periodic every 50ms, priority 3                              |
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
// GetVector: Small helper for separating magnitude from direction            |
//----------------------------------------------------------------------------+
typedef struct { int dir; int mag; } vector;
inline vector GetVector(int val) {
	if (val < 0) {
		vector v = { -1, -val };
		return v;
	}
	else {
		vector v = { 1, val };
		return v;
	}
}

//----------------------------------------------------------------------------+
// FollowLine: Drive until loosing the line or hitting the timeout (0 => inf) |
// returns true: If the line is lost before the time runs out                 |
//----------------------------------------------------------------------------+
inline bool FollowLine(int speed, int direction, unsigned int timeout) {
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
		
		if (eMask & LineUpdateEvent && on_line) {
			ClearEvent(LineUpdateEvent);
			continue; 
		}
		
		// Stop now that we've hit our timer or lost the line
		SetEvent(MotorControl, StopMotorEvent);
		
		drive_counter = 0;
		ClearEvent(DriveCompleteEvent);
		ClearEvent(LineUpdateEvent);
		
		return eMask & LineUpdateEvent ? true : false;
	}
}

//----------------------------------------------------------------------------+
// SeekLine: Drive until finding the line or hitting the timeout              |
// returns true: If the line is found before the time runs out                |
//----------------------------------------------------------------------------+
inline bool SeekLine(int speed, int direction, unsigned int timeout) {
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
// Steer: Does an in-place turn and returns out when finished turning         |
//----------------------------------------------------------------------------+
inline void Steer(int angle) {
	// Stop to simplify logic (should be a no-op but safety first)
	SetEvent(MotorControl, StopMotorEvent);
	
	// Clear any previous signal (should be a no-op but safety first)
	steer_signal_event = false;
	ClearEvent(SteerCompleteEvent);
	
	target_angle = angle;
	steer_signal_event = true;
	
	WaitEvent(SteerCompleteEvent);
	ClearEvent(SteerCompleteEvent);
}

//----------------------------------------------------------------------------+
// TestForward: Attempts to find the line, reverses the test if it fails      |
// returns true: If the line is found in the allotted time                    |
//----------------------------------------------------------------------------+
inline bool TestForward(int timeout) {
	if (SeekLine(SPEED_4, FORWARD, timeout)) {
		return true;
	}
	// Our car inches slightly forward given equal timeouts forward and back
	SeekLine(SPEED_4, REVERSE, ++timeout);
	return false;
}

//----------------------------------------------------------------------------+
// SymmetricFinder:                                                           |
// Tests increasing multiples of `bump` on alternating sides of `angle`       |
// returns true: If the line is found                                         |
//----------------------------------------------------------------------------+
bool SymmetricFinder(int* angle, int dir1, int bump, int maxit, int timeout) {
	int step = 0;
	int dir2 = -dir1;
	bool hard1 = false;
	bool hard2 = false;
	int seek_angle;
	
	while (1) {
		++step;
		if (maxit && step > maxit) { return false; }
		
		if (!hard1) {
			seek_angle = *angle + step * dir1 * bump;
			vector v = GetVector(seek_angle);
			if (v.mag >= HARD) {
				seek_angle = dir1 * HARD;
				hard1 = true;
			}
			
			Steer(seek_angle);
			if (TestForward(timeout)) {
				*angle = seek_angle;
				return true;
			}
		}
		
		if (!hard2) {
			seek_angle = *angle + step * dir2 * bump;
			vector v = GetVector(seek_angle);
			if (v.mag >= HARD) {
				seek_angle = dir2 * HARD; 
				hard2 = true;
			}
			
			Steer(seek_angle);
			if (TestForward(timeout)) {
				*angle = seek_angle;
				return true;
			}
		}
		
		if (hard1 && hard2) {
			return false;
		}
	}
}

//----------------------------------------------------------------------------+
// AsymmetricFinder                                                           |
// Tests increasing multiples of `bump` on the given side of `angle`          |
// returns true: If the line is found                                         |
//----------------------------------------------------------------------------+
bool AsymmetricFinder(int* angle, int dir, int bump, int maxit, int timeout) {
	int step = 0;
	bool hard = false;
	int seek_angle;
	
	while (1) {
		++step;
		if (maxit && step > maxit) { return false; }
		
		if (!hard) {
			seek_angle = *angle + step * dir * bump;
			vector v = GetVector(seek_angle);
			if (v.mag >= HARD) {
				seek_angle = dir * HARD;
				hard = true;
			}
			
			Steer(seek_angle);
			if (TestForward(timeout)) {
				*angle = seek_angle;
				return true;
			}
		}
		else {
			return false;
		}
	}
}

enum COURSE_FSM {
	START           = 0,
	FIRST_CURVE     = 1,
	
	FIRST_DOTTED    = 3,
	FIRST_CORNER    = 4,
	
	OBSTACLE        = 6,
	
	SECOND_CORNER   = 9,
	
	SECOND_DOTTED   = 11,
	SECOND_CURVE    = 12,
	
	FINISH          = 14,
};

// default left to right looking orientation
// multiply by -1 to reverse course_prediction
int course_prediction[15] = {
	BUMP,				//  0 => START
	
	LEFT * SOFT,		//  1 => FIRST_CURVE
	RIGHT * SOFT,		//  2 => END_FIRST_CURVE
	
	STRAIGHT,			//  3 => FIRST_DOTTED
	
	RIGHT * HARD,		//  4 => FIRST_CORNER
	LEFT * HARD,		//  5 => END_FIRST_CORNER
	
	RIGHT * HARD,		//  6 => OBSTACLE_AVERT
	LEFT * HARD,		//  7 => OBSTACLE_CIRCLE_AROUND
	RIGHT * HARD,		//  8 => OBSTACLE_STRAIGHTEN_BACK_ON_TRACK
	
	RIGHT * HARD,		//  9 => SECOND_CORNER
	LEFT * HARD,		// 10 => END_SECOND_CORNER
	
	STRAIGHT,			// 11 => SECOND_DOTTED
	
	LEFT * SOFT,		// 12 => SECOND_CURVE
	RIGHT * SOFT,		// 13 => END_SECOND_CURVE
	
	BUMP,				// 14 => FINISH
};

//----------------------------------------------------------------------------+
// LineFollower - aperiodic task while(1), event-driven, priority 4           |
// Assumptions:                                                               |
//   1: The wheels are straight to begin with                                 |
//   2: The car is over the line to begin with (and relatively straight)      |
//----------------------------------------------------------------------------+
TASK(LineFollower) {
	int angle = STRAIGHT;
	int angle_next = angle;
	U8 state = START;
	
	int drive_last = drive.now;
	int course_dir = LEFT;
	int turn_dir = LEFT;
	int bump_dir = LEFT;
	
	// Follow a straight line
	while (1) {
		drive_last = drive.now;
		FollowLine(SPEED_4, FORWARD, 0);
		int drive_delta = drive.now - drive_last;
		debug = drive_delta;
		
		if (SymmetricFinder(&angle_next, bump_dir, BUMP, 0, 30)) {
			vector delta = GetVector(angle_next - angle);
			angle = angle_next;
			
			// It is expected we'll wobble on alternating sides
			// Hopefully with a small ripple
			if (delta.dir == bump_dir) {
				bump_dir *= -1;
			}
			
			if (delta.mag >= SOFT) {
				break;
			}
		}
		else {
			TerminateTask();
			return;
		}
	}
	
	// Follow a curved line
	//while (1) {
		//FollowLine(SPEED_4, FORWARD, 0);
		
	//}
	
	TerminateTask();
}

//----------------------------------------------------------------------------+
// MotorControl - aperiodic task while(1), event-driven, priority 5           |
//----------------------------------------------------------------------------+
TASK(MotorControl) {
	while(1) {
		WaitEvent(SteerMotorEvent | AdjustMotorEvent | StopMotorEvent);
		
		EventMaskType eMask = 0;
		GetEvent(MotorControl, &eMask);
		
		if (eMask & SteerMotorEvent) {
			ClearEvent(SteerMotorEvent);
			
			int current_angle = nxt_motor_get_count(STEER_MOTOR);
			vector delta = GetVector(target_angle - current_angle);
			
			// Signal that steering is complete
			if (delta.mag < 2) {
				nxt_motor_set_speed(STEER_MOTOR, STOPPED, 1);
				
				if (steer_signal_event) {
					steer_signal_event = false;
					SetEvent(LineFollower, SteerCompleteEvent);
				}
			}
			
			// Adjust the steering motor
			else {
				int speed;
				if (delta.mag > 30) {
					speed = 80;
				}
				else if (delta.mag > 15) {
					speed = 70;
				}
				else {
					speed = 60;
				}
				
				nxt_motor_set_speed(STEER_MOTOR, speed * delta.dir, 0);
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
