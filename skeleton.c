#include <stdlib.h>
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

/* You can define the ports used here */
#define COLOR_PORT_ID NXT_PORT_S1 
int cnt = 0;
U16 av_cnt = 0;
U16 av_sum = 0;

DeclareTask(TestColorSensor);
DeclareTask(DisplayData);
DeclareTask(ControlSensor);
DeclareCounter(SysTimerCnt);

/* nxtOSEK hooks */
void ecrobot_device_initialize() {
	ecrobot_init_nxtcolorsensor(COLOR_PORT_ID, NXT_LIGHTSENSOR_NONE);
}

void ecrobot_device_terminate() {
	ecrobot_term_nxtcolorsensor(COLOR_PORT_ID);
}

void user_1ms_isr_type2() {
  StatusType ercd;

  ercd = SignalCounter(SysTimerCnt); /* Increment OSEK Alarm Counter */
  if (ercd != E_OK)
  {
    ShutdownOS(ercd);
  }
}

TASK(ControlSensor)
{
	while(1){
		ecrobot_process_bg_nxtcolorsensor();
	}
}

TASK(TestColorSensor)
{
	av_sum += ecrobot_get_nxtcolorsensor_light(COLOR_PORT_ID);
	av_cnt++;
	cnt++;
	TerminateTask();
}

TASK(DisplayData)
{
	//should be 500ms
	U16 av = av_sum/av_cnt;
	av_sum = 0;
	av_cnt = 0;
	display_clear(0);
	display_goto_xy(0, 0);
	display_string("Devin and John\n");
	display_string("Light sensor:\n    ");
	display_int(av, 5);
	display_string("\nCount:\n   ");
	display_int(cnt, 6);
	display_update();

	TerminateTask();
}

