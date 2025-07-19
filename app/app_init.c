

#include "app_init.h"

#define LOG_TAG "APP_INIT"
#include "elog.h"
#include "hrtimer.h"
#include "main.h"
#include "cm_backtrace.h"
#include "bsp_uart.h"
#include "bsp_can.h"

#define HARDWARE_VERSION               "V1.0.0"
#define SOFTWARE_VERSION               "V0.1.0"

int app_init(void)
{
	

	int ret =0;
	USART4_Init();//uart4 init
	CAN_Init();//can init
	ret = my_log_init();//log init
	ret = hrtimer_init(); 	//hrtimer init
	cm_backtrace_init("CmBacktrace",HARDWARE_VERSION, SOFTWARE_VERSION);//cm backtrace_init


	if(!ret){
		log_i("app init sucessfully!");
	}
	return ret;
	
	
	
}

