#include "app_init.h"
#include "elog.h"
#include "hrtimer.h"
#include "main.h"
#include "cm_backtrace.h"


#define HARDWARE_VERSION               "V1.0.0"
#define SOFTWARE_VERSION               "V0.1.0"

int app_init(void)
{
	//log init
	HAL_Delay(1000);
	int ret =0;
	ret = my_log_init();
	
	//hrtimer init

	ret = hrtimer_init(); 
	cm_backtrace_init("CmBacktrace",HARDWARE_VERSION, SOFTWARE_VERSION);
	
	
	if(!ret){
		log_i("app init sucessfully!");
	}
	return ret;
	
	
	
}

