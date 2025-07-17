#include "app_init.h"
#include "elog.h"
#include "hrtimer.h"
#include "main.h"
int app_init(void)
{
	//log init
	HAL_Delay(1000);
	int ret =0;
	ret = my_log_init();
	
	//hrtimer init
	ret = hrtimer_init(); 
	
	
	return ret;
	
	
	
}