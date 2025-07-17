
#include "hrtimer.h"
#include "tim.h"

/*
 * Period of the free-running counter, in microseconds.
 */
#define HRT_COUNTER_PERIOD 65536

/*
 * Scaling factor(s) for the free-running counter; convert an input
 * in counts to a time in microseconds.
 */
#define HRT_COUNTER_SCALE(_c) (_c)

/*
 * HRT clock must be a multiple of 1MHz greater than 1MHz
 */
#if (HRT_TIMER_CLOCK % 1000000) != 0
#   error HRT_TIMER_CLOCK must be a multiple of 1MHz
#endif

static volatile uint64_t _timebase_us = 0;

extern TIM_HandleTypeDef htim13;


void TIM8_UP_TIM13_IRQHandler(void) {



    /* TIM Update event */
    if (__HAL_TIM_GET_FLAG(&htim13, TIM_FLAG_UPDATE) != RESET) {
        if (__HAL_TIM_GET_IT_SOURCE(&htim13, TIM_IT_UPDATE) != RESET) {
            _timebase_us += HRT_COUNTER_PERIOD;
           // hrt_call_isr(1);
            __HAL_TIM_CLEAR_IT(&htim13, TIM_IT_UPDATE);
#if (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
            htim->PeriodElapsedCallback(htim);
#else
            HAL_TIM_PeriodElapsedCallback(&htim13);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
        }
    }

}

hrt_abstime hrt_absolute_time(void) {
    return _timebase_us + __HAL_TIM_GET_COUNTER(&htim13);
}

/**
 * Initialise the timer we are going to use.
 *
 * We expect that we'll own one of the reduced-function STM32 general
 * timers, and that we can use channel 1 in compare mode.
 */
int hrtimer_init()
{
	    HAL_TIM_Base_Start_IT(&htim13);
	return 0;
}
