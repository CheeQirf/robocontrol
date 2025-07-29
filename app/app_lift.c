#include "app_lift.h"

#include "app_can.h"

Lift_t Lift;

int lift_init(Lift_t *lift){

    for(int i = 0 ;  i < 4 ; ++i)
    {
        lift->motors[i].measure = get_motor_lift_measure_ptr(i);

    }
    
    lift->status = LIFT_INIT;
    return lift_update_data(lift);
}

int lift_calibrate(Lift_t *lift){

}

int lift_check(Lift_t *lift){
    


}

int lift_update_data(Lift_t* lift){
    //TODO 2 用编码器数据更新 lift位置


}

int lift_control(Lift_t * lift)
{

}

void lift_debug(Lift_t *lift)
{

}