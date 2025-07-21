#ifndef __APP_ROBOT
#define __APP_ROBOT

#include "app_arm.h"
#include "app_lift.h"
#include "app_joint.h"

typedef struct{
    Arm_t* arm;
    Lift_t* lift;
    Joint_t* joint;
    
   

    bool calibrated;

}Robot_t;






#endif