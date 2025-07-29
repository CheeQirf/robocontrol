#ifndef __APP_ROBOT
#define __APP_ROBOT

#include "app_arm.h"
#include "app_lift.h"
#include "app_joint.h"


typedef enum{

    ROBOT_INITED,
    ROBOT_CALIBRATE,
    ROBOT_CONTROL,
    ROBOT_LOCK,
    ROBOT_FREE,
    ROBOT_ERROR,
    ROBOT_DEBUG

}RobotStatus_t;


typedef struct{
    Arm_t* arm;
    Lift_t* lift;
    Joint_t* joint;
    RobotStatus_t last_status;
    RobotStatus_t status;
    bool calibrated;

}Robot_t;

int robot_init(Robot_t *robot);

int robot_calibrate(Robot_t *robot);

int robot_control(Robot_t *robot);




#endif