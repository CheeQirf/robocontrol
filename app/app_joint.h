#ifndef __APP_JOINT
#define __APP_JOINT

#include "app_motor.h"
#include "stdbool.h"
typedef enum
{
    JOINT_OK = 0,
    JOINT_CALIBRATING,
    JOINT_MOVING,
    JOINT_HOLD,
    JOINT_ERROR_ANGLE,
    JOINT_ERROR_COLLAPSE,
    JOINT_INIT,
    JOINT_DEBUG,
    JOINT_CONTROL,

} JointStatus_t;

typedef struct
{
    // UnitreeMotor_Measure_t motor[2];
    UnitreeMotor_t motor[2];
    JointStatus_t status;

    float current_angle;

    float angular_velocity;

    float set_angle;
    float set_angular_velocity;

    bool calibrated;

} Joint_t;

int joint_init(Joint_t *joint);
int joint_calibrate(Joint_t *joint);
int joint_update_data(Joint_t *joint);
int joint_check(Joint_t *joint);

int joint_control(Joint_t *joint);

void joint_debug(Joint_t *joint);

#endif
// 当前角度 逆时针为正 弧度制
// 角速度 逆时针为证