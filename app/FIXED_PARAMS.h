#ifndef __FIXED_PARAMS
#define __FIXED_PARAMS
/*

下面这些是 机械参数 连杆长度等 用于计算使用

单位 长度 都是 米
    角度 都是 弧度
*/

//抬升机构 允许的 最大高度 高度基准面以4个3508电机安装位置或底盘平面 
const float LIFT_MAX_HEIGHT = 0.0;
//抬升机构 允许的 最小高度 ........
const float LIFT_MIN_HEIGHT = 0.0;
//抬升机构 校准零点 的高度
const float LIFT_CALIBRATE_HEIGHT = 0.0;
//机械臂伸缩的最大长度 从关节处 到 机械爪闭合中心
const float ARM_STRETCH_MAX_LENGTH = 0.0;
//机械臂完全缩进去的长度 也就是最小长度
const float ARM_STRETCH_MIN_LENGTH = 0.0;




#endif