#ifndef _PID_H
#define _PID_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpu6050.h"
// #include "motor.h"


#define DEFAULT_PID_INTEGRATION_LIMIT  300.0
#define YM_Dead 100

// //************外环pid参数**************************//
// #define PID_OUT_PITCH_KP  2.5 //2.5
// #define PID_OUT_PITCH_KI  0   //0
// #define PID_OUT_PITCH_KD  0   //0
// #define PID_OUT_PITCH_INTEGRATION_LIMIT   500.0 // 500.0

// #define PID_OUT_ROLL_KP  3    //3
// #define PID_OUT_ROLL_KI  0    //0
// #define PID_OUT_ROLL_KD  0    //0
// #define PID_OUT_ROLL_INTEGRATION_LIMIT    500.0 // 500.0


// //************内环pid参数**************************//
// #define PID_IN_PITCH_KP  0.5     //0.5
// #define PID_IN_PITCH_KI  0.003   //0.003
// #define PID_IN_PITCH_KD  0.25    //0.25
// #define PID_IN_PITCH_INTEGRATION_LIMIT   500.0 // 500.0

// #define PID_IN_ROLL_KP  0.3      //0.3
// #define PID_IN_ROLL_KI  0.003    //0.003
// #define PID_IN_ROLL_KD  0.20     //0.20
// #define PID_IN_ROLL_INTEGRATION_LIMIT    500.0 // 500.0

// #define PID_IN_YAW_KP   3        //3.0
// #define PID_IN_YAW_KI   0        //0
// #define PID_IN_YAW_KD   0        //0
// #define PID_IN_YAW_INTEGRATION_LIMIT   200.0 // 200.0

typedef struct
{
  float desired;      //< 被调量期望值
  float error;        //< 期望值-实际值
  float prevError;    //< 前一次偏差
  float integ;        //< 积分部分
  float deriv;        //< 微分部分
  float kp;           //< 比例参数
  float ki;           //< 积分参数
  float kd;           //< 微分参数
  float outP;         //< pid比例部分，调试用
  float outI;         //< pid积分部分，调试用
  float outD;         //< pid微分部分，调试用
  float iLimit;       //< 积分限制
} pidsuite;


typedef struct
{
  float out_rol_p; 
  float out_rol_i; 
  float out_rol_d; 
  float out_pit_p; 
  float out_pit_i; 
  float out_pit_d; 
  float in_rol_p;  
  float in_rol_i;  
  float in_rol_d;  
  float in_pit_p;  
  float in_pit_i;  
  float in_pit_d;  
  float in_yaw_p;  
  float in_yaw_i;  
  float in_yaw_d;  
} piddata;

extern uint8_t armed;

void PID_controllerInit(void);     //pid参数初始化
void Control(uint8_t armed);
float pidUpdate(pidsuite* pid, const float measured,float expect);
void pidSetIntegralLimit(pidsuite* pid, const float limit);
void pidInit(pidsuite* pid, const float desired, const float kp,
             const float ki, const float kd);
void argument_receive_task(void *arg);
void PID_Z(void);
int16_t estimateHoverThru(void);
void pid_change(uint8_t pid[96]);


#endif