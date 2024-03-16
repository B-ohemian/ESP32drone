#include "altitude.h"
#include "spl06.h"
#include "imu.h"
#include "mpu6050.h"
#include "filter.h"

#define TIME_CONTANST_Z 5.0f
#define K_ACC_Z (5.0f / (TIME_CONTANST_Z * TIME_CONTANST_Z * TIME_CONTANST_Z))
#define K_VEL_Z (3.0f / (TIME_CONTANST_Z * TIME_CONTANST_Z))
#define K_POS_Z (3.0f / TIME_CONTANST_Z)

float High_Filter[3] = {
    100.0, // 0.015
    10.0,  // 0.05
    15.0   // 0.03
};

#define CNTLCYCLE 0.001f

cal_heigh CAL_HEIGH;

extern alt_bro ALT_BRO;
extern alt_acc ALT_ACC;

float acc_correction;
float vel_correction;
float vel_correction2;
float pos_correction;
float pos_correction2;

float SpeedDealt;

float Altitude_Dealt = 0;
float velocity_Dealt = 0;
float two_sensor_Altitude_Dealt = 0;
float last_hight = 0;
float last_v = 0;
float last_v2 = 0;
float last_highttemp = 0;
float highttemp = 0;
float vcctemp = 0;

void reset_bro() // 复位气压计的值
{
    CAL_HEIGH.h = 0.0;
    CAL_HEIGH.v = 0.0;
    CAL_HEIGH.a = 0.0;
    CAL_HEIGH.last_h = 0.0;

    ALT_BRO.h = 0.0;
    ALT_ACC.a = 0.0;
    ALT_ACC.v = 0.0;
    ALT_ACC.h = 0.0;
}

Butter_Parameter Butter_10HZ_Parameter_Acce = {
    // 200hz---10hz
    {1, -1.561018075801, 0.6413515380576},
    {0.02008336556421, 0.04016673112842, 0.02008336556421}};

Butter_Parameter Butter_5HZ_Parameter_Acce = {
    // 200hz---5hz
    {1, -1.778631777825, 0.8008026466657},
    {0.005542717210281, 0.01108543442056, 0.005542717210281}};

void Strapdown_INS_High(float hight)
{
    float temp = 0;
    hight = 0.8f * hight + 0.2f * last_hight;
    // LPButterworth(ALT_ACC.a, &Butter_Buffer, &Butter_5HZ_Parameter_Acce);

    // Altitude_Dealt = ALT_BRO.h - CAL_HEIGH.h; // 气压计(超声波)与SINS估计量的差，单位cm 观测器的高度-融合后的高度
    Altitude_Dealt = ALT_BRO.h - highttemp;
    velocity_Dealt = ALT_BRO.v - vcctemp;
    // 对高度误差进行积分，取不同的权重
    acc_correction += Altitude_Dealt * 0.0006; // 加速度校正量
    vel_correction += Altitude_Dealt * 0.006;  // 速度校正量
    vel_correction2 += velocity_Dealt * 0.03;  // 速度校正量
    pos_correction += Altitude_Dealt * 0.06;   // 位置校正量

    // //原始加速度+加速度校正量=融合后的加速度
    CAL_HEIGH.a = ALT_ACC.a + acc_correction;

    // //融合后的加速度乘以时间得到速度增量
    SpeedDealt = CAL_HEIGH.a * CNTLCYCLE;

    // //原始速度+速度校正量=融合后的速度
    ALT_ACC.v += SpeedDealt;
    vcctemp = ALT_ACC.v + vel_correction + vel_correction2;
    // CAL_HEIGH.v = ALT_ACC.v + vel_correction;

    vcctemp = 0.1 * vcctemp + 0.9 * last_v;
    ALT_BRO.v = 0.1 * ALT_BRO.v + 0.9 * last_v2;

    CAL_HEIGH.v = 0.8 * vcctemp + 0.2 * ALT_BRO.v;

    LPButterworth(vcctemp, &Butter_Buffer, &Butter_5HZ_Parameter_Acce);

    // //得到速度增量后，更新原始位置 h=v0*t+1/2v*t
    ALT_ACC.h += (vcctemp + 0.5 * SpeedDealt) * CNTLCYCLE;
    highttemp = ALT_ACC.h + pos_correction;
    // highttemp = 0.85 * highttemp + 0.15 * last_highttemp;

    CAL_HEIGH.h = 0.3 * highttemp + 0.7 * hight; // 气压计和超声波融合

    CAL_HEIGH.last_h = CAL_HEIGH.h;
    last_highttemp = highttemp;
    last_hight = hight;
    last_v = vcctemp;
    last_v2 = ALT_BRO.v;

    // printf("cal:%f,%f,%f,%f,%f,%f,%f,%f\n", CAL_HEIGH.a, CAL_HEIGH.v, CAL_HEIGH.h,
    //        ALT_BRO.h, ALT_BRO.v, hight, highttemp, vcctemp);
}

