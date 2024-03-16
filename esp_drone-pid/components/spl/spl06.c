

#include <math.h>
#include "driver/i2c.h"
#include "spl06.h"
#include "filter.h"
#include "stdio.h"
#include "string.h"
#include "mpu6050.h"
/********************************************************************************
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * SPL06驱动代码
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
 ********************************************************************************/

#define P_MEASURE_RATE SPL06_MWASURE_16   // 每秒测量次数
#define P_OVERSAMP_RATE SPL06_OVERSAMP_64 // 过采样率
#define SPL06_PRESSURE_CFG (P_MEASURE_RATE << 4 | P_OVERSAMP_RATE)

#define T_MEASURE_RATE SPL06_MWASURE_16  // 每秒测量次数
#define T_OVERSAMP_RATE SPL06_OVERSAMP_8 // 过采样率
#define SPL06_TEMPERATURE_CFG (TEMPERATURE_EXTERNAL_SENSOR << 7 | T_MEASURE_RATE << 4 | T_OVERSAMP_RATE)

#define SPL06_MODE (SPL06_CONTINUOUS_MODE)

const uint32_t scaleFactor[8] = {524288, 1572864, 3670016, 7864320, 253952, 516096, 1040384, 2088960};

float alt_3, height;

typedef enum
{
    PRESURE_SENSOR,
    TEMPERATURE_SENSOR
} spl06Sensor_e;

typedef struct
{
    int16_t c0;
    int16_t c1;
    int32_t c00;
    int32_t c10;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;
} spl06CalibCoefficient_t;

spl06CalibCoefficient_t spl06Calib;

static uint8_t devAddr;
static bool isInit = false;

int32_t kp = 0;
int32_t kt = 0;
int32_t SPL06RawPressure = 0;
int32_t SPL06RawTemperature = 0;

// static void SPL06GetPressure(void);

uint8_t i2cdevWriteByte(uint8_t add, uint8_t reg, uint8_t data)
{
    esp_err_t error;
    // 创建I2C连接，返回连接句柄
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    // 写启动信号
    i2c_master_start(cmd);
    // 广播地址，并说明读写，并等待响应
    error = i2c_master_write_byte(cmd, (add << 1) | I2C_MASTER_WRITE, 1);
    if (error != ESP_OK)
        return 1;
    // 写需要写入数据的寄存器地址
    error = i2c_master_write_byte(cmd, reg, 1);
    if (error != ESP_OK)
        return 1;
    // 向上一步指定的寄存器写入数据。
    error = i2c_master_write_byte(cmd, data, 1);
    if (error != ESP_OK)
        return 1;
    // 停止
    i2c_master_stop(cmd);
    // 发送数据
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    // 删除连接
    i2c_cmd_link_delete(cmd);

    return 0;
}

uint8_t i2cdevReadByte(uint8_t add, uint8_t reg, uint8_t *res)
{

    esp_err_t error;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // 地址广播
    error = i2c_master_write_byte(cmd, (add << 1) | I2C_MASTER_WRITE, 1);
    if (error != ESP_OK)
        return 1;
    // 写将要读取的寄存器地址
    error = i2c_master_write_byte(cmd, reg, 1);
    if (error != ESP_OK)
        return 1;
    // 起始信号
    i2c_master_start(cmd);
    // 广播地址，并说明要读数据
    error = i2c_master_write_byte(cmd, (add << 1) | I2C_MASTER_READ, 1);
    if (error != ESP_OK)
        return 1;
    // 读上述寄存器数据
    error = i2c_master_read_byte(cmd, res, I2C_MASTER_LAST_NACK);
    if (error != ESP_OK)
        return 1;
    // 停止信号
    i2c_master_stop(cmd);
    // 发送数据
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);

    i2c_cmd_link_delete(cmd);
    return 0;
}

uint8_t i2cdevRead(uint8_t add, uint8_t reg, uint8_t *buf, uint8_t len)
{
    esp_err_t error;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    error = i2c_master_write_byte(cmd, (add << 1) | I2C_MASTER_WRITE, 1);
    if (error != ESP_OK)
        return 1;

    error = i2c_master_write_byte(cmd, reg, 1);
    if (error != ESP_OK)
        return 1;

    i2c_master_start(cmd);
    error = i2c_master_write_byte(cmd, (add << 1) | I2C_MASTER_READ, 1);
    if (error != ESP_OK)
        return 1;

    error = i2c_master_read(cmd, buf, len, I2C_MASTER_LAST_NACK);
    if (error != ESP_OK)
        return 1;

    i2c_master_stop(cmd);

    i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);

    i2c_cmd_link_delete(cmd);
    return 0;
}

void spl0601_get_calib_param(void)
{
    uint8_t buffer[SPL06_CALIB_COEFFICIENT_LENGTH] = {0};

    i2cdevRead(devAddr, SPL06_COEFFICIENT_CALIB_REG, buffer, SPL06_CALIB_COEFFICIENT_LENGTH);

    spl06Calib.c0 = (int16_t)buffer[0] << 4 | buffer[1] >> 4;
    spl06Calib.c0 = (spl06Calib.c0 & 0x0800) ? (spl06Calib.c0 | 0xF000) : spl06Calib.c0;

    spl06Calib.c1 = (int16_t)(buffer[1] & 0x0F) << 8 | buffer[2];
    spl06Calib.c1 = (spl06Calib.c1 & 0x0800) ? (spl06Calib.c1 | 0xF000) : spl06Calib.c1;

    spl06Calib.c00 = (int32_t)buffer[3] << 12 | (int32_t)buffer[4] << 4 | (int32_t)buffer[5] >> 4;
    spl06Calib.c00 = (spl06Calib.c00 & 0x080000) ? (spl06Calib.c00 | 0xFFF00000) : spl06Calib.c00;

    spl06Calib.c10 = (int32_t)(buffer[5] & 0x0F) << 16 | (int32_t)buffer[6] << 8 | (int32_t)buffer[7];
    spl06Calib.c10 = (spl06Calib.c10 & 0x080000) ? (spl06Calib.c10 | 0xFFF00000) : spl06Calib.c10;

    spl06Calib.c01 = (int16_t)buffer[8] << 8 | buffer[9];
    spl06Calib.c11 = (int16_t)buffer[10] << 8 | buffer[11];
    spl06Calib.c20 = (int16_t)buffer[12] << 8 | buffer[13];
    spl06Calib.c21 = (int16_t)buffer[14] << 8 | buffer[15];
    spl06Calib.c30 = (int16_t)buffer[16] << 8 | buffer[17];
}

void spl0601_rateset(spl06Sensor_e sensor, uint8_t measureRate, uint8_t oversamplRate)
{
    uint8_t reg;
    if (sensor == PRESURE_SENSOR)
    {
        kp = scaleFactor[oversamplRate];
        i2cdevWriteByte(devAddr, SPL06_PRESSURE_CFG_REG, measureRate << 4 | oversamplRate);
        if (oversamplRate > SPL06_OVERSAMP_8)
        {
            i2cdevReadByte(devAddr, SPL06_INT_FIFO_CFG_REG, &reg);
            i2cdevWriteByte(devAddr, SPL06_INT_FIFO_CFG_REG, reg | 0x04);
        }
    }
    else if (sensor == TEMPERATURE_SENSOR)
    {
        kt = scaleFactor[oversamplRate];
        i2cdevWriteByte(devAddr, SPL06_TEMPERATURE_CFG_REG, measureRate << 4 | oversamplRate | 0x80); // Using mems temperature
        if (oversamplRate > SPL06_OVERSAMP_8)
        {
            i2cdevReadByte(devAddr, SPL06_INT_FIFO_CFG_REG, &reg);
            i2cdevWriteByte(devAddr, SPL06_INT_FIFO_CFG_REG, reg | 0x08);
        }
    }
}

bool SPL06Init()
{
    uint8_t SPL06ID = 0;
    // if (isInit)
    //     return true;
    // I2Cx = i2cPort;
    devAddr = SPL06_I2C_ADDR;

    // vTaskDelay(50 / portTICK_RATE_MS);

    i2cdevReadByte(devAddr, SPL06_CHIP_ID, &SPL06ID); /* 读取SPL06 ID*/

    if (SPL06ID == SPL06_DEFAULT_CHIP_ID)
        printf("SPL06 ID IS: 0x%X\n", SPL06ID);
    else
        return false;

    // 读取校准数据
    spl0601_get_calib_param();
    spl0601_rateset(PRESURE_SENSOR, SPL06_MWASURE_16, SPL06_OVERSAMP_64);
    spl0601_rateset(TEMPERATURE_SENSOR, SPL06_MWASURE_16, SPL06_OVERSAMP_64);

    i2cdevWriteByte(devAddr, SPL06_MODE_CFG_REG, SPL06_MODE);

    // isInit = true;
    return true;
}

void SPL06GetPressure(void)
{
    uint8_t data[SPL06_DATA_FRAME_SIZE];

    i2cdevRead(devAddr, SPL06_PRESSURE_MSB_REG, data, SPL06_DATA_FRAME_SIZE);
    SPL06RawPressure = (int32_t)data[0] << 16 | (int32_t)data[1] << 8 | (int32_t)data[2];
    SPL06RawPressure = (SPL06RawPressure & 0x800000) ? (0xFF000000 | SPL06RawPressure) : SPL06RawPressure;

    SPL06RawTemperature = (int32_t)data[3] << 16 | (int32_t)data[4] << 8 | (int32_t)data[5];
    SPL06RawTemperature = (SPL06RawTemperature & 0x800000) ? (0xFF000000 | SPL06RawTemperature) : SPL06RawTemperature;
}

float spl0601_get_temperature(int32_t rawTemperature)
{
    float fTCompensate;
    float fTsc;

    fTsc = rawTemperature / (float)kt;
    fTCompensate = spl06Calib.c0 * 0.5 + spl06Calib.c1 * fTsc;
    return fTCompensate;
}

float spl0601_get_pressure(int32_t rawPressure, int32_t rawTemperature)
{
    float fTsc, fPsc;
    float qua2, qua3;
    float fPCompensate;

    fTsc = rawTemperature / (float)kt;
    fPsc = rawPressure / (float)kp;
    qua2 = spl06Calib.c10 + fPsc * (spl06Calib.c20 + fPsc * spl06Calib.c30);
    qua3 = fTsc * fPsc * (spl06Calib.c11 + fPsc * spl06Calib.c21);
    // qua3 = 0.9f *fTsc * fPsc * (spl06Calib.c11 + fPsc * spl06Calib.c21);

    fPCompensate = spl06Calib.c00 + fPsc * qua2 + fTsc * spl06Calib.c01 + qua3;
    // fPCompensate = spl06Calib.c00 + fPsc * qua2 + 0.9f *fTsc  * spl06Calib.c01 + qua3;
    return fPCompensate;
}

// 参数：com 为采样的原始数值
// 返回值：iData 经过一阶滤波后的采样值
float lowV(float com)
{
    static float iLastData;                            // 上一次值
    float iData;                                       // 本次计算值
    float dPower = 0.1;                                // 滤波系数
    iData = (com * dPower) + (1 - dPower) * iLastData; // 计算
    iLastData = iData;                                 // 存贮本次数据
    return iData;                                      // 返回数据
}

void SPL06GetData(float *pressure, float *temperature, float *asl)
{
    static float t;
    static float p;

    SPL06GetPressure();

    t = spl0601_get_temperature(SPL06RawTemperature);
    p = spl0601_get_pressure(SPL06RawPressure, SPL06RawTemperature);

    //	pressureFilter(&p,pressure);
    *temperature = (float)t; /*单位度*/
    *pressure = (float)p;    /*单位hPa*/

    *pressure = lowV(*pressure);
    *asl = SPL06PressureToAltitude(*pressure); /*转换成海拔*/
}

/**
 * Converts pressure to altitude above sea level (ASL) in meters
 */

float SPL06PressureToAltitude(float pressure /*, float* groundPressure, float* groundTemp*/)
{
    alt_3 = (101000 - pressure) / 1000.0f;
    height = 0.82f * alt_3 * alt_3 * alt_3 + 0.09f * (101000 - pressure) * 100.0f;
    return height;
}

float asl_offset = 0;

// 获取初始高度
uint8_t get_offset_start(void)
{
    asl_offset=0;
    float pressure, temperature, asl;

    for (uint16_t i = 0; i < 400; i++)
    {
        SPL06GetData(&pressure, &temperature, &asl);
        if (i > 199)
        {
            SPL06GetData(&pressure, &temperature, &asl);
            asl_offset += asl;
            // printf("offset_alt:%f,%f\n", asl_offset, asl);
        }
    }
    // printf("offset_alt:%f\n",asl_sum/200.0f);
    asl_offset = asl_offset / 200.0f;

    return 1 ;
}

// 获取初始高度
uint8_t get_offset(void)
{
    float pressure, temperature, asl;

    for (uint16_t i = 0; i < 20; i++)
    {
        SPL06GetData(&pressure, &temperature, &asl);
        asl_offset += asl;
        // printf("offset_alt:%f,%f\n", asl_offset, asl);
    }
    // printf("offset_alt:%f\n",asl_sum/200.0f);
    asl_offset = asl_offset / 20.0f;

    return 1;

}

/*
基础单位：cm
*/

TickType_t xLastWakeTimestart = 0, xLastWakeTimeend = 0, dt = 0;

int32_t baro_speed_o, baro_speed;

#define safe_div(numerator, denominator, safe_value) ((denominator == 0) ? (safe_value) : ((numerator) / (denominator)))
#define LPF_1_(hz, t, in, out) ((out) += (1 / (1 + 1 / ((hz)*6.28f * (t)))) * ((in) - (out)))
#define ABS(x) ((x) > 0 ? (x) : -(x))
#define LIMIT(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

#define MONUM 10
float speed_av_arr[MONUM];
float speed_av;
uint16_t speed_av_cnt;

float speed_delta;
float speed_delta_lpf;
uint8_t baro_start = 1;

// 滑动滤波
void Moving_Average(float moavarray[], uint16_t len, uint16_t *fil_cnt, float in, float *out)
{
    uint16_t width_num;
    float last;

    width_num = len;

    if (++*fil_cnt >= width_num)
    {
        *fil_cnt = 0; // now
    }

    last = moavarray[*fil_cnt];

    moavarray[*fil_cnt] = in;

    *out += (in - (last)) / (float)(width_num);
    *out += 0.00001f * LIMIT((in - *out), -1, 1); //????????
}

float baro_height = 0, baro_Offset;
float spl_vcc = 0;
float baro_height_old = 0;

#define safe_div(numerator, denominator, safe_value) ((denominator == 0) ? (safe_value) : ((numerator) / (denominator)))
#define LPF_1_(hz, t, in, out) ((out) += (1 / (1 + 1 / ((hz)*6.28f * (t)))) * ((in) - (out)))

#define MONUM 10

extern alt_bro ALT_BRO;

void Height_Get(void) // 取得高度
{
    float pressure, temperature, asl;
    // 读取气压计高度
    SPL06GetData(&pressure, &temperature, &asl);
    ALT_BRO.h = asl - asl_offset;
    // printf("height:%f\n", alt);
}

void Height_vcc_Get(float dT)
{
    float delta_h,heigh_vcc;
    static float last_h=0,last_v=0;
    delta_h=ALT_BRO.h-last_h;
    heigh_vcc= delta_h/dT;
    heigh_vcc=0.3f*heigh_vcc+0.7f*last_v;
    ALT_BRO.v=heigh_vcc;
    last_h=ALT_BRO.h;
    last_v=heigh_vcc;
    
    // // 气压高度清零
    // if (baro_start)
    // {
    //     baro_height = 0;
    //     baro_height_old = 0;
    //     SPL06GetData(&pressure, &temperature, &asl);
    //     baro_Offset = asl;
    //     baro_start = 0;
    // }
    // else
    // {
    //     // 读取气压计高度
    //     SPL06GetData(&pressure, &temperature, &asl);
    //     baro_height = asl - baro_Offset;
    //     baro_speed_o = safe_div(baro_height - baro_height_old, dT, 0);
    //     baro_height_old = baro_height;
    // }

    //	//计算速度
    // Moving_Average(speed_av_arr,MONUM ,&speed_av_cnt,baro_speed_o,&speed_av);//滑动滤波
    // // vcc+=speed_av;

    // speed_delta = LIMIT(speed_av - baro_speed,-2000*dT,2000*dT);
    // LPF_1_(0.3f, dT, baro_speed_o, speed_delta_lpf);
    // ALT_BRO.v = baro_speed_o;
    // baro_speed += speed_delta *LIMIT((ABS(speed_delta_lpf)/(2000*dT)),0,1);
    // // speed_delta_lpf=lowV(speed_av);
    //  printf("vcc:%f\n",speed_delta_lpf);
    //  return speed_delta_lpf;
    // printf("vcc:%f,%f,%f,%d\n",speed_av,speed_delta,speed_delta_lpf,baro_speed);
}