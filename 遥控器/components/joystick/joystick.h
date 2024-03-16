#ifndef JOYSTICK_H
#define JOYSTICK_H

#define LETF_UP_DOWN_CHANNEL ADC1_CHANNEL_6
#define LETF_LEFT_RIGHT_CHANNEL ADC1_CHANNEL_4
#define RIGHT_UP_DOWN_CHANNEL ADC1_CHANNEL_7
#define RIGHT_LEFT_RIGHT_CHANNEL ADC1_CHANNEL_5

#include "nvs_flash.h"

// 结构体
typedef struct
{
  int16_t left_thr ;
  int16_t left_right ;
  int16_t right_up ;
  int16_t right_left ;
  int16_t thr ;
} key_offset;

void adc_Init(void);
int adc_left_up_down(void);
int adc_left_left_right(void);
int adc_right_up_down(void);
int adc_right_left_right(void);
void joystick(void *pvParameters);
void key_scan(void *pvParameters);
void led_message(void *pvParameters);


esp_err_t nvs_write_data(key_offset *Temp_st, uint32_t len);
esp_err_t nvs_read_data(key_offset *Temp_st, uint32_t *len);

#endif