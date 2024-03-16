#include "driver/ledc.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#define LED_GREEN_PIN 38
#define LED_BLUE_PIN 39
#define LED_GREEN_PIN2 36
#define LED_BLUE_PIN2 35

// #define LED_GREEN_PIN 46
// #define LED_BLUE_PIN 42




//定义LED状态
#define LED_ON          1   //LED灯亮电平为低电平
#define LED_OFF         0   //LED灯灭电平为高电平


#define LEDC_MAX_DUTY         	(8191)	//2的13次方-1(13位PWM)
#define LEDC_FADE_TIME    		(1000)	//渐变时间(ms)

#define PWM_RED_CHANNEL   LEDC_CHANNEL_0   //定义红灯通道
#define PWM_GREEN_CHANNEL   LEDC_CHANNEL_1   //定义绿灯通道
#define PWM_BLUE_CHANNEL   LEDC_CHANNEL_2   //定义蓝灯通道

void PWM_init(void);
void CtrRBG_G(unsigned char);
void CtrRBG_B(unsigned char);
void task_pwm1(void *pvParameter);
void ATimerCallback( TimerHandle_t xTimer );
void LED_Init(void);
void led_task(void* arg);
//控制绿灯
void led_green(int on);
//控制蓝灯
void led_blue(int on);
//控制绿灯
void led_green2(int on);
//控制蓝灯
void led_blue2(int on);

