#include "driver/ledc.h"
#include "led.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm.h"
#include "user_udp.h"
#include "esp_task_wdt.h"
#include "init.h"
// 电机对应的IO口
#define MOTOR1 42
#define MOTOR2 4
#define MOTOR3 5
#define MOTOR4 41
// 接收微信小程序的数据的结构体


void mcpwm_example_gpio_initialize();                                     // 初始化引脚
void brushed_motor1(float duty_cycle);                                    // 电机1
void brushed_motor2(float duty_cycle);                                    // 电机2
void brushed_motor3(float duty_cycle);                                    // 电机3
void brushed_motor4(float duty_cycle);                                    // 电机4
void brushed_motor1_stop();                                               // 电机1停转
void brushed_motor2_stop();                                               // 电机2停转
void brushed_motor3_stop();                                               // 电机3停转
void brushed_motor4_stop();                                               // 电机4停转
void brushed_motor_stop(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num); // 停转
void mcpwm_example_brushed_motor_control(void *arg);                      // 电机任务
void creat_motor_task(void *arg);
void motor_init(void); //初始化电机
void Moto_Pwm(uint8_t , uint8_t , uint8_t , uint8_t );//设置电机的PWM