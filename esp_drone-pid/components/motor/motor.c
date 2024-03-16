#include "motor.h"
/*-------------------------------------
                电机1 左二
                电机2 左一
                电机3 右二
                电机4 右一

这个是使用ESP32的MCPWM控制的电机，会出现电机停止的时候随机乱转的现象，没有解决，所有这个没有用到，使用的是ledc_motor来驱动
---------------------------------------*/
extern QueueHandle_t QHandle_recv = NULL;

void mcpwm_example_gpio_initialize()
{
    // esp32有两个MCPWM模块，每个模块三个opreater，每个opreter可以控制两个PWM
    // printf("initializing mcpwm gpio...\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MOTOR1);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, MOTOR2);

    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, MOTOR3);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0B, MOTOR4);
}

/**
 * @brief motor moves in forward direction, with duty cycle = duty %
 */
void brushed_motor1(float duty_cycle)
{
    // mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_cycle);             // 设置占空比
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); // 设置占空比类型
    // call this each time, if operator was previously in low/high state
}

/**
 * @brief motor moves in backward direction, with duty cycle = duty %
 */
void brushed_motor2(float duty_cycle)
{
    // mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, duty_cycle);             // 设置占空比
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0); // 设置占空比类型
                                                                                      // call this each time, if operator was previously in low/high state
}

/**
 * @brief motor moves in backward direction, with duty cycle = duty %
 */
void brushed_motor3(float duty_cycle)
{
    // mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, duty_cycle);             // 设置占空比
    mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); // 设置占空比类型
                                                                                      // call this each time, if operator was previously in low/high state
}

/**
 * @brief motor moves in backward direction, with duty cycle = duty %
 */
void brushed_motor4(float duty_cycle)
{
    // mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B, duty_cycle);             // 设置占空比
    mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0); // 设置占空比类型
                                                                                      // call this each time, if operator was previously in low/high state
}
/**
 * @brief motor1 stop
 */
void brushed_motor1_stop()
{
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
}
/**
 * @brief motor2 stop
 */
void brushed_motor2_stop()
{
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
}
/**
 * @brief motor3 stop
 */
void brushed_motor3_stop()
{
    mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A);
}
/**
 * @brief motor4 stop
 */
void brushed_motor4_stop()
{
    mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B);
}

/**
 * @brief motor stop
 */
void brushed_motor_stop(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
}


// 初始化电机
void motor_init()
{
    mcpwm_example_gpio_initialize();
    mcpwm_config_t pwm_config1, pwm_config2;
    pwm_config1.frequency = 1000; // frequency = 500Hz,
    pwm_config1.cmpr_a = 0;        // duty cycle of PWMxA = 0
    pwm_config1.cmpr_b = 0;        // duty cycle of PWMxb = 0
    pwm_config1.counter_mode = MCPWM_UP_COUNTER;
    pwm_config1.duty_mode = MCPWM_DUTY_MODE_0;

    pwm_config2.frequency = 1000; // frequency = 500Hz,
    pwm_config2.cmpr_a = 0;        // duty cycle of PWMxA = 0
    pwm_config2.cmpr_b = 0;        // duty cycle of PWMxb = 0
    pwm_config2.counter_mode = MCPWM_UP_COUNTER;
    pwm_config2.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config1); // Configure PWM0A & PWM0B with above settings
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &pwm_config2); // Configure PWM0A & PWM0B with above settings
    Moto_Pwm(0,0,0,0);
}
//设置电机的PWM
void Moto_Pwm(uint8_t MOTO1_PWM, uint8_t MOTO2_PWM, uint8_t MOTO3_PWM, uint8_t MOTO4_PWM)
{
    brushed_motor1(MOTO1_PWM);
    brushed_motor2(MOTO2_PWM);
    brushed_motor3(MOTO3_PWM);
    brushed_motor4(MOTO4_PWM);
}


/**
 * @brief Configure MCPWM module for brushed dc motor
 */
void mcpwm_example_brushed_motor_control(void *arg)
{
    QueueHandle_t QHandle_MOTOR;
    QHandle_MOTOR = (QueueHandle_t)arg;
    BaseType_t xStaus;
    xStruct_recv wechatrecvdata = {0, 0, 0, 0, 0, 0, 0, 0}; // 从队列接收数据的结构体
    // esp_task_wdt_add(NULL);//看门狗
    // 1. mcpwm gpio initialization
    mcpwm_example_gpio_initialize();

    // 2. initial mcpwm configuration
    // printf("i am motor\n");
    mcpwm_config_t pwm_config1, pwm_config2;
    pwm_config1.frequency = 20000; // frequency = 500Hz,
    pwm_config1.cmpr_a = 0;        // duty cycle of PWMxA = 0
    pwm_config1.cmpr_b = 0;        // duty cycle of PWMxb = 0
    pwm_config1.counter_mode = MCPWM_UP_COUNTER;
    pwm_config1.duty_mode = MCPWM_DUTY_MODE_0;

    pwm_config2.frequency = 20000; // frequency = 500Hz,
    pwm_config2.cmpr_a = 0;        // duty cycle of PWMxA = 0
    pwm_config2.cmpr_b = 0;        // duty cycle of PWMxb = 0
    pwm_config2.counter_mode = MCPWM_UP_COUNTER;
    pwm_config2.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config1); // Configure PWM0A & PWM0B with above settings
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &pwm_config2); // Configure PWM0A & PWM0B with above settings

    printf("MOTOR TASK IS RUN!\n");
    while (1)
    {   
        if (uxQueueMessagesWaiting(QHandle_MOTOR) != 0)
        {
            xStaus = xQueueReceive(QHandle_MOTOR, &wechatrecvdata, 0);
            if (xStaus != pdPASS)
            {
                printf("recv faild!\n");
            }
            else
            {
                 
                brushed_motor1(wechatrecvdata.pwmmotor1);
                brushed_motor2(wechatrecvdata.pwmmotor2);
                brushed_motor3(wechatrecvdata.pwmmotor3);
                brushed_motor4(wechatrecvdata.pwmmotor4);
                
                // printf("motor1:%d\n", wechatrecvdata.pwmmotor1);
                // printf("motor2:%d\n", wechatrecvdata.pwmmotor2);
                // printf("motor3:%d\n", wechatrecvdata.pwmmotor3);
                // printf("motor4:%d\n", wechatrecvdata.pwmmotor4);
            }
        }
        vTaskDelay(1 / portTICK_RATE_MS);
    }
}

void creat_motor_task(void *arg)
{
    uint32_t ulNotifiedValue; // 事件通知值
    // printf("MOTORCREAT TASK IS RUN!\n");
    while (1)
    {
        xTaskNotifyWait(
            0x00,             /* Don't clear any notification bits on entry. */
            ULONG_MAX,        /* Reset the notification value to 0 on exit. */
            &ulNotifiedValue, /* Notified value pass out in
                                 ulNotifiedValue. */
            portMAX_DELAY);   /* Block indefinitely. */
        if ((ulNotifiedValue & 0x01) != 0)
        {
            // mcpwm_example_brushed_motor_control(&arg);
            xTaskCreate(&mcpwm_example_brushed_motor_control, "mcpwm_example_brushed_motor_control", 4096, (void *)QHandle_recv, 2, NULL);
            printf("motor blocking is delete!\n");
            // 删除掉这个阻塞任务
            vTaskDelete(NULL);
        }
    }
}