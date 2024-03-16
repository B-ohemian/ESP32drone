#include "led.h"
#include "ws2812.h"

// #include "freertos/timers.h"
// LED GPIO初始化
void LED_Init()
{

  gpio_pad_select_gpio(LED_GREEN_PIN);  // 使能GPIO口
  gpio_pad_select_gpio(LED_BLUE_PIN);   // 使能GPIO口
  gpio_pad_select_gpio(LED_GREEN_PIN2); // 使能GPIO口
  gpio_pad_select_gpio(LED_BLUE_PIN2);  // 使能GPIO口

  gpio_set_direction(LED_BLUE_PIN, GPIO_MODE_OUTPUT);  // 设置GPIO的输入输出
  gpio_set_direction(LED_GREEN_PIN, GPIO_MODE_OUTPUT); // 设置GPIO的输入输出

  gpio_set_direction(LED_BLUE_PIN2, GPIO_MODE_OUTPUT);  // 设置GPIO的输入输出
  gpio_set_direction(LED_GREEN_PIN2, GPIO_MODE_OUTPUT); // 设置GPIO的输入输出

  gpio_set_level(LED_GREEN_PIN, 0); // 设置初始化电平
  gpio_set_level(LED_BLUE_PIN, 0);
  gpio_set_level(LED_GREEN_PIN2, 0); // 设置初始化电平
  gpio_set_level(LED_BLUE_PIN2, 0);

  led_green(LED_OFF);
  led_blue(LED_OFF);
  led_green2(LED_OFF);
  led_blue2(LED_OFF);
}
// 定时器回调函数
void ATimerCallback(TimerHandle_t xTimer) // 绿灯表示wifi连接，蓝灯表示信号连接
{
  static bool flag = 1;
  led_green(flag);
  led_blue2(flag);
  flag = !flag;
}

// 定时器回调函数
void ATimerCallback2(TimerHandle_t xTimer)
{
  static bool flag = 1;
  led_green2(flag);
  led_blue(flag);
  flag = !flag;
}

void led_task(void *arg)
{
  uint32_t ulNotifiedValue; // 事件通知值

  TimerHandle_t xTimer1; // 定时器句柄
  TimerHandle_t xTimer2; // 定时器句柄

  LED_Init();
  // ws2812_init();
  xTimer1 = xTimerCreate("Timer1", pdMS_TO_TICKS(100), pdTRUE, (void *)0, ATimerCallback);
  xTimer2 = xTimerCreate("Timer2", pdMS_TO_TICKS(100), pdTRUE, (void *)0, ATimerCallback2);
  xTimerStart(xTimer1, 0);
  xTimerStart(xTimer2, 0);
  while (1)
  {
    // 在这里阻塞
    xTaskNotifyWait(
        0x00,             /* Don't clear any notification bits on entry. */
        ULONG_MAX,        /* Reset the notification value to 0 on exit. */
        &ulNotifiedValue, /* Notified value pass out in
                             ulNotifiedValue. */
        portMAX_DELAY);   /* Block indefinitely. */
    if ((ulNotifiedValue & 0x01) != 0)
    {
      led_green(LED_OFF);
      led_blue2(LED_OFF);
      xTimerStop(xTimer1, 0);
      vTaskDelay(10 / portTICK_RATE_MS);
    }
    if ((ulNotifiedValue & 0x02) != 0)
    {
      xTimerStart(xTimer1, 0);
      vTaskDelay(10 / portTICK_RATE_MS);
    }
    if ((ulNotifiedValue & 0x03) != 0)
    {
      led_blue(LED_OFF);
      led_green2(LED_OFF);
      xTimerStop(xTimer2, 0);
      vTaskDelay(10 / portTICK_RATE_MS);
    }
    if ((ulNotifiedValue & 0x04) != 0)
    {
      xTimerStart(xTimer2, 0);
      vTaskDelay(10 / portTICK_RATE_MS);
    }
  }
}

// 所有灯亮
void all_led_on(void)
{
  led_green(LED_OFF);
  led_green2(LED_OFF);

  led_blue(LED_OFF);
  led_blue2(LED_OFF);
}

// 控制绿灯
void led_green(int on)
{
  if (on == LED_ON)
  {
    gpio_set_level(LED_GREEN_PIN, LED_ON); // 开灯
  }
  else
  {
    gpio_set_level(LED_GREEN_PIN, LED_OFF); // 关灯
  }
}

// 控制蓝灯
void led_blue(int on)
{
  if (on == LED_ON)
  {
    gpio_set_level(LED_BLUE_PIN, LED_ON); // 开灯
  }
  else
  {
    gpio_set_level(LED_BLUE_PIN, LED_OFF); // 关灯
  }
}

// 控制绿灯
void led_green2(int on)
{
  if (on == LED_ON)
  {
    gpio_set_level(LED_GREEN_PIN2, LED_ON); // 开灯
  }
  else
  {
    gpio_set_level(LED_GREEN_PIN2, LED_OFF); // 关灯
  }
}

// 控制蓝灯
void led_blue2(int on)
{
  if (on == LED_ON)
  {
    gpio_set_level(LED_BLUE_PIN2, LED_ON); // 开灯
  }
  else
  {
    gpio_set_level(LED_BLUE_PIN2, LED_OFF); // 关灯
  }
}

// ledc配置结构体
ledc_channel_config_t g_ledc_red, g_ledc_green, g_ledc_blue;
unsigned char pwm_mode = 1; // PWM模块，如果为1表示通过库函数实现渐变功能

void PWM_init(void)
{
  // 定时器配置结构体
  ledc_timer_config_t ledc_timer;
  // 定时器配置->timer0
  ledc_timer.duty_resolution = LEDC_TIMER_13_BIT; // PWM分辨率
  ledc_timer.freq_hz = 5000;                      // 频率
  ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;    // 速度
  ledc_timer.timer_num = LEDC_TIMER_0;            // 选择定时器
  ledc_timer.clk_cfg = LEDC_USE_APB_CLK;
  ledc_timer_config(&ledc_timer); // 设置定时器PWM模式

  // PWM通道0配置->IO15->绿色灯
  g_ledc_green.channel = PWM_GREEN_CHANNEL;      // PWM通道
  g_ledc_green.duty = LEDC_MAX_DUTY;             // 占空比
  g_ledc_green.gpio_num = LED_GREEN_PIN;         // IO映射
  g_ledc_green.speed_mode = LEDC_LOW_SPEED_MODE; // 速度
  g_ledc_green.timer_sel = LEDC_TIMER_0;         // 选择定时器
  ledc_channel_config(&g_ledc_green);            // 配置PWM
  // PWM模式为1的时候，使能ledc渐变功能
  if (pwm_mode == 1)
  {
    ledc_fade_func_install(0);
  }
}

// 设置绿灯的PWM级别
// 输入level取值0~255
void CtrRBG_G(unsigned char level)
{
  int duty = 0;

  if (level == 255)
  {
    duty = LEDC_MAX_DUTY;
  }
  else if (level == 0)
  {
    duty = 0;
  }
  else
  {
    // 计算占空比
    duty = (level * LEDC_MAX_DUTY) / 255;
  }

  ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_GREEN_CHANNEL, duty); // 修改占空比
  ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_GREEN_CHANNEL);    // 新的占空比生效
}

// 设置蓝灯的PWM级别
// 输入level取值0~255
void CtrRBG_B(unsigned char level)
{
  int duty = 0;

  if (level == 255)
  {
    duty = LEDC_MAX_DUTY;
  }
  else if (level == 0)
  {
    duty = 0;
  }
  else
  {
    // 计算占空比
    duty = (level * LEDC_MAX_DUTY) / 255;
  }

  ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_BLUE_CHANNEL, duty); // 修改占空比
  ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_BLUE_CHANNEL);    // 新的占空比生效
}

// 通过渐变功能演示PWM任务
void task_pwm1(void *pvParameter)
{
  while (1)
  {
    if (1 == pwm_mode)
    {
      printf("pwm mode.\r\n");
      // 渐变功能演示PWM
      ///////////
      // 绿灯占空比100%-->0%-->100%，时间2*LEDC_FADE_TIME
      // 绿灯：灭-->亮-->灭，的过程
      //////////

      // 绿灯占空比100%渐变至0%，时间LEDC_FADE_TIME
      ledc_set_fade_with_time(g_ledc_green.speed_mode,
                              g_ledc_green.channel,
                              0,
                              LEDC_FADE_TIME);
      // 渐变开始
      ledc_fade_start(g_ledc_green.speed_mode,
                      g_ledc_green.channel,
                      LEDC_FADE_NO_WAIT);
      vTaskDelay(LEDC_FADE_TIME / portTICK_PERIOD_MS);

      // 绿灯占空比0%渐变至100%，时间LEDC_FADE_TIME
      ledc_set_fade_with_time(g_ledc_green.speed_mode,
                              g_ledc_green.channel,
                              LEDC_MAX_DUTY,
                              LEDC_FADE_TIME);
      // 渐变开始
      ledc_fade_start(g_ledc_green.speed_mode,
                      g_ledc_green.channel,
                      LEDC_FADE_NO_WAIT);
      vTaskDelay(LEDC_FADE_TIME / portTICK_PERIOD_MS);
      vTaskDelay(LEDC_FADE_TIME / portTICK_PERIOD_MS);

      ///////////
      // 蓝灯占空比100%-->0%-->100%，时间2*LEDC_FADE_TIME
      // 蓝灯：灭-->亮-->灭，的过程
      //////////

      // //蓝灯占空比100%渐变至0%，时间LEDC_FADE_TIME
      // ledc_set_fade_with_time(g_ledc_blue.speed_mode,
      //                 g_ledc_blue.channel,
      //                 0,
      //                 LEDC_FADE_TIME);
      // //渐变开始
      // ledc_fade_start(g_ledc_blue.speed_mode,
      //                 g_ledc_blue.channel,
      //                 LEDC_FADE_NO_WAIT);
      // //延时LEDC_FADE_TIME，给LEDC控制时间
      // vTaskDelay(LEDC_FADE_TIME / portTICK_PERIOD_MS);

      // //蓝灯占空比0%渐变至100%，时间LEDC_FADE_TIME
      // ledc_set_fade_with_time(g_ledc_blue.speed_mode,
      //                 g_ledc_blue.channel,
      //                 LEDC_MAX_DUTY,
      //                 LEDC_FADE_TIME);
      // //渐变开始
      // ledc_fade_start(g_ledc_blue.speed_mode,
      //                 g_ledc_blue.channel,
      //                 LEDC_FADE_NO_WAIT);

      // //延时LEDC_FADE_TIME，给LEDC控制时间
      // vTaskDelay(LEDC_FADE_TIME / portTICK_PERIOD_MS);
      // vTaskDelay(LEDC_FADE_TIME / portTICK_PERIOD_MS);
    }
    else
    {
      vTaskDelay(5); // 延时一下
    }
  }
}

// 用户修改占空比，改变LED灯亮度任务
//  void task_pwm2(void *pvParameter)
//  {
//    int pwm_level=0;//0~255占空比
//    int pwm_index=1;//红1绿2蓝3

//   while(1)
//   {
//     if(0==pwm_mode)
//     {
//       printf("pwm mode2.\r\n");
//     	if(pwm_index==1)
//     	{
//     		//修改红灯占空比
//     		CtrRBG_R(pwm_level);
//     		pwm_level+=10;//占空比递增

//     		if(pwm_level>255)
//     		{
//     			pwm_level=0;
//     			pwm_index=2;
//     			CtrRBG_R(255);
//     		}
//     	}
//     	else if(pwm_index==2)
//     	{
//     		//修改绿灯占空比
//     		CtrRBG_G(pwm_level);
//     		pwm_level+=10;//占空比递增

//     		if(pwm_level>255)
//     		{
//     			pwm_level=0;
//     			pwm_index=3;
//     			CtrRBG_G(255);
//     		}
//     	}
//     	else
//     	{
//     		//修改蓝灯占空比
//     		CtrRBG_B(pwm_level);
//     		pwm_level+=10;//占空比递增

//     		if(pwm_level>255)
//     		{
//     			pwm_level=0;
//     			pwm_index=1;
//     			CtrRBG_B(255);
//     		}
//     	}
//     }
//     vTaskDelay(5);//延时一下
//   }

// }