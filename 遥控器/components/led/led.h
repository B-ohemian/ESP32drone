#ifndef _LED_H_
#define _LED_H_

//定义LED灯的IO口
#define LED_RED_IO 		26  //对应红灯的LED
#define LED_GREEN_IO 	27  //对应红灯的LED

//定义按键
#define KEY1_IO 23
#define KEY2_IO 22
#define KEY3_IO 21
#define KEY4_IO 19

#define KEY5_IO 18
#define KEY6_IO 5
#define KEY7_IO 17
#define KEY8_IO 16

#define KEY_SAVE_IO 0




//定义LED状态
#define LED_ON          0   //LED灯亮电平为低电平
#define LED_OFF         1   //LED灯灭电平为高电平




//控制红灯
void led_red(int on);
//控制绿灯
void led_green(int on);
//LED初始化
void initLed();
void initkey(void);
void led_twinkle(void);


#endif