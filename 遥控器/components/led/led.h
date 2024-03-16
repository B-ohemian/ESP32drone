#ifndef _LED_H_
#define _LED_H_

//����LED�Ƶ�IO��
#define LED_RED_IO 		26  //��Ӧ��Ƶ�LED
#define LED_GREEN_IO 	27  //��Ӧ��Ƶ�LED

//���尴��
#define KEY1_IO 23
#define KEY2_IO 22
#define KEY3_IO 21
#define KEY4_IO 19

#define KEY5_IO 18
#define KEY6_IO 5
#define KEY7_IO 17
#define KEY8_IO 16

#define KEY_SAVE_IO 0




//����LED״̬
#define LED_ON          0   //LED������ƽΪ�͵�ƽ
#define LED_OFF         1   //LED�����ƽΪ�ߵ�ƽ




//���ƺ��
void led_red(int on);
//�����̵�
void led_green(int on);
//LED��ʼ��
void initLed();
void initkey(void);
void led_twinkle(void);


#endif