#ifndef _USER_UDP_H_
#define _USER_UDP_H_

typedef struct
{
     int pwmmotor1;
     int pwmmotor2;
     int pwmmotor3;
     int pwmmotor4;

     int green_led_on;
     int green_led_off;

     int blue_led_on;
     int blue_led_off;
} xStruct_recv;

bool FLAG;


// typedef struct
// {
//      float pit;
//      float yaw;
//      float rol;
//      float battery;
// } xStruct_tran;

// xStruct_tran trandata={0.0,0.0,0.0,0.0};

#define UDP_ADRESS "192.168.31.90"
// #define UDP_ADRESS              "255.255.255.255"   //��Ϊclient��Ҫ����UDP�������ĵ�ַ
// ���Ϊ255.255.255.255��UDP�㲥
// ���Ϊ192.168.1.90��UDP����:�����Լ���serverд

#define UDP_PORT 3333 // ͳһ�Ķ˿ںţ�����UDP�ͻ��˻��߷����

// extern void lcd_display(int s);
extern void create_udp();
void udp_tran_data(void *pvParameters);
void num2char(char *str, double number, uint8_t g, uint8_t l);
void udp_send_data(char *data, int len);
void receivedata(void);

#endif
