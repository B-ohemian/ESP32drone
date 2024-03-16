#ifndef _USER_UDP_H_
#define _USER_UDP_H_

#include <string.h>
#include <sys/socket.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"

#define UDP_ADRESS              "192.168.2.90"
//#define UDP_ADRESS              "255.255.255.255"   //��Ϊclient��Ҫ����UDP�������ĵ�ַ
                                                    //���Ϊ255.255.255.255��UDP�㲥
                                                    //���Ϊ192.168.1.90��UDP����:�����Լ���serverд

#define UDP_PORT                3333               //ͳһ�Ķ˿ںţ�����UDP�ͻ��˻��߷����

extern void create_udp();
void udp_send_data(char* data, int len);

#endif
