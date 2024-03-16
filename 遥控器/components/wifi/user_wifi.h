#ifndef _USER_WIFI_H_
#define _USER_WIFI_H_

#include <stdio.h>
#include <lwip/sockets.h>
#include "esp_system.h"
//#include "esp_spi_flash.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_err.h"
//#include "nvs_flash.h"
#include "esp_event.h"



//�������APģʽ��������������STAģʽ
#define WIFI_AP

//�Ƿ�ʹ�þ�̬IP,������������DHCPģʽ
#define ESP32_STATIC_IP


#ifdef WIFI_AP
#define  DEFAULT_SSID "UESTC_REMOTE"        //������WIFI����
#define  DEFAULT_PWD "12345678"                 //������wifi��Ӧ������

//APģʽIP
#define DEVICE_IP "192.168.10.1"
//APģʽ����
#define DEVICE_GW "192.168.10.1"
//APģʽ����
#define DEVICE_NETMASK "255.255.255.0"
#else
#define  DEFAULT_SSID "TP_LINK"        //STAģʽҪ����WIFI����
#define  DEFAULT_PWD "12345678"        //Ҫ����wifi��Ӧ������

//STAģʽIP
#define DEVICE_IP "192.168.2.180"
//STAģʽ����
#define DEVICE_GW "192.168.2.1"
//STAģʽ����
#define DEVICE_NETMASK "255.255.255.0"

#endif

extern ip4_addr_t s_ip_addr,s_gw_addr,s_netmask_addr;


//����WIFI��STA
void wifi_init_sta();

//����WIFI��AP
void wifi_init_ap();


#endif
