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



//定义就是AP模式，如果不定义就是STA模式
#define WIFI_AP

//是否使用静态IP,如果不定义就是DHCP模式
#define ESP32_STATIC_IP


#ifdef WIFI_AP
#define  DEFAULT_SSID "UESTC_REMOTE"        //创建的WIFI名称
#define  DEFAULT_PWD "12345678"                 //创建的wifi对应的密码

//AP模式IP
#define DEVICE_IP "192.168.10.1"
//AP模式网关
#define DEVICE_GW "192.168.10.1"
//AP模式掩码
#define DEVICE_NETMASK "255.255.255.0"
#else
#define  DEFAULT_SSID "TP_LINK"        //STA模式要接入WIFI名称
#define  DEFAULT_PWD "12345678"        //要接入wifi对应的密码

//STA模式IP
#define DEVICE_IP "192.168.2.180"
//STA模式网关
#define DEVICE_GW "192.168.2.1"
//STA模式掩码
#define DEVICE_NETMASK "255.255.255.0"

#endif

extern ip4_addr_t s_ip_addr,s_gw_addr,s_netmask_addr;


//启动WIFI的STA
void wifi_init_sta();

//启动WIFI的AP
void wifi_init_ap();


#endif
