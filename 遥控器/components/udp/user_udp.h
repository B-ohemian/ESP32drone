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
//#define UDP_ADRESS              "255.255.255.255"   //作为client，要连接UDP服务器的地址
                                                    //如果为255.255.255.255：UDP广播
                                                    //如果为192.168.1.90：UDP单播:根据自己的server写

#define UDP_PORT                3333               //统一的端口号，包括UDP客户端或者服务端

extern void create_udp();
void udp_send_data(char* data, int len);

#endif
