#include "led.h"
#include "init.h"
#include "motor.h"
#include "user_udp.h"
#include "mpu6050.h"
#include "spl06.h"
#include "pid.h"

#define REMOTE_MODE 0

#define REMOTE_MODE_SELECT 1

static struct sockaddr_in dest_addr;
socklen_t dest_addr_socklen = sizeof(dest_addr);

static int udp_socket = 0;
TaskHandle_t xUDPRecvTask = NULL;

extern QueueHandle_t QHandle_recv;
extern TaskHandle_t xTask_motor;
extern TaskHandle_t xTask_LED;
extern TaskHandle_t allTask_task;
extern TaskHandle_t xTask_pidrec;
static char table[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9'};

extern uint16_t RC_THROTTLE;
extern uint16_t RC_PIT;
extern uint16_t RC_ROL;

extern int16_t Z_THROTTLE;
extern uint8_t OFFSET_FLAG;
extern float asl_offset;
extern bool MODE;

bool FLAG = false;

void num2char(char *str, double number, uint8_t g, uint8_t l)
{
    uint8_t i;
    int temp = number / 1;
    double t2 = 0.0;
    for (i = 1; i <= g; i++)
    {
        if (temp == 0)
            str[g - i] = table[0];
        else
            str[g - i] = table[temp % 10];
        temp = temp / 10;
    }
    *(str + g) = '.';
    temp = 0;
    t2 = number;
    for (i = 1; i <= l; i++)
    {
        temp = t2 * 10;
        str[g + i] = table[temp % 10];
        t2 = t2 * 10;
    }
    *(str + g + l + 1) = '\0';
}

void udp_send_data(char *data, int len)
{
    if (udp_socket > 0)
    {

        int err = sendto(udp_socket, data, len, 0, (struct sockaddr *)&dest_addr, dest_addr_socklen);

        if (err < 0)
        {
            printf("Error occurred during sending: errno %d", errno);
        }
    }
}

void udp_tran_data(void *pvParameters)
{
    QueueHandle_t QHandle_UDP;
    QHandle_UDP = (QueueHandle_t)pvParameters;
    BaseType_t xStaus;
    char rx_data[10] = {0};
    // printf("rx_data:%s",rx_data);
    while (1)
    {
        if (uxQueueMessagesWaiting(QHandle_UDP) != 0)
        {
            printf("QHandle_UDP is ok!\n");
            xStaus = xQueueReceive(QHandle_UDP, &rx_data, portMAX_DELAY);
            if (xStaus != pdPASS)
            {
                printf("recv faild!\n");
            }
            else
            {
                int len = strlen(rx_data);
                rx_data[len] = 0;
                printf("len:%d,rx_data:%s\n", len, rx_data);
                udp_send_data((char *)rx_data, len);
            }
        }
        vTaskDelay(1 / portTICK_RATE_MS);
    }
}

void udp_recv_data(void *pvParameters)
{
    QueueHandle_t QHandle_UDP;
    QHandle_UDP = (QueueHandle_t)pvParameters;
    BaseType_t xStaus;

    socklen_t socklen = sizeof(dest_addr);

    bool flag = false;
    float pid_arg[18] = {0};
    float BB;
    char AAA[10] = {0};
    int len = 0;
    int num = 0;
    char *pEnd = NULL;
    uint8_t rx_buffer[1024] = {0};
    char temp[3] = {0};

    uint16_t thr = 0, up = 0, down = 0, left = 0, right = 0, key = 0;

    printf("create udp recv\n");
    if (xTask_motor != NULL)
    {
        xTaskNotify(xTask_motor, 0x01, eSetValueWithoutOverwrite);
    }
    while (1)
    {
        int len = recvfrom(udp_socket, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&dest_addr, &dest_addr_socklen);
        if (len > 0)
        {
            rx_buffer[len] = 0;
            if (rx_buffer[0] == 'O')
            {
                xTaskNotify(xTask_LED, 0x01, eSetValueWithoutOverwrite);
                flag = true;
                printf("flag value :%d\n", flag);
            }
            else if (rx_buffer[0] == 'L')
            {
                xTaskNotify(xTask_LED, 0x02, eSetValueWithoutOverwrite);
                flag = false;
                printf("flag value :%d\n", flag);
            }
            if (flag)
            {
                if (rx_buffer[0] == 'T')
                {
                    temp[0] = rx_buffer[1];
                    temp[1] = rx_buffer[2];
                    temp[2] = rx_buffer[3];
                    RC_THROTTLE = atoi(temp);
                    printf("RC_THROTTLE: %d\n", RC_THROTTLE);
                }
                if (rx_buffer[0] == 'U')
                {
                    temp[0] = rx_buffer[1];
                    temp[1] = rx_buffer[2];
                    RC_PIT = atoi(temp);
                    printf("RC_PIT: %d\n", RC_PIT);
                }
                if (rx_buffer[0] == 'D')
                {
                    temp[0] = rx_buffer[1];
                    temp[1] = rx_buffer[2];
                    RC_PIT = -atoi(temp);
                    printf("RC_PIT: %d\n", RC_PIT);
                }
                if (rx_buffer[0] == 'L')
                {

                    temp[0] = rx_buffer[1];
                    temp[1] = rx_buffer[2];
                    RC_ROL = atoi(temp);
                    printf("RC_ROL: %d\n", RC_ROL);
                }
                if (rx_buffer[0] == 'R')
                {

                    temp[0] = rx_buffer[1];
                    temp[1] = rx_buffer[2];
                    RC_ROL = -atoi(temp);
                    printf("RC_ROL: %d\n", RC_ROL);
                }
                if (rx_buffer[0] == 'Z')
                {

                    RC_PIT = 0;
                    RC_ROL = 0;
                    printf("RC_ROL: %d\n", RC_ROL);
                }
                if (rx_buffer[0] == '1')
                {
                    RC_ROL = RC_ROL + 0.1;
                    // printf("%0.2f,%0.2f\n", RC_ROL, RC_PIT);
                }
                if (rx_buffer[0] == '2')
                {

                    RC_ROL = RC_ROL - 0.1;
                    // printf("%0.2f,%0.2f\n", RC_ROL, RC_PIT);
                }
                if (rx_buffer[0] == '3')
                {

                    RC_PIT = RC_PIT + 0.1;
                    // printf("%0.2f,%0.2f\n", RC_ROL, RC_PIT);
                }
                if (rx_buffer[0] == '4')
                {

                    RC_PIT = RC_PIT - 0.1;
                    // printf("%0.2f,%0.2f\n", RC_ROL, RC_PIT);
                }
                if (rx_buffer[0] == 'A' && rx_buffer[1] == 'A' && rx_buffer[2] == 'A')
                {
                    int m = strlen(&rx_buffer);
                    printf("length:%d,data_pid: %s\n", m, rx_buffer);
                    for (int i = 0; i < m; i = i + len)
                    {
                        len = 0;
                        for (int i = 0; i < 10; i++)
                        {
                            AAA[i] = '\0';
                        }
                        if (rx_buffer[i] == 'A')
                        {
                            i += 3;
                            for (int j = i; rx_buffer[j] != 'A'; j++)
                            {
                                len++;
                            }
                            for (int j = 0; j < len; j++)
                            {
                                AAA[j] = rx_buffer[j + i];
                            }
                            AAA[len] = '\0';
                            BB = atof(AAA);
                            pid_arg[num] = BB;
                            // printf("%s,i:%d,bb:%f\n", AAA, number, pid_arg[number]);
                            // printf("bb:%f\n", pid_arg[number]);
                            num++;
                            // printf("num:%d\n", num);
                        }
                    }
                    num = 0;
                    // pid_arg[15]='\0';
                    // for (int i = 0; i < 18; i++)
                    // {
                    //     printf("pid:%f\n", pid_arg[i]);
                    // }
                    xQueueSend(QHandle_UDP, &pid_arg, 0);
                }
            }
            // printf("value :%s\n", rx_buffer);
            // xQueueSend(QHandle_UDP, &rx_buffer, 0);
            // printf("value flag :%s\n", rx_buffer);
        }
    }
}

void udp_tran_data_remote(void *pvParameters)
{
    // TickType_t xLastWakeTime = xTaskGetTickCount();
    // const TickType_t xFrequency = 10;

    socklen_t socklen = sizeof(dest_addr);
    uint8_t rx_buffer[15] = {0};
    uint16_t thr = 0, pit = 0, rol = 0;

    while (1)
    {
        // vTaskDelayUntil(&xLastWakeTime, xFrequency);
        int len = recvfrom(udp_socket, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&dest_addr, &dest_addr_socklen);
        if (len > 0)
        {
            if (rx_buffer[0] == 'O')
            {
                FLAG = true;
                xTaskNotify(xTask_LED, 0x01, eSetValueWithoutOverwrite);
                OFFSET_FLAG = get_offset_start();
                printf("flag value :%d\n", FLAG);
            }
            else if (rx_buffer[0] == 'L')
            {
                FLAG = false;
                OFFSET_FLAG = 0;
                xTaskNotify(xTask_LED, 0x02, eSetValueWithoutOverwrite);
                printf("flag value :%d\n", FLAG);
            }
            else if (rx_buffer[0] == 'S')
            {
                RC_THROTTLE = 0;
                FLAG = false;
            }
            else if (rx_buffer[0] == 'P')
            {
               MODE=1;  
            }
            else if (rx_buffer[0] == 'Q')
            {
               MODE=0;  
            }

            if (FLAG)
            {
                RC_THROTTLE = rx_buffer[3] << 8 | rx_buffer[2];

                RC_ROL = rx_buffer[5] << 8 | rx_buffer[4];

                RC_PIT = rx_buffer[7] << 8 | rx_buffer[6];

                // printf("receive:%d,%d,%d\n", RC_THROTTLE, RC_ROL, RC_PIT);
            }
        }
    }
}

void receivedata(void)
{
    uint8_t rx_buffer[15] = {0};
    int len = recvfrom(udp_socket, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&dest_addr, &dest_addr_socklen);
    if (len > 0)
    {
        // if (rx_buffer[0] == 'O')
        // {
        //     FLAG = true;
        //     xTaskNotify(xTask_LED, 0x01, eSetValueWithoutOverwrite);
        //     OFFSET_FLAG = get_offset_start();
        //     printf("flag value :%d\n", FLAG);
        // }
        // else if (rx_buffer[0] == 'L')
        // {
        //     FLAG = false;
        //     OFFSET_FLAG = 0;
        //     xTaskNotify(xTask_LED, 0x02, eSetValueWithoutOverwrite);
        //     printf("flag value :%d\n", FLAG);
        // }
        // else if (rx_buffer[0] == 'S')
        // {
        //     RC_THROTTLE = 0;
        //     FLAG = false;
        // }

        // if (rx_buffer[0] == 'T')
        // {
        //     Z_THROTTLE = Z_THROTTLE + 10;
        //     printf("z_thrust,%d\n", Z_THROTTLE);
        // }
        // if (rx_buffer[0] == 'D')
        // {
        //     Z_THROTTLE = Z_THROTTLE - 10;
        //     printf("z_thrust,%d\n", Z_THROTTLE);
        // }
        // if (FLAG)
        // {
        RC_THROTTLE = rx_buffer[3] << 8 | rx_buffer[2];

        RC_ROL = rx_buffer[5] << 8 | rx_buffer[4];

        RC_PIT = rx_buffer[7] << 8 | rx_buffer[6];

        printf("receive:%d,%d,%d\n", RC_THROTTLE, RC_ROL, RC_PIT);
        // }
    }
}

void udp_ini_client(void *pvParameters)
{

    if (udp_socket > 0)
    {
        close(udp_socket);
        udp_socket = 0;
    }

    udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
    printf("connect_socket:%d\n", udp_socket);

    if (udp_socket < 0)
    {
        printf("Unable to create socket: errno %d", errno);
        return;
    }

    dest_addr.sin_addr.s_addr = inet_addr("255.255.255.255");
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(UDP_PORT);
    printf("Socket created, sending to %s:%d\n", UDP_ADRESS, UDP_PORT);

    struct sockaddr_in Loacl_addr;
    Loacl_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    Loacl_addr.sin_family = AF_INET;
    Loacl_addr.sin_port = htons(UDP_PORT);

    uint8_t res = 0;

    res = bind(udp_socket, (struct sockaddr *)&Loacl_addr, sizeof(Loacl_addr));
    if (res != 0)
    {
        printf("bind error\n");
    }
    // else{
    //     xTaskNotify( xTask_motor, 0x01, eSetValueWithoutOverwrite );
    // }

    if (xUDPRecvTask != NULL)
    {
        vTaskDelete(xUDPRecvTask);
        xUDPRecvTask = NULL;
    }

    // xTaskCreate(&udp_recv_data, "udp_recv_data", 2048 * 5, (void *)QHandle_recv, 2, &xUDPRecvTask);
    // xTaskCreate(&udp_tran_data_remote, "udp_tran_data_remote", 2048 * 2, NULL, 10, &xUDPRecvTask,0);
    xTaskCreatePinnedToCore(&udp_tran_data_remote, "udp_tran_data_remote", 2048 * 2, NULL, 10, &xUDPRecvTask, 0);
    vTaskDelete(NULL);
}

void create_udp()
{
    xTaskCreate(&udp_ini_client, "udp_ini_client", 2048 * 2, NULL, 10, NULL);
    // if(REMOTE_MODE)
    //     xTaskCreate(&udp_tran_data, "udp_tran_data", 4096, (void *)QHandle_tran, 2, NULL);
    // else
    //     xTaskCreate(&udp_tran_data_remote, "udp_tran_data_remote", 4096, (void *)QHandle_tran, 2, NULL);
}
