#include "supersconic.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "filter.h"

static const int RX_BUF_SIZE = 1024;
unsigned char COMMAND[1] = {0XA0};

#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_18)
#define UART_NUM (UART_NUM_1)

#define LPF_5_(hz, t, in, out) ((out) += (1 / (1 + 1 / ((hz)*6.28f * (t)))) * ((in) - (out)))

// 串口调参数
void DebugUartInit(void)
{
    uart_config_t uartConfig =
        {
            .baud_rate = 115200,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_APB,
        };

    uart_driver_install(UART_NUM_0, RX_BUF_SIZE, RX_BUF_SIZE * 2, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &uartConfig);
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void receive_pid_data(uint8_t data[96],uint8_t *flag)
{
    *flag= uart_read_bytes(UART_NUM_0, data, RX_BUF_SIZE, 100);
    if (*flag)
    {
        printf("receive is ok!\n");
    }
}

void wave_hight_init(void)
{

    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

int sendData()
{
    const int txBytes = uart_write_bytes(UART_NUM, COMMAND, 1);
    return txBytes;
}

// Butter_Parameter Butter_20HZ_Parameter_Acce1 = {
//     // 200hz---20hz
//     {-0.00777631271910257,0.06445464557871,0.443321667140393},
//     {0.443321667140393, 0.06445464557871,-0.00777631271910257}};
// Butter_BufferData Butter_Buffer4;

float recv_data(void)
{
    uint8_t data[10];
    static float length = 0, length_fliter = 0;
    sendData();
    const int rxBytes = uart_read_bytes(UART_NUM, data, RX_BUF_SIZE, 30 / portTICK_RATE_MS);
    if (rxBytes > 0)
    {
        data[rxBytes] = 0;
        length = (data[0] * 65536 + data[1] * 256 + data[2]) / 10000.0f;
        // LPButterworth(length, &Butter_Buffer4, &Butter_20HZ_Parameter_Acce1);
        // LPF_5_(20, 0.001, length, length_fliter);
        // printf("length:%f,%f\n", length, Butter_Buffer4.Output_Butter[2]);
    }
    return length;
}
