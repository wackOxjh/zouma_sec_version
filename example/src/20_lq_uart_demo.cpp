#include "lq_all_demo.hpp"

/********************************************************************************
 * @file    lq_uart_demo.cpp
 * @brief   UART 通信 demo.
 * @author  龙邱科技-012
 * @date    2026-04-21
 * @version V2.1.0
 * @note    适用与龙芯 2K0300/0301 平台.
 *          本 demo 实现 UART 通信功能.
 ********************************************************************************/

/********************************************************************************
 * @brief   UART 通信 demo.
 * @param   none.
 * @return  none.
 * @note    本 demo 实现 UART 通信功能.
 ********************************************************************************/
void lq_uart_demo(void)
{
    // 初始化串口端口, 端口引脚和波特率自定义, 其余参数采用默认值
    ls_uart uart(UART1_PIN42, 115200);

    // 定义发送数组和接收数组
    uint8_t send_buf[] = "Hello World!\n";
    uint8_t recv_buf[128] = {0};

    while (ls_system_running.load())
    {
        // 发送数据, 超时时间采用默认值
        uart.uart_write(send_buf, sizeof(send_buf));
        // 清空接收数组
        memset(recv_buf, 0, sizeof(recv_buf));
        // 接收数据, 超时时间采用默认值
        uart.uart_read(recv_buf, 50);
        // 打印接收数据
        printf("recv: %s\n", recv_buf);
        usleep(100*1000);
    }
}