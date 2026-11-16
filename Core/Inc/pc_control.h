/*
 * pc_control.h - raw I2C access from the PC over UART (for testing)
 *
 * Wiring into main.c:
 *   1. pc_control_start();      once, after MX_USART1_UART_Init()
 *   2. pc_control_poll();       in the while(1) loop
 *   3. in USER CODE BEGIN 4:
 *        void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
 *        { pc_control_rx_callback(huart); }
 *   4. CubeMX: enable the USART1 global interrupt (NVIC tab)
 *   5. don't run the I2S-to-UART stream at the same time (same UART TX)
 */
#ifndef PC_CONTROL_H
#define PC_CONTROL_H

#include "main.h"

void pc_control_start(void);
void pc_control_poll(void);
void pc_control_rx_callback(UART_HandleTypeDef *huart);
void reply(const char *text);

#endif
