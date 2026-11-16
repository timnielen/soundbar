/*
 * pc_control.c - raw I2C access from the PC over UART (see pc_control.h)
 *
 * Two commands, in the script syntax of the datasheet (DS 7.5.2.6):
 *
 *   w 58 24 00 80 00 00   write bytes 00 80 00 00 to device 0x58,
 *                         starting at register 0x24
 *   r 58 24 4             read 4 bytes from device 0x58, register 0x24
 *
 * All numbers are hex; the device address is the 8-bit form (0x58 =
 * 7-bit 0x2C). Text after # is ignored. Replies: "ok", the bytes read,
 * or an error message. Everything else (books, pages, muting ...) is
 * done on the PC side by sending these two commands.
 */
#include "pc_control.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

extern UART_HandleTypeDef huart1;
extern I2C_HandleTypeDef  hi2c2;
#define PC_UART  (&huart1)
#define PC_I2C   (&hi2c2)

static uint8_t       rx_byte;
static char          line[96];
static volatile uint8_t len;
static volatile bool line_ready;

void reply(const char *text)
{
    HAL_UART_Transmit(PC_UART, (uint8_t *)text, (uint16_t)strlen(text), 100);
}

void pc_control_start(void)
{
    HAL_UART_Receive_IT(PC_UART, &rx_byte, 1);
}

/* Interrupt: only collect characters until end of line. */
void pc_control_rx_callback(UART_HandleTypeDef *huart)
{
    if (huart != PC_UART) return;
    if (!line_ready) {
        if (rx_byte == '\n' || rx_byte == '\r') {
            if (len) { line[len] = '\0'; line_ready = true; }
        } else if (len < sizeof line - 1) {
            line[len++] = (char)rx_byte;
        }
    }
    HAL_UART_Receive_IT(PC_UART, &rx_byte, 1);
}

bool handle_macros(const char *cmd)
{
    if (strcmp(cmd, "i2c") == 0) {
        for (uint8_t a = 1; a < 128; a++) {
            if (HAL_I2C_IsDeviceReady(&hi2c2, a << 1, 2, 10) == HAL_OK) {
                char buf[sizeof line]; 
                snprintf(buf, sizeof buf, " Found device at 0x%02X\n", a << 1);
                reply(buf);
            }
        }
        return true;
    }
    return false;
}

/* Main loop: execute one finished line. All I2C traffic happens here,
 * never in the interrupt, so it can't interleave with the firmware's
 * own book/page sequences.                                           */
void pc_control_poll(void)
{
    if (!line_ready) return;

    char cmd[sizeof line];
    memcpy(cmd, line, sizeof cmd);
    len = 0;
    line_ready = false;

    char *hash = strchr(cmd, '#');           /* drop comments */
    if (hash) *hash = '\0';

    if (handle_macros(cmd)) return;

    char *op = strtok(cmd, " ");
    char *a  = strtok(NULL, " ");
    char *r  = strtok(NULL, " ");
    if (!op) return;
    if (!a || !r || (op[0] != 'w' && op[0] != 'r') || op[1] != '\0') {
        reply("use: w <addr> <reg> <bytes..>  |  r <addr> <reg> [count]\r\n");
        return;
    }
    uint16_t addr = (uint16_t)strtoul(a, NULL, 16);
    uint8_t  reg  = (uint8_t)strtoul(r, NULL, 16);
    uint8_t  data[16];
    uint8_t  n = 0;

    if (op[0] == 'w') {
        for (char *t = strtok(NULL, " "); t && n < sizeof data; t = strtok(NULL, " "))
            data[n++] = (uint8_t)strtoul(t, NULL, 16);
        if (n == 0) { reply("no data\r\n"); return; }
        if (HAL_I2C_Mem_Write(PC_I2C, addr, reg, I2C_MEMADD_SIZE_8BIT, data, n, 10) != HAL_OK) {
            reply("I2C error\r\n"); return;
        }
        reply("ok\r\n");
    } else {
        char *c = strtok(NULL, " ");
        n = c ? (uint8_t)strtoul(c, NULL, 16) : 1;
        if (n < 1 || n > sizeof data) n = 1;
        if (HAL_I2C_Mem_Read(PC_I2C, addr, reg, I2C_MEMADD_SIZE_8BIT, data, n, 10) != HAL_OK) {
            reply("I2C error\r\n"); return;
        }
        char out[3 * sizeof data + 3];
        for (uint8_t k = 0; k < n; k++)
            snprintf(&out[3 * k], 4, "%02X ", data[k]);
        strcpy(&out[3 * n], "\r\n");
        reply(out);
    }
}
