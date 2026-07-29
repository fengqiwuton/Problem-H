#ifndef __APP_UART_FRAME_H__
#define __APP_UART_FRAME_H__

#include "headfile.h"

#define APP_UART_FRAME_MAX_LEN 64

typedef struct
{
    char start_ch;
    char end_ch;
    char buffer[APP_UART_FRAME_MAX_LEN];
    uint8_t index;
    uint8_t ready;
    uint8_t receiving;
} app_uart_frame_t;

void app_uart_frame_init(app_uart_frame_t *parser, char start_ch, char end_ch);
uint8_t app_uart_frame_push(app_uart_frame_t *parser, char ch);
uint8_t app_uart_frame_ready(const app_uart_frame_t *parser);
char *app_uart_frame_data(app_uart_frame_t *parser);
void app_uart_frame_clear(app_uart_frame_t *parser);
void app_uart_sendf(UARTn_enum uartn, const char *fmt, ...);

#endif

