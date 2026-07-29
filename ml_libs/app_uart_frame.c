#include <stdarg.h>
#include <stdio.h>
#include "app_uart_frame.h"

void app_uart_frame_init(app_uart_frame_t *parser, char start_ch, char end_ch)
{
    if (parser == 0)
    {
        return;
    }

    parser->start_ch = start_ch;
    parser->end_ch = end_ch;
    parser->index = 0;
    parser->ready = 0;
    parser->receiving = 0;
    parser->buffer[0] = '\0';
}

uint8_t app_uart_frame_push(app_uart_frame_t *parser, char ch)
{
    if (parser == 0)
    {
        return 0;
    }

    if (ch == parser->start_ch)
    {
        parser->index = 0;
        parser->ready = 0;
        parser->receiving = 1;
        parser->buffer[0] = '\0';
        return 0;
    }

    if (!parser->receiving)
    {
        return 0;
    }

    if (ch == parser->end_ch)
    {
        parser->buffer[parser->index] = '\0';
        parser->ready = 1;
        parser->receiving = 0;
        return 1;
    }

    if (parser->index >= APP_UART_FRAME_MAX_LEN - 1)
    {
        parser->index = 0;
        parser->receiving = 0;
        parser->ready = 0;
        parser->buffer[0] = '\0';
        return 0;
    }

    parser->buffer[parser->index++] = ch;
    parser->buffer[parser->index] = '\0';
    return 0;
}

uint8_t app_uart_frame_ready(const app_uart_frame_t *parser)
{
    return parser == 0 ? 0 : parser->ready;
}

char *app_uart_frame_data(app_uart_frame_t *parser)
{
    if (parser == 0 || !parser->ready)
    {
        return 0;
    }

    return parser->buffer;
}

void app_uart_frame_clear(app_uart_frame_t *parser)
{
    if (parser == 0)
    {
        return;
    }

    parser->index = 0;
    parser->ready = 0;
    parser->receiving = 0;
    parser->buffer[0] = '\0';
}

void app_uart_sendf(UARTn_enum uartn, const char *fmt, ...)
{
    char buffer[APP_UART_FRAME_MAX_LEN];
    va_list args;

    if (fmt == 0)
    {
        return;
    }

    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    uart_sendstr(uartn, buffer);
}

