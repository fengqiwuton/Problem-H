#include "app_board.h"
#include "app_module_test.h"

void uart_test_run(void)
{
    app_uart_frame_t parser;
    char *frame;

    OLED_Init();
    OLED_Clear();
    uart_init(BOARD_UART_DEBUG, BOARD_UART_DEBUG_BAUD, 2);
    app_uart_frame_init(&parser, '$', '#');

    app_uart_frame_push(&parser, '$');
    app_uart_frame_push(&parser, 'O');
    app_uart_frame_push(&parser, 'K');
    app_uart_frame_push(&parser, '#');
    frame = app_uart_frame_data(&parser);

    uart_sendstr(BOARD_UART_DEBUG, "UART TEST\r\n");
    app_uart_sendf(BOARD_UART_DEBUG, "FRAME:%s\r\n", frame == 0 ? "NULL" : frame);

    OLED_ShowString(1, 1, "UART TEST");
    OLED_ShowString(2, 1, "TX OK");
    OLED_ShowString(3, 1, "RX:");
    OLED_ShowString(3, 4, frame == 0 ? "NULL" : frame);

    while (1)
    {
        delay_ms(500);
    }
}

