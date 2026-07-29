#include "app_board.h"
#include "app_module_test.h"

void spi_soft_test_run(void)
{
    app_spi_soft_t spi;
    uint8_t rx;

    spi.sck_port = BOARD_SPI_SCK_PORT;
    spi.sck_pin = BOARD_SPI_SCK_PIN;
    spi.miso_port = BOARD_SPI_MISO_PORT;
    spi.miso_pin = BOARD_SPI_MISO_PIN;
    spi.mosi_port = BOARD_SPI_MOSI_PORT;
    spi.mosi_pin = BOARD_SPI_MOSI_PIN;
    spi.cs_port = BOARD_SPI_CS_PORT;
    spi.cs_pin = BOARD_SPI_CS_PIN;

    OLED_Init();
    OLED_Clear();
    app_spi_soft_init(&spi);

    while (1)
    {
        app_spi_soft_select(&spi);
        rx = app_spi_soft_transfer(&spi, 0x55);
        app_spi_soft_release(&spi);

        OLED_ShowString(1, 1, "SPI TEST");
        OLED_ShowString(2, 1, "TX:55");
        OLED_ShowString(3, 1, "RX:");
        OLED_ShowHexNum(3, 4, rx, 2);
        delay_ms(300);
    }
}

