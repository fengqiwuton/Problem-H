#include "app_spi_soft.h"

void app_spi_soft_init(app_spi_soft_t *spi)
{
    if (spi == 0)
    {
        return;
    }

    gpio_init(spi->sck_port, spi->sck_pin, OUT_PP);
    gpio_init(spi->mosi_port, spi->mosi_pin, OUT_PP);
    gpio_init(spi->miso_port, spi->miso_pin, IU);
    gpio_init(spi->cs_port, spi->cs_pin, OUT_PP);
    app_spi_soft_release(spi);
    gpio_set(spi->sck_port, spi->sck_pin, 0);
}

void app_spi_soft_select(app_spi_soft_t *spi)
{
    if (spi != 0)
    {
        gpio_set(spi->cs_port, spi->cs_pin, 0);
    }
}

void app_spi_soft_release(app_spi_soft_t *spi)
{
    if (spi != 0)
    {
        gpio_set(spi->cs_port, spi->cs_pin, 1);
    }
}

uint8_t app_spi_soft_transfer(app_spi_soft_t *spi, uint8_t data)
{
    uint8_t i;
    uint8_t recv = 0;

    if (spi == 0)
    {
        return 0;
    }

    for (i = 0; i < 8; i++)
    {
        gpio_set(spi->mosi_port, spi->mosi_pin, (data & 0x80) ? 1 : 0);
        data <<= 1;
        gpio_set(spi->sck_port, spi->sck_pin, 1);
        recv <<= 1;
        if (gpio_get(spi->miso_port, spi->miso_pin))
        {
            recv |= 1;
        }
        gpio_set(spi->sck_port, spi->sck_pin, 0);
    }

    return recv;
}

