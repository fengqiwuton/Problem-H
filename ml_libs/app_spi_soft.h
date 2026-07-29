#ifndef __APP_SPI_SOFT_H__
#define __APP_SPI_SOFT_H__

#include "headfile.h"

typedef struct
{
    GPIOn_enum sck_port;
    Pinx_enum sck_pin;
    GPIOn_enum miso_port;
    Pinx_enum miso_pin;
    GPIOn_enum mosi_port;
    Pinx_enum mosi_pin;
    GPIOn_enum cs_port;
    Pinx_enum cs_pin;
} app_spi_soft_t;

void app_spi_soft_init(app_spi_soft_t *spi);
void app_spi_soft_select(app_spi_soft_t *spi);
void app_spi_soft_release(app_spi_soft_t *spi);
uint8_t app_spi_soft_transfer(app_spi_soft_t *spi, uint8_t data);

#endif

