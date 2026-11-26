#ifndef MOCK_DRIVER_SPI_MASTER_H
#define MOCK_DRIVER_SPI_MASTER_H

#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>

typedef enum {
    SPI1_HOST = 0,
    SPI2_HOST = 1,
    SPI3_HOST = 2,
} spi_host_device_t;

#define SPI_DMA_CH_AUTO 3

typedef struct {
    int mosi_io_num;
    int miso_io_num;
    int sclk_io_num;
    int quadwp_io_num;
    int quadhd_io_num;
    int max_transfer_sz;
} spi_bus_config_t;

typedef struct {
    int     clock_speed_hz;
    uint8_t mode;
    int     spics_io_num;
    int     queue_size;
} spi_device_interface_config_t;

typedef struct {
    size_t      length;
    const void *tx_buffer;
    void       *rx_buffer;
} spi_transaction_t;

typedef void *spi_device_handle_t;

esp_err_t spi_bus_initialize(spi_host_device_t host_id, const spi_bus_config_t *bus_config, int dma_chan);
esp_err_t spi_bus_free(spi_host_device_t host_id);
esp_err_t spi_bus_add_device(spi_host_device_t host_id, const spi_device_interface_config_t *dev_config, spi_device_handle_t *handle);
esp_err_t spi_bus_remove_device(spi_device_handle_t handle);
esp_err_t spi_device_transmit(spi_device_handle_t handle, spi_transaction_t *trans_desc);

#endif /* MOCK_DRIVER_SPI_MASTER_H */
