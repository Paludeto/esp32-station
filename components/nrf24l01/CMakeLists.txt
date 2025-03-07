idf_component_register(SRCS "src/nrf24l01.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_driver_spi esp_driver_gpio)