idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES nrf24l01 esp_driver_uart esp_timer)
