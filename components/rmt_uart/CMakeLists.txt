idf_component_register(SRCS "rmt_uart.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_rmt esp_driver_gpio)
