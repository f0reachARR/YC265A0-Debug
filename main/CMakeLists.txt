idf_component_register(SRCS "main.c"
    PRIV_REQUIRES tlv320aic32x4 rmt_uart esp_driver_gpio esp_driver_uart esp_driver_i2c spi_flash
    INCLUDE_DIRS "")
