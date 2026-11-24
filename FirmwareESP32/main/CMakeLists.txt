idf_component_register(
    SRCS "main.c" "cloud.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_driver_uart esp_driver_spi
             esp_wifi esp_event esp_netif esp_http_client nvs_flash mbedtls
             freertos
)
