idf_component_register(
    SRCS "display.c"
    INCLUDE_DIRS "." "${IDF_PATH}/components/esp_hw_support/include/esp_private"
    REQUIRES driver freertos u8g2
)
