idf_component_register(
    SRCS "u8g2_test.c"
    INCLUDE_DIRS "."   # local includes only
    REQUIRES display rotary buttons u8g2 driver freertos
)