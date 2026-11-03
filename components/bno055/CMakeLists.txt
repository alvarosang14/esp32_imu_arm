idf_component_register(
    SRCS "BNO055/bno055.c"
         "bno055_wrapper.c"
    INCLUDE_DIRS "." "include"
    REQUIRES esp_driver_i2c
)