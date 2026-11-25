idf_component_register(SRCS "BMP280.c" "i2c_basic_example_main.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_driver_i2c)
