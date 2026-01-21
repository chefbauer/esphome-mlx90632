idf_component_register(
    SRCS "Adafruit_BusIO_Register.cpp"
         "Adafruit_I2CDevice.cpp"
         "Adafruit_MLX90632.cpp"
         "ESPHomeI2CAdapter.cpp"
         "mlx90632.cpp"
    INCLUDE_DIRS "."
    REQUIRES driver esp_common
    PRIV_REQUIRES log
)
