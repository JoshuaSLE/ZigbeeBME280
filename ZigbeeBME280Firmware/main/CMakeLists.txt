idf_component_register(SRCS "main.c"
                    PRIV_REQUIRES esp_event k0i05__esp_ssd1306 i2c_helper bme280 vcnl4010
                    INCLUDE_DIRS ".")
