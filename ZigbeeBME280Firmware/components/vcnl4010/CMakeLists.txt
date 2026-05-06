idf_component_register(SRCS "vcnl4010.c" 
                    PRIV_REQUIRES esp_driver_i2c esp_driver_gpio esp_event
                    INCLUDE_DIRS "include")
