idf_component_register(SRCS "sensor_manager.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver
                             sensor_dht22 sensor_mq135 sensor_mq7 sensor_mq4
                             sensor_mq136 sensor_scd40 sensor_dust sensor_light
                             sensor_water_level sensor_sound sensor_door)
