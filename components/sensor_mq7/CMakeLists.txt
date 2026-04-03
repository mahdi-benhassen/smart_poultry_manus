idf_component_register(SRCS "sensor_mq7.c"
                    INCLUDE_DIRS "include")

idf_component_set_dependencies(sensor_mq7 "driver")
