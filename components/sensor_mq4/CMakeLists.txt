idf_component_register(SRCS "sensor_mq4.c"
                    INCLUDE_DIRS "include")

idf_component_set_dependencies(sensor_mq4 "driver")
