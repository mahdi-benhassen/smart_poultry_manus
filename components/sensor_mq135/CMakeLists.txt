idf_component_register(SRCS "sensor_mq135.c"
                    INCLUDE_DIRS "include")

idf_component_set_dependencies(sensor_mq135 "driver")
