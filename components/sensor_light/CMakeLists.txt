idf_component_register(SRCS "sensor_light.c"
                    INCLUDE_DIRS "include")

idf_component_set_dependencies(sensor_light "driver")
