idf_component_register(SRCS "sensor_dust.c"
                    INCLUDE_DIRS "include")

idf_component_set_dependencies(sensor_dust "driver")
