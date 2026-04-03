idf_component_register(SRCS "sensor_scd40.c"
                    INCLUDE_DIRS "include")

idf_component_set_dependencies(sensor_scd40 "driver")
