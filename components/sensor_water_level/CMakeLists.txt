idf_component_register(SRCS "sensor_water_level.c"
                    INCLUDE_DIRS "include")

idf_component_set_dependencies(sensor_water_level "driver")
