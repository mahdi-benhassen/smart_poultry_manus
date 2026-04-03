idf_component_register(SRCS "sensor_sound.c"
                    INCLUDE_DIRS "include")

idf_component_set_dependencies(sensor_sound "driver")
