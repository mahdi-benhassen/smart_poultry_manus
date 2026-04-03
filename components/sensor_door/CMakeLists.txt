idf_component_register(SRCS "sensor_door.c"
                    INCLUDE_DIRS "include")

idf_component_set_dependencies(sensor_door "driver")
