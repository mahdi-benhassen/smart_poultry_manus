idf_component_register(
    SRCS "control_logic.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_timer sensor_manager actuator_manager actuator_alarm config_manager
)
