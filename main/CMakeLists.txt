idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES sensor_manager actuator_manager control_logic
             comm_manager config_manager ota_manager
             freertos nvs_flash esp_system
)
