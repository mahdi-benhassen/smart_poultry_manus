idf_component_register(
    SRCS "comm_manager.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_wifi esp_netif esp_event esp_http_server mqtt
             sensor_manager actuator_manager control_logic
)
