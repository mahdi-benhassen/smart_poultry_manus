idf_component_register(
    SRCS "actuator_manager.c"
    INCLUDE_DIRS "include"
    REQUIRES driver freertos esp_timer
             config_manager
             actuator_fan actuator_exhaust_fan actuator_heater
             actuator_cooler actuator_light actuator_water_pump
             actuator_feed_dispenser actuator_alarm actuator_door_servo
)
