#pragma once

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t actuator_door_servo_init(int gpio_num);
esp_err_t actuator_door_servo_set_angle(uint8_t angle);
uint8_t actuator_door_servo_get_angle(void);

#ifdef __cplusplus
}
#endif
