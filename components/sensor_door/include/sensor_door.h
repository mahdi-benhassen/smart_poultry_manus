#ifndef SENSOR_DOOR_H
#define SENSOR_DOOR_H

#include "esp_err.h"
#include "driver/gpio.h"

/**
 * @brief Initializes the door sensor.
 * 
 * @param door_pin The GPIO pin connected to the door sensor.
 * @return ESP_OK on success, or an error code otherwise.
 */
esp_err_t sensor_door_init(gpio_num_t door_pin);

/**
 * @brief Reads the state of the door sensor.
 * 
 * @param door_open Pointer to a boolean to store the door state (true if open, false if closed).
 * @return ESP_OK on success, or an error code otherwise.
 */
esp_err_t sensor_door_read(bool *door_open);

#endif // SENSOR_DOOR_H
