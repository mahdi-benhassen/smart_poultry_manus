#ifndef SENSOR_LIGHT_H
#define SENSOR_LIGHT_H

#include "esp_err.h"
#include "driver/i2c.h"

/**
 * @brief Initializes the BH1750 light sensor.
 * 
 * @param i2c_port The I2C port to use.
 * @param sda_pin The GPIO pin for I2C SDA.
 * @param scl_pin The GPIO pin for I2C SCL.
 * @return ESP_OK on success, or an error code otherwise.
 */
esp_err_t sensor_light_init(i2c_port_t i2c_port, gpio_num_t sda_pin, gpio_num_t scl_pin);

/**
 * @brief Reads lux value from the BH1750 light sensor.
 * 
 * @param light_lux Pointer to a float to store the light intensity in lux.
 * @return ESP_OK on success, or an error code otherwise.
 */
esp_err_t sensor_light_read(float *light_lux);

#endif // SENSOR_LIGHT_H
