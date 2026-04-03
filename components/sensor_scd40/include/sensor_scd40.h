#ifndef SENSOR_SCD40_H
#define SENSOR_SCD40_H

#include "esp_err.h"
#include "driver/i2c.h"

/**
 * @brief Initializes the SCD40 sensor.
 * 
 * @param i2c_port The I2C port to use.
 * @param sda_pin The GPIO pin for I2C SDA.
 * @param scl_pin The GPIO pin for I2C SCL.
 * @return ESP_OK on success, or an error code otherwise.
 */
esp_err_t sensor_scd40_init(i2c_port_t i2c_port, gpio_num_t sda_pin, gpio_num_t scl_pin);

/**
 * @brief Reads CO2 concentration, temperature, and humidity from the SCD40 sensor.
 * 
 * @param co2_ppm Pointer to a float to store the CO2 concentration in ppm.
 * @param temperature Pointer to a float to store the temperature in Celsius.
 * @param humidity Pointer to a float to store the humidity in percentage.
 * @return ESP_OK on success, or an error code otherwise.
 */
esp_err_t sensor_scd40_read(float *co2_ppm, float *temperature, float *humidity);

#endif // SENSOR_SCD40_H
