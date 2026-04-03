#ifndef SENSOR_DHT22_H
#define SENSOR_DHT22_H

#include "esp_err.h"
#include "driver/gpio.h"

/**
 * @brief Initializes the DHT22 sensor.
 * 
 * @param dht_pin The GPIO pin connected to the DHT22 data line.
 * @return ESP_OK on success, or an error code otherwise.
 */
esp_err_t sensor_dht22_init(gpio_num_t dht_pin);

/**
 * @brief Reads temperature and humidity from the DHT22 sensor.
 * 
 * @param temperature Pointer to a float to store the temperature in Celsius.
 * @param humidity Pointer to a float to store the humidity in percentage.
 * @return ESP_OK on success, or an error code otherwise.
 */
esp_err_t sensor_dht22_read(float *temperature, float *humidity);

#endif // SENSOR_DHT22_H
