#ifndef SENSOR_DUST_H
#define SENSOR_DUST_H

#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"

/**
 * @brief Initializes the dust sensor (GP2Y1010AU0F).
 *
 * @param adc_channel The ADC channel connected to the dust sensor analog output.
 * @param led_pin The GPIO pin connected to the dust sensor's LED control pin.
 * @return ESP_OK on success, or an error code otherwise.
 */
esp_err_t sensor_dust_init(adc_channel_t adc_channel, gpio_num_t led_pin);

/**
 * @brief Reads dust concentration from the dust sensor.
 *
 * @param dust_ugm3 Pointer to a float to store the dust concentration in ug/m3.
 * @return ESP_OK on success, or an error code otherwise.
 */
esp_err_t sensor_dust_read(float *dust_ugm3);

#endif // SENSOR_DUST_H
