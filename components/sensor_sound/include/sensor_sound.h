#ifndef SENSOR_SOUND_H
#define SENSOR_SOUND_H

#include "esp_err.h"
#include "driver/adc.h"

/**
 * @brief Initializes the sound sensor (analog microphone).
 * 
 * @param adc_channel The ADC channel connected to the sound sensor analog output.
 * @return ESP_OK on success, or an error code otherwise.
 */
esp_err_t sensor_sound_init(adc_channel_t adc_channel);

/**
 * @brief Reads sound level from the sensor and calculates it in dB.
 * 
 * @param sound_db Pointer to a float to store the sound level in dB.
 * @return ESP_OK on success, or an error code otherwise.
 */
esp_err_t sensor_sound_read(float *sound_db);

#endif // SENSOR_SOUND_H
