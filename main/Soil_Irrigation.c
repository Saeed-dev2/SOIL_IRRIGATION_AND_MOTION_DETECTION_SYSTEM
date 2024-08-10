/**
 * @file smart_irrigation.c
 * @brief Smart Irrigation System with Soil Moisture Sensor and PIR Sensor
 *
 * This program monitors soil moisture levels and detects motion using a PIR sensor.
 * It controls a relay for irrigation and activates a buzzer when motion is detected.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_log.h"

// GPIO pin definitions
#define SOIL_SENSOR_PIN    ADC1_CHANNEL_6 ///< ADC channel for soil moisture sensor (GPIO34)
#define PIR_SENSOR_PIN     GPIO_NUM_35    ///< GPIO pin for PIR sensor
#define RELAY_PIN          GPIO_NUM_2     ///< GPIO pin for relay module
#define BUZZER_PIN         GPIO_NUM_4     ///< GPIO pin for buzzer

// Threshold for soil moisture level
// Soil moisture threshold for triggering irrigation
#define MOISTURE_THRESHOLD 100 

static const char *TAG = "SmartIrrigation"; ///< Tag for logging

/**
 * @brief Main application function
 *
 * This function initializes the ADC for the soil moisture sensor, configures the GPIOs
 * for the PIR sensor, relay, and buzzer, and enters a loop to continuously monitor
 * the sensors and control the relay and buzzer accordingly.
 */
void app_main(void) {
    // Initialize ADC for soil moisture sensor
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(SOIL_SENSOR_PIN, ADC_ATTEN_DB_0);

    // Configure GPIO for PIR sensor
    gpio_config_t pir_config = {
        .pin_bit_mask = (1ULL << PIR_SENSOR_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&pir_config);

    // Configure GPIO for relay and buzzer
    gpio_set_direction(RELAY_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(BUZZER_PIN, GPIO_MODE_OUTPUT);

    while (1) {
        // Read soil moisture value
        int soil_moisture_value = adc1_get_raw(SOIL_SENSOR_PIN);
        ESP_LOGI(TAG, "Soil Moisture: %d", soil_moisture_value);

        // Read PIR sensor value
        int pir_value = gpio_get_level(PIR_SENSOR_PIN);

        // Control relay based on soil moisture
        if (soil_moisture_value < MOISTURE_THRESHOLD) {
            gpio_set_level(RELAY_PIN, 1); // Activate relay
            ESP_LOGI(TAG, "Soil is dry! Activating relay...");
        } else {
            gpio_set_level(RELAY_PIN, 0); // Deactivate relay
            ESP_LOGI(TAG, "Soil moisture is adequate.");
        }

        // Control buzzer based on PIR sensor
        if (pir_value == 1) {
            ESP_LOGI(TAG, "Motion detected! Activating buzzer...");
            gpio_set_level(BUZZER_PIN, 1); // Turn on buzzer
            vTaskDelay(pdMS_TO_TICKS(5000)); // Buzzer on for 5 seconds
            gpio_set_level(BUZZER_PIN, 0); // Turn off buzzer
        }

        vTaskDelay(pdMS_TO_TICKS(2000)); // Delay for 2 seconds before the next reading
    }
}
