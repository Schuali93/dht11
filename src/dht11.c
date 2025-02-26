/*
 * DHT11 Sensor Driver
 * 
 * Author: schuali93
 * 
 * This file provides the interface for initializing and reading data from the DHT11 sensor.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "dht11.h"

static bool isInit = false;

/* ---------------------------------------------------------------------------- */

static int Dht11_wait_for_level(gpio_num_t pin, int level, int timeout);

/* ---------------------------------------------------------------------------- */

void Dht11_init( void )
{   
    gpio_set_direction(DHT11_DATA_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(DHT11_DATA_PIN, 1);

    isInit = true;
    vTaskDelay(3000 / portTICK_PERIOD_MS);  // Wait for sensor to stabilize
}

/* ---------------------------------------------------------------------------- */

int Dht11_read(int *temperature, int *humidity) 
{
    if(!isInit)
    {
        ESP_LOGE("DHT11", "Sensor not initialized. Call DHT11_init() first.");
        return -1;
    }

    uint8_t data[5] = {0};
    int checksum;

    // Send start signal
    gpio_set_direction(DHT11_DATA_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(DHT11_DATA_PIN, 0);
    vTaskDelay(20 / portTICK_PERIOD_MS);  // 20ms low
    gpio_set_level(DHT11_DATA_PIN, 1);
    esp_rom_delay_us(40);  // 40us high
    gpio_set_direction(DHT11_DATA_PIN, GPIO_MODE_INPUT);

    // Wait for DHT11 response
    if (Dht11_wait_for_level(DHT11_DATA_PIN, 0, 80) < 0) return -1;
    if (Dht11_wait_for_level(DHT11_DATA_PIN, 1, 80) < 0) return -1;
    if (Dht11_wait_for_level(DHT11_DATA_PIN, 0, 80) < 0) return -1;

    // Read data
    for (int i = 0; i < 40; i++) {
        if (Dht11_wait_for_level(DHT11_DATA_PIN, 1, 50) < 0) return -1;
        esp_rom_delay_us(28);
        data[i / 8] <<= 1;
        if (gpio_get_level(DHT11_DATA_PIN)) {
            data[i / 8] |= 1;
        }
        if (Dht11_wait_for_level(DHT11_DATA_PIN, 0, 50) < 0) return -1;
    }

    // Checksum validation
    checksum = data[0] + data[1] + data[2] + data[3];
    if (checksum != data[4]) {
        return -1;
    }

    *humidity = data[0];
    *temperature = data[2];

    return 0;
}

/* ---------------------------------------------------------------------------- */

static int Dht11_wait_for_level(gpio_num_t pin, int level, int timeout) 
{
    int us = 0;
    while (gpio_get_level(pin) != level) {
        if (us++ > timeout) {
            return -1;
        }
        esp_rom_delay_us(1);
    }
    return us;
}
