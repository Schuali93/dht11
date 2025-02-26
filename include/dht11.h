/*
 * DHT11 Sensor Driver
 * 
 * Author: schuali93
 * 
 * This file provides the interface for initializing and reading data from the DHT11 sensor.
 */

#ifndef DHT11_H
#define DHT11_H

#include "driver/gpio.h"

#define DHT11_DATA_PIN GPIO_NUM_2

/**
 * @brief Initialize the DHT11 sensor.
 *
 * This function initializes the DHT11 sensor by setting the specified GPIO pin
 * to output mode and setting it high. It also waits for the sensor to stabilize.
 */
void Dht11_init( void );

/**
 * @brief Read data from the DHT11 sensor.
 *
 * This function reads the temperature and humidity data from the DHT11 sensor.
 * It sends a start signal to the sensor, waits for the sensor's response, and
 * then reads the data. The function returns the temperature and humidity values
 * through the provided pointers.
 *
 * @param temperature Pointer to store the read temperature value.
 * @param humidity Pointer to store the read humidity value.
 * @return 0 on success, -1 on failure.
 */
int Dht11_read(int *temperature, int *humidity);

#endif // DHT11_H