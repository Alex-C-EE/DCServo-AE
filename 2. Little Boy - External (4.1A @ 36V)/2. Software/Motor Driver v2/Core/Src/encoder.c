// Handles getting data from the controller

#include "main.h"

// Define the CS pin and SPI handle
#define CS_PIN GPIO_PIN_4   // Change this to your CS pin
#define CS_PORT GPIOA       // Change this to your GPIO port for CS
extern SPI_HandleTypeDef hspi1; // Assuming you are using SPI1, adjust if different

// Define a variable to store the received data
volatile uint16_t spi_rx_buffer = 0;
extern volatile uint8_t spi_rx_done; // Flag to indicate SPI data received

// Function to convert raw SPI data to degrees
float ConvertRawToDegrees(uint16_t raw_value) {
    // Convert the 16-bit raw value into degrees (assuming 16-bit resolution)
    return ((float)raw_value / 65536.0f) * 360.0f;
}

// Function to read MA730 data over SPI using interrupt and convert to degrees
float Read_MA730_Angle(void) {
    uint16_t angle_raw = 0;
    float angle_degrees = 0.0f;

    // Pull CS pin low to select the MA730
    HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);

    // Start SPI communication to read 16 bits (2 bytes) from the MA730
    HAL_SPI_Receive_IT(&hspi1, (uint8_t*)&spi_rx_buffer, 2); // Read 2 bytes asynchronously // TODO: Technically PID will be running with Angle Value from 1 run previous...

    // CS pin high to release the MA730
    HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);

    // Clear the received flag for future readings
    spi_rx_done = 0;

    // Swap bytes if necessary (SPI might send MSB first depending on configuration)
    angle_raw = (spi_rx_buffer << 8) | (spi_rx_buffer >> 8); // Handle endian if needed

    // Convert the raw value to degrees
    angle_degrees = ConvertRawToDegrees(angle_raw);

    return angle_degrees;
}
