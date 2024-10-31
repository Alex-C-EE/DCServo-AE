/* encoder.c */
#include "main.h"
#include "config.h"

// Define constants
#define CS_PIN GPIO_PIN_4        // Change this to your CS pin
#define CS_PORT GPIOA            // Change this to your GPIO port for CS
#define FIELD_ERROR_CHECK_DELAY_MS 100  // How often to check field strength in milliseconds

extern SPI_HandleTypeDef hspi1;         // Assuming you are using SPI1, adjust if different

// Define a variable to store the received data
static volatile uint16_t spi_rx_buffer = 0;
extern volatile uint8_t spi_rx_done;    // Flag to indicate SPI data received

static double prevAngle = 0.0;
static uint32_t prevTime = 0;
static const double VELOCITY_FILTER_ALPHA = 0.2;  // Adjustable filter constant

// Function prototypes
static double ConvertRawToDegrees(uint16_t raw_value);
static uint16_t SwapBytes(uint16_t val);
void Encoder_Init(void);
static HAL_StatusTypeDef Encoder_ReadRegister(uint8_t reg_addr, uint16_t *value);
static HAL_StatusTypeDef Encoder_WriteRegister(uint8_t reg_addr, uint16_t value);
void Update_Encoder_Angle(void);
void Encoder_SetZero(void);
void CheckMagneticField(void);

// Function to convert raw SPI data to degrees
static double ConvertRawToDegrees(uint16_t raw_value) {
    // The MA730 uses 14-bit resolution, mask out unused bits
    raw_value &= 0x3FFF;  // Mask to keep only the 14 relevant bits
    // Convert the 14-bit raw value into degrees
    return ((double)raw_value / 16384.0) * 360.0;
}

// Helper function to swap bytes (big-endian to little-endian or vice versa)
static uint16_t SwapBytes(uint16_t val) {
    return (val << 8) | (val >> 8);
}

// Function to initialize encoder parameters
void Encoder_Init(void) {
    // Verify that the encoder is configured for SPI communication
    if (config.encoderComm != ENCODER_COMM_SPI) {
        // Handle incorrect configuration
        Error_Handler();
    }

    // Initialize GPIO for CS pin if needed
    // Example: Configure CS pin as output
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(CS_PORT, &GPIO_InitStruct);

    // Ensure CS is high (inactive) after initialization
    HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
}

// Helper function to read a 16-bit register from the encoder
static HAL_StatusTypeDef Encoder_ReadRegister(uint8_t reg_addr, uint16_t *value) {
    uint8_t read_cmd = 0x40 | (reg_addr & 0x3F);  // Assuming 0x40 is the read command base
    uint8_t rx_buffer[2] = {0};

    // Pull CS low to select the MA730
    HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);

    // Transmit the read command
    if (HAL_SPI_Transmit(&hspi1, &read_cmd, 1, HAL_MAX_DELAY) != HAL_OK) {
        HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
        return HAL_ERROR;
    }

    // Receive the 16-bit register value
    if (HAL_SPI_Receive(&hspi1, rx_buffer, 2, HAL_MAX_DELAY) != HAL_OK) {
        HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
        return HAL_ERROR;
    }

    // Pull CS high to release the MA730
    HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);

    // Combine received bytes into a 16-bit value (assuming big-endian)
    *value = (rx_buffer[0] << 8) | rx_buffer[1];
    *value = SwapBytes(*value);  // Swap bytes if necessary based on SPI configuration

    return HAL_OK;
}

double Encoder_CalculateVelocity(void) {
    uint32_t currentTime = HAL_GetTick();
    uint32_t deltaTime = currentTime - prevTime;

    if (deltaTime < 1) {  // Prevent division by zero
        return config.currentVelocity;
    }

    double deltaAngle = config.currentAngle - prevAngle;

    // Handle angle wrapping
    if (deltaAngle > 180.0) {
        deltaAngle -= 360.0;
    } else if (deltaAngle < -180.0) {
        deltaAngle += 360.0;
    }

    // Calculate instantaneous velocity in degrees per second
    double instantVelocity = (deltaAngle * 1000.0) / deltaTime;  // Convert ms to seconds

    // Apply low-pass filter
    config.currentVelocity = (VELOCITY_FILTER_ALPHA * instantVelocity) +
                            ((1.0 - VELOCITY_FILTER_ALPHA) * config.currentVelocity);

    // Update previous values
    prevAngle = config.currentAngle;
    prevTime = currentTime;

    return config.currentVelocity;
}

// Helper function to write a 16-bit register to the encoder
static HAL_StatusTypeDef Encoder_WriteRegister(uint8_t reg_addr, uint16_t value) {
    uint8_t write_cmd = 0x20 | (reg_addr & 0x3F);  // Assuming 0x20 is the write command base
    uint16_t swapped_value = SwapBytes(value);
    uint8_t tx_buffer[3] = {
        write_cmd,
        (swapped_value >> 8) & 0xFF,  // High byte
        swapped_value & 0xFF          // Low byte
    };

    // Pull CS low to select the MA730
    HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);

    // Transmit the write command and data
    if (HAL_SPI_Transmit(&hspi1, tx_buffer, 3, HAL_MAX_DELAY) != HAL_OK) {
        HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
        return HAL_ERROR;
    }

    // Pull CS high to release the MA730
    HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);

    return HAL_OK;
}

// Function to read MA730 data over SPI and update config
void Update_Encoder_Angle(void) {
    uint16_t angle_raw = 0;

    // Read the angle register (assuming register 0x00 contains the angle)
    if (Encoder_ReadRegister(0x00, &angle_raw) != HAL_OK) {
        // Handle read error
        Error_Handler();
    }

    // Convert the raw value to degrees and update the config structure
    config.currentAngle = ConvertRawToDegrees(angle_raw);

    // Calculate and update velocity
    Encoder_CalculateVelocity();
}

// Function to set the zero reference point to the current angle
void Encoder_SetZero(void) {
    HAL_StatusTypeDef status;
    uint16_t current_angle_raw = 0;
    uint16_t zero_setting = 0;
    uint16_t read_zero0 = 0;
    uint16_t read_zero1 = 0;

    // Step 1: Read the Current Angle
    status = Encoder_ReadRegister(0x00, &current_angle_raw);  // Assuming register 0x00 contains the angle
    if (status != HAL_OK) {
        // Handle read error
        Error_Handler();
    }

    // Step 2: Calculate the Complementary Zero Setting
    zero_setting = 65536 - current_angle_raw;

    // Step 3: Write to the Zero Setting Registers (0x00 and 0x01)
    // Ensure that the 16-bit zero_setting is correctly split into two bytes (big-endian)
    status = Encoder_WriteRegister(0x00, zero_setting);
    if (status != HAL_OK) {
        // Handle write error
        Error_Handler();
    }

    status = Encoder_WriteRegister(0x01, zero_setting);
    if (status != HAL_OK) {
        // Handle write error
        Error_Handler();
    }

    // Step 4: Wait approximately 20ms to allow non-volatile memory to update
    HAL_Delay(20);

    // Step 5: Verify the Update by reading back the registers
    status = Encoder_ReadRegister(0x00, &read_zero0);
    if (status != HAL_OK) {
        // Handle read error
        Error_Handler();
    }

    status = Encoder_ReadRegister(0x01, &read_zero1);
    if (status != HAL_OK) {
        // Handle read error
        Error_Handler();
    }

    // Check if the read values match the zero setting
    if (read_zero0 != zero_setting || read_zero1 != zero_setting) {
        // Verification failed
        Error_Handler();
    }

    // Optionally, update the current angle in the config structure
    config.currentAngle = ConvertRawToDegrees(current_angle_raw);
}

// Function to check magnetic field strength
void CheckMagneticField(void) {
    static uint32_t lastCheckTime = 0;
    uint16_t status = 0;

    // Only check periodically to avoid flooding CAN with messages
    if (HAL_GetTick() - lastCheckTime < FIELD_ERROR_CHECK_DELAY_MS) {
        return;
    }

    lastCheckTime = HAL_GetTick();

    // Read register 27 which contains MGH (bit 7) and MGL (bit 6) flags
    if (Encoder_ReadRegister(27, &status) != HAL_OK) {
        CAN_Log_Now("ERR:FIELD_READ_FAIL");
        return;
    }

    // Check MGL (bit 6) - field too low (<40mT)
    if (status & (1 << 6)) {
        CAN_Log_Now("ERR:FIELD_TOO_LOW");
    }

    // Check MGH (bit 7) - field too high (>100mT)
    if (status & (1 << 7)) {
        CAN_Log_Now("ERR:FIELD_TOO_HIGH");
    }
}

/*
 * To integrate the CheckMagneticField function into your system:
 *
 * 1. If you have a main loop, call CheckMagneticField() periodically within it.
 *    Example:
 *
 *    while (1) {
 *        Update_Encoder_Angle();
 *        CheckMagneticField();
 *        // Other tasks...
 *    }
 *
 * 2. Alternatively, if you have a scheduled encoder update routine, include
 *    CheckMagneticField() within that routine to ensure regular checks.
 *
 * Ensure that CAN_Log_Now is properly implemented to handle logging over CAN.
 */

