/* can.h */
#ifndef CAN_H
#define CAN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "config.h"         // Ensure Config_t and related types are defined here
#include "fdcan.h"          // Include the FDCAN HAL library header
#include <stdint.h>
#include <stdbool.h>

/* Constants -----------------------------------------------------------------*/

/* CAN Protocol Version */
#define CAN_PROTOCOL_VERSION        "1.0.0"

/* CAN TX and RX Buffer Sizes */
#define CAN_LOG_BUFFER_SIZE         256
#define CAN_RX_BUFFER_SIZE          64
#define CAN_MAX_MESSAGE_SIZE        8

/* CAN Transmission Parameters */
#define CAN_TX_RETRY_LIMIT          3
#define CAN_TX_RETRY_DELAY_MS        10
#define CAN_RX_MESSAGE_TIMEOUT_MS  100

/* Old Command Strings (for reference/backward compatibility) */
#define CMD_SET_TAU_LONG                   "SET_TAU"
#define CMD_SET_KAW_LONG                   "SET_KAW"
#define CMD_SET_DISTURBANCE_THRESHOLD_LONG "SET_DISTURBANCE_THRESHOLD"

/* New Shortened Command Strings */
#define CMD_SET_MAX_RPM     "SMR"    // SET_MAX_RPM
#define CMD_SET_ENC_TYPE    "SET"    // SET_ENCODER_TYPE
#define CMD_SET_ENC_COMM    "SEC"    // SET_ENCODER_COMM
#define CMD_SET_TRG_ANG     "STA"    // SET_TARGET_ANGLE
#define CMD_SET_SPEED       "SSP"    // SET_SPEED
#define CMD_POWER_MODE      "PMD"    // POWER_MODE
#define CMD_SET_ACC         "SAC"    // SET_ACC
#define CMD_SET_JERK        "SJK"    // SET_JERK
#define CMD_SET_LIM_UP      "SLU"    // SET_LIMIT_UP
#define CMD_SET_LIM_DN      "SLD"    // SET_LIMIT_DOWN
#define CMD_SET_ERR_LIM     "SEL"    // SET_ERROR_LIM
#define CMD_SET_FEEDBACK    "SFB"    // SET_FEEDBACK
#define CMD_SET_PID_KP      "SKP"    // SET_PID_KP
#define CMD_SET_PID_KI      "SKI"    // SET_PID_KI
#define CMD_SET_PID_KD      "SKD"    // SET_PID_KD
#define CMD_SET_TAU         "STU"    // SET_TAU
#define CMD_SET_KAW         "SKW"    // SET_KAW
#define CMD_SET_DIST_THR    "SDT"    // SET_DISTURBANCE_THRESHOLD

/* Limits for New Parameters */
#define MIN_TAU                         0.001    /* Minimum allowable tau */
#define MAX_TAU                         1.0      /* Maximum allowable tau */

#define MIN_KAW                         0.0      /* Minimum allowable Kaw */
#define MAX_KAW                         10.0     /* Maximum allowable Kaw */

#define MIN_DISTURBANCE_THRESHOLD       0.0      /* Minimum disturbance threshold */
#define MAX_DISTURBANCE_THRESHOLD       360.0    /* Maximum disturbance threshold */

/* Enumerations ---------------------------------------------------------------*/

/**
 * @brief  CAN Error Codes Enumeration
 */
typedef enum {
    CAN_ERROR_NONE = 0,
    CAN_ERROR_FILTER_CONFIG,
    CAN_ERROR_BITTIMING_CONFIG,
    CAN_ERROR_START,
    CAN_ERROR_ACTIVATE_NOTIFICATION,
    CAN_ERROR_TRANSMIT,
    CAN_ERROR_RECEIVE,
    CAN_ERROR_BUS_OFF,
    CAN_ERROR_OVERRUN,
    CAN_ERROR_INVALID_MESSAGE,
    CAN_ERROR_UNKNOWN,
    // Add more as needed
} CAN_ErrorCode_t;

/* Function Prototypes --------------------------------------------------------*/

/**
 * @brief  Initializes the CAN peripheral with the specified Standard ID.
 * @param  StdId Standard Identifier for CAN messages.
 * @retval None
 *
 * @note   This function configures the CAN filters, bit timing, starts the CAN peripheral,
 *         and activates necessary notifications for message reception and error handling.
 */
void CAN_Init(uint32_t StdId);

/**
 * @brief  Logs a message by adding it to the CAN TX circular buffer.
 * @param  message Pointer to the null-terminated string message to log.
 * @retval None
 *
 * @note   This function does not transmit the message immediately but queues it for transmission.
 */
void CAN_Log(const char *message);

/**
 * @brief  Logs a high-priority message by transmitting it immediately with a retry mechanism.
 * @param  message Pointer to the null-terminated string message to log.
 * @retval None
 *
 * @note   This function bypasses the TX buffer and attempts to send the message directly.
 */
void CAN_Log_Now(const char *message);

/**
 * @brief  Processes and transmits messages from the CAN TX circular buffer.
 * @retval None
 *
 * @note   This function should be called periodically, e.g., within the main loop or a dedicated task.
 */
void CAN_TX(void);

/**
 * @brief  Processes received CAN messages from the RX circular buffer.
 * @retval None
 *
 * @note   This function should be called periodically to handle incoming messages.
 */
void CAN_ProcessReceivedMessages(void);

/* Utility Functions ----------------------------------------------------------*/

/**
 * @brief  Converts a CAN Data Length Code (DLC) to the corresponding byte count.
 * @param  dlc The Data Length Code.
 * @retval The number of bytes corresponding to the DLC.
 */
uint8_t CAN_DLC_To_Bytes(uint32_t dlc);

/**
 * @brief  Converts a byte count to the corresponding CAN Data Length Code (DLC).
 * @param  byteCount The number of bytes.
 * @retval The Data Length Code corresponding to the byte count.
 */
uint32_t CAN_ByteToDLC(uint8_t byteCount);

/* Error Handling -------------------------------------------------------------*/

/**
 * @brief  Sends an error message over CAN based on the specified error code.
 * @param  errorCode The CAN error code to send.
 * @retval None
 *
 * @note   This function logs the error message and increments the corresponding error counter.
 */
void CAN_SendError(CAN_ErrorCode_t errorCode);

/* Logging Functions ----------------------------------------------------------*/

/**
 * @brief  Logs a formatted message immediately over CAN.
 * @param  format The format string (printf-style).
 * @param  ...    Variable arguments corresponding to the format string.
 * @retval None
 *
 * @note   This function formats the message and sends it using CAN_Log_Now.
 */
void CAN_Logf_Now(const char *format, ...);

/**
 * @brief  Logs a formatted message by adding it to the CAN TX buffer.
 * @param  format The format string (printf-style).
 * @param  ...    Variable arguments corresponding to the format string.
 * @retval None
 *
 * @note   This function formats the message and queues it using CAN_Log.
 */
void CAN_Logf(const char *format, ...);

/* Callback Functions ---------------------------------------------------------*/

/**
 * @brief  Callback function executed when a new CAN message is received in FIFO0.
 * @param  hfdcan Pointer to the CAN handle.
 * @param  RxFifo0ITs Interrupt flags.
 * @retval None
 *
 * @note   This function should be registered with the HAL FDCAN library to handle incoming messages.
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);

/* Transmission Functions -----------------------------------------------------*/

/**
 * @brief  Transmits a CAN message with a retry mechanism.
 * @param  message Pointer to the null-terminated string message to transmit.
 * @retval None
 *
 * @note   This function attempts to send the message up to CAN_TX_RETRY_LIMIT times with delays in between.
 */
void CAN_TransmitWithRetry(const char *message);

/* Reception Functions --------------------------------------------------------*/

/**
 * @brief  Retrieves the next received CAN message from the RX buffer.
 * @param  data    Pointer to the buffer where the message data will be stored.
 * @param  length  Pointer to a variable where the length of the received message will be stored.
 * @retval true   If a message was successfully retrieved.
 * @retval false  If the RX buffer is empty.
 */
bool CAN_GetReceivedMessage(uint8_t *data, uint8_t *length);

/* Command Processing Functions -----------------------------------------------*/

/**
 * @brief  Validates the format of a received CAN message.
 * @param  message Pointer to the null-terminated string message to validate.
 * @retval true   If the message format is valid.
 * @retval false  If the message format is invalid.
 */
bool CAN_ValidateMessage(const char *message);

/**
 * @brief  Parses and applies a received CAN message to update the system configuration.
 * @param  message Pointer to the null-terminated string message to parse.
 * @retval true   If the message was successfully parsed and applied.
 * @retval false  If the message parsing or application failed.
 */
bool CAN_ParseMessage(const char *message);

/* Bus Recovery Functions -----------------------------------------------------*/

/**
 * @brief  Attempts to recover the CAN bus from a bus-off state.
 * @retval true   If recovery was successful.
 * @retval false  If recovery failed.
 */
bool CAN_RecoverBusOff(void);

/* Checksum Functions ---------------------------------------------------------*/

/**
 * @brief  Calculates a simple checksum for a given message.
 * @param  message Pointer to the null-terminated string message.
 * @retval The calculated checksum as a 16-bit unsigned integer.
 */
uint16_t CAN_CalculateChecksum(const char *message);

/* Utility Functions ----------------------------------------------------------*/

/**
 * @brief  Converts an array of bytes to a double precision floating-point number.
 * @param  data Pointer to the byte array.
 * @retval The converted double value.
 */
double CAN_BytesToDouble(uint8_t *data);

/**
 * @brief  Converts an array of bytes to a 16-bit signed integer.
 * @param  data Pointer to the byte array.
 * @retval The converted 16-bit signed integer.
 */
int16_t CAN_BytesToInt16(uint8_t *data);

/* Logging Helpers ------------------------------------------------------------*/

/**
 * @brief  Retrieves the full command name corresponding to a shortened command string.
 * @param  shortCmd Pointer to the shortened command string.
 * @retval Pointer to the full command name string.
 *
 * @note   Returns "UNKNOWN_COMMAND" if the short command does not match any known commands.
 */
const char* CAN_GetLongCommandName(const char* shortCmd);

/* Logging and Transmission ---------------------------------------------------*/

/**
 * @brief  Processes and transmits messages from the CAN TX circular buffer.
 * @retval None
 *
 * @note   This function should be called periodically, e.g., within the main loop or a dedicated task.
 */
void CAN_ProcessLogBuffer(void);

/**
 * @brief  Logs a message by adding it to the CAN TX circular buffer.
 * @param  message Pointer to the null-terminated string message to log.
 * @retval None
 *
 * @note   This function adds the message to the TX buffer. Use CAN_ProcessLogBuffer to transmit.
 */
void CAN_LogBuffer_Add(const char *message);

/**
 * @brief  Transmits all messages currently in the CAN TX circular buffer.
 * @retval None
 *
 * @note   This function iterates through the TX buffer and attempts to send each message.
 */
void CAN_TX_BufferTransmit(void);

#ifdef __cplusplus
}
#endif

#endif /* CAN_H */
