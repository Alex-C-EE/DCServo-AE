/* can_module_fdcan.h */

#ifndef __CAN_MODULE_FDCAN_H
#define __CAN_MODULE_FDCAN_H

#include "stm32g0xx_hal.h"
#include "stdint.h"

/* Macro Definitions */
#define CAN_LOG_BUFFER_SIZE 256
#define CAN_RX_BUFFER_SIZE 256
#define CAN_RX_MESSAGE_SIZE 64  // FDCAN supports up to 64 bytes

/* External Variables */
extern FDCAN_HandleTypeDef hfdcan2;

/* Type Definitions */

/**
  * @brief  Structure to store CAN message data and its length.
  */
typedef struct {
    uint8_t data[CAN_RX_MESSAGE_SIZE];
    uint8_t length;
} CAN_Message_t;

/* Function Prototypes */

/**
  * @brief  Initializes the FDCAN peripheral with customizable IDs and other parameters.
  * @param  StdId: Standard Identifier (11-bit).
  * @param  DLC: Data Length Code (number of bytes in the CAN frame, up to 8 for Classic CAN).
  * @retval None
  */
void CAN_INIT(uint32_t StdId, uint32_t DLC);

/**
  * @brief  Sends a string over CAN immediately.
  * @param  msg: Pointer to the string message to be sent.
  * @retval None
  */
void CAN_LOG_NOW(char *msg);

/**
  * @brief  Stores a string message into the buffer for later transmission.
  * @param  msg: Pointer to the string message to be stored.
  * @retval None
  */
void CAN_LOG(char *msg);

/**
  * @brief  Sends out the buffered messages over CAN.
  * @retval None
  */
void CAN_TX(void);

/**
  * @brief  Retrieves the next received CAN message from the buffer.
  * @param  data: Pointer to a buffer where the received data will be copied.
  * @param  length: Pointer to a variable where the length of the data will be stored.
  * @retval 1 if a message was retrieved, 0 if no messages are available.
  */
uint8_t CAN_GetReceivedMessage(uint8_t *data, uint8_t *length);

/**
  * @brief  Converts an array of 4 bytes into a float.
  * @param  data: Pointer to the array containing the bytes.
  * @retval The converted float value.
  */
float CAN_BytesToFloat(uint8_t *data);

/**
  * @brief  Processes the received CAN messages as floats and returns the latest float value.
  * @retval Latest float value (0.0f to 360.0f) received via CAN.
  */
float CAN_ProcessReceivedMessages(void);

/**
  * @brief  This function is called when a CAN message is received.
  *         It reads the message and saves it into the receive buffer.
  * @param  hfdcan: Pointer to a FDCAN_HandleTypeDef structure that contains
  *                 the configuration information for the specified FDCAN.
  * @param  RxFifo0ITs: Receive FIFO0 interrupt flags.
  * @retval None
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);

/**
  * @brief  Converts a DLC code to the actual number of data bytes.
  * @param  dlc: The DataLength field from the RxHeader.
  * @retval The number of data bytes.
  */
uint8_t DLC_To_Bytes(uint32_t dlc);

#endif /* __CAN_MODULE_FDCAN_H */
