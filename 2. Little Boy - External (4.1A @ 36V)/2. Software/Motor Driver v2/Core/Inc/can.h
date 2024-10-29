#ifndef CAN_H
#define CAN_H

#include "stm32g0xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* Constants */
#define CAN_LOG_BUFFER_SIZE    256
#define CAN_RX_BUFFER_SIZE     256  // Size of the receive buffer
#define CAN_MAX_MESSAGE_SIZE   64   // FDCAN supports up to 64 bytes

/* Function Prototypes */
void CAN_Init(uint32_t StdId);
void CAN_Log_Now(const char *message);
void CAN_Log(const char *message);
void CAN_TX(void);
void CAN_ProcessReceivedMessages(void);

/* Data Conversion Functions */
float CAN_BytesToFloat(uint8_t *data);
int16_t CAN_BytesToInt16(uint8_t *data);

#endif /* CAN_H */
