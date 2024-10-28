/* can_module_fdcan.c */

#include "stm32g0xx_hal.h"
#include "string.h"

#define CAN_LOG_BUFFER_SIZE 256
#define CAN_RX_BUFFER_SIZE 256  // Size of the receive buffer
#define CAN_RX_MESSAGE_SIZE 64  // FDCAN supports up to 64 bytes

extern FDCAN_HandleTypeDef hfdcan2;  // Assume hfdcan2 is defined in your main application

static FDCAN_TxHeaderTypeDef TxHeader;
static FDCAN_RxHeaderTypeDef RxHeader;

char CAN_LogBuffer[CAN_LOG_BUFFER_SIZE];
uint16_t CAN_LogBufferIndex = 0;

// Receive buffer to store incoming CAN messages
typedef struct {
    uint8_t data[CAN_RX_MESSAGE_SIZE];
    uint8_t length;
} CAN_Message_t;

CAN_Message_t CAN_RxBuffer[CAN_RX_BUFFER_SIZE];
uint16_t CAN_RxBufferHead = 0;  // Points to the next free slot in the buffer
uint16_t CAN_RxBufferTail = 0;  // Points to the next message to be processed

// Function to map chunkSize to DLC code
uint32_t DataLengthCode[9] = {
    FDCAN_DLC_BYTES_0,
    FDCAN_DLC_BYTES_1,
    FDCAN_DLC_BYTES_2,
    FDCAN_DLC_BYTES_3,
    FDCAN_DLC_BYTES_4,
    FDCAN_DLC_BYTES_5,
    FDCAN_DLC_BYTES_6,
    FDCAN_DLC_BYTES_7,
    FDCAN_DLC_BYTES_8
};

/**
  * @brief  Converts a DLC code to the actual number of data bytes.
  * @param  dlc: The DataLength field from the RxHeader.
  * @retval The number of data bytes.
  */
uint8_t DLC_To_Bytes(uint32_t dlc)
{
    switch (dlc)
    {
        case FDCAN_DLC_BYTES_0: return 0;
        case FDCAN_DLC_BYTES_1: return 1;
        case FDCAN_DLC_BYTES_2: return 2;
        case FDCAN_DLC_BYTES_3: return 3;
        case FDCAN_DLC_BYTES_4: return 4;
        case FDCAN_DLC_BYTES_5: return 5;
        case FDCAN_DLC_BYTES_6: return 6;
        case FDCAN_DLC_BYTES_7: return 7;
        case FDCAN_DLC_BYTES_8: return 8;
        default: return 0;
    }
}

/**
  * @brief  Initializes the FDCAN peripheral with customizable IDs and other parameters.
  * @param  StdId: Standard Identifier (11-bit).
  * @param  DLC: Data Length Code (number of bytes in the CAN frame).
  * @retval None
  */
void CAN_INIT(uint32_t StdId, uint32_t DLC)
{
    // Configure the FDCAN Tx Header
    TxHeader.Identifier = StdId;           // Set the standard ID
    TxHeader.IdType = FDCAN_STANDARD_ID;   // Use standard ID
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = DataLengthCode[DLC]; // Use the correct DLC code
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF; // Disable Bit Rate Switching
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;  // Use Classic CAN format
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    // Configure the FDCAN Filter to accept all messages
    FDCAN_FilterTypeDef sFilterConfig;
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x000;      // Accept all IDs
    sFilterConfig.FilterID2 = 0x000;

    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
    {
        // Filter configuration Error
        Error_Handler();
    }

    // Start the FDCAN peripheral
    if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
    {
        // Starting Error
        Error_Handler();
    }

    // Activate FDCAN RX FIFO 0 new message notification
    if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        // Notification Error
        Error_Handler();
    }
}

/**
  * @brief  Sends a string over CAN immediately.
  * @param  msg: Pointer to the string message to be sent.
  * @retval None
  */
void CAN_LOG_NOW(char *msg)
{
    uint8_t data[64];  // FDCAN supports up to 64 bytes
    uint16_t len = strlen(msg);
    uint16_t index = 0;

    while (len > 0)
    {
        uint8_t chunkSize = (len > 8) ? 8 : len;  // For Classic CAN, max 8 bytes
        memset(data, 0x00, 64);  // Clear the data buffer
        memcpy(data, &msg[index], chunkSize);

        TxHeader.DataLength = DataLengthCode[chunkSize];

        // Transmit the message
        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, data) != HAL_OK)
        {
            // Transmission request Error
            Error_Handler();
        }

        index += chunkSize;
        len -= chunkSize;
    }
}

/**
  * @brief  Stores a string message into the buffer for later transmission.
  * @param  msg: Pointer to the string message to be stored.
  * @retval None
  */
void CAN_LOG(char *msg)
{
    uint16_t msg_len = strlen(msg);

    if (CAN_LogBufferIndex + msg_len >= CAN_LOG_BUFFER_SIZE)
    {
        // Buffer overflow handling
        CAN_LogBufferIndex = 0; // Reset buffer index if overflow occurs
    }

    strcpy(&CAN_LogBuffer[CAN_LogBufferIndex], msg);
    CAN_LogBufferIndex += msg_len;
}

/**
  * @brief  Sends out the buffered messages over CAN.
  * @retval None
  */
void CAN_TX(void)
{
    uint8_t data[64];  // FDCAN supports up to 64 bytes
    uint16_t index = 0;
    uint16_t len = CAN_LogBufferIndex;

    while (len > 0)
    {
        uint8_t chunkSize = (len > 8) ? 8 : len;  // For Classic CAN, max 8 bytes
        memset(data, 0x00, 64);  // Clear the data buffer
        memcpy(data, &CAN_LogBuffer[index], chunkSize);

        TxHeader.DataLength = DataLengthCode[chunkSize];

        // Transmit the message
        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, data) != HAL_OK)
        {
            // Transmission request Error
            Error_Handler();
        }

        index += chunkSize;
        len -= chunkSize;
    }

    // Reset the buffer index after transmission
    CAN_LogBufferIndex = 0;
}

/**
  * @brief  This function is called when a CAN message is received.
  *         It reads the message and saves it into the receive buffer.
  * @param  hfdcan: pointer to a FDCAN_HandleTypeDef structure that contains
  *               the configuration information for the specified FDCAN.
  * @param  RxFifo0ITs: Receive FIFO0 interrupts flags.
  * @retval None
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (hfdcan->Instance == hfdcan2.Instance)
    {
        if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
        {
            uint8_t data[64];
            if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, data) != HAL_OK)
            {
                // Reception Error
                Error_Handler();
            }

            // Get the actual data length in bytes from the DLC code
            uint8_t data_length = DLC_To_Bytes(RxHeader.DataLength);

            // Save the received data and length into the receive buffer
            memcpy(CAN_RxBuffer[CAN_RxBufferHead].data, data, data_length);
            CAN_RxBuffer[CAN_RxBufferHead].length = data_length;

            // Update the buffer head
            CAN_RxBufferHead = (CAN_RxBufferHead + 1) % CAN_RX_BUFFER_SIZE;

            // Check for buffer overflow
            if (CAN_RxBufferHead == CAN_RxBufferTail)
            {
                // Buffer overflow handling: move tail to next message to overwrite oldest message
                CAN_RxBufferTail = (CAN_RxBufferTail + 1) % CAN_RX_BUFFER_SIZE;
            }
        }
    }
}

/**
  * @brief  Retrieves the next received CAN message from the buffer.
  * @param  data: Pointer to a buffer where the received data will be copied.
  * @param  length: Pointer to a variable where the length of the data will be stored.
  * @retval 1 if a message was retrieved, 0 if no messages are available.
  */
uint8_t CAN_GetReceivedMessage(uint8_t *data, uint8_t *length)
{
    if (CAN_RxBufferHead == CAN_RxBufferTail)
    {
        // No new messages
        return 0;
    }

    // Copy the data and length from the buffer
    *length = CAN_RxBuffer[CAN_RxBufferTail].length;
    memcpy(data, CAN_RxBuffer[CAN_RxBufferTail].data, *length);

    // Update the buffer tail
    CAN_RxBufferTail = (CAN_RxBufferTail + 1) % CAN_RX_BUFFER_SIZE;

    return 1;
}

/**
  * @brief  Converts an array of 4 bytes into a float.
  * @param  data: Pointer to the array containing the bytes.
  * @retval The converted float value.
  */
float CAN_BytesToFloat(uint8_t *data)
{
    float value;
    memcpy(&value, data, sizeof(float));  // Copy the 4 bytes into a float variable
    return value;
}

/**
  * @brief  Converts an array of 2 bytes into a signed int16_t.
  * @param  data: Pointer to the array containing the bytes.
  * @retval The converted int16_t value.
  */
int16_t CAN_BytesToInt16(uint8_t *data)
{
    int16_t value;
    memcpy(&value, data, sizeof(int16_t));  // Copy the 2 bytes into an int16_t variable
    return value;
}

/**
  * @brief  Processes the received CAN messages, extracting a float and a signed int.
  *         The first 4 bytes are interpreted as a float (0.0f to 360.0f),
  *         and the next 2 bytes as a signed int (-100 to +100).
  *         Stores the latest valid values in global variables.
  * @retval None
  */
float latest_CAN_value = 0.0f;      // Store the latest float value from the CAN messages
int16_t latest_CAN_int_value = 0;   // Store the latest signed int value from the CAN messages

void CAN_ProcessReceivedMessages(void)
{
    uint8_t data[64];
    uint8_t length;

    // Process all messages in the buffer
    while (CAN_GetReceivedMessage(data, &length))
    {
        // Ensure the message is at least 6 bytes (size of a float + int16_t)
        if (length >= (sizeof(float) + sizeof(int16_t)))
        {
            // Convert the first 4 bytes into a float
            float received_value = CAN_BytesToFloat(data);

            // Convert the next 2 bytes into an int16_t
            int16_t received_int_value = CAN_BytesToInt16(&data[sizeof(float)]);

            // Ensure the float value is within the expected range (0.0f to 360.0f)
            // and the int16_t value is within the expected range (-100 to +100)
            if (received_value >= 0.0f && received_value <= 360.0f &&
                received_int_value >= -100 && received_int_value <= 100)
            {
                latest_CAN_value = received_value;          // Update the latest valid float value
                latest_CAN_int_value = received_int_value;  // Update the latest valid int16_t value
            }
        }
    }
}
