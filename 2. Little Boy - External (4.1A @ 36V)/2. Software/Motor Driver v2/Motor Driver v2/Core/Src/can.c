/* can.c */
#include "can.h"
#include "config.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <ctype.h>
#include <errno.h>

/* Define Missing Constants */
#define CAN_LOG_BUFFER_SIZE         256
#define CAN_RX_BUFFER_SIZE          64
#define CAN_MAX_MESSAGE_SIZE        8
#define CAN_TX_RETRY_LIMIT          3
#define CAN_TX_RETRY_DELAY_MS        10
#define CAN_RX_MESSAGE_TIMEOUT_MS  100
#define CAN_PROTOCOL_VERSION        "1.0.0"

/* Define Old Command Strings (kept for reference/backward compatibility) */
#define CMD_SET_TAU_LONG                   "SET_TAU"
#define CMD_SET_KAW_LONG                   "SET_KAW"
#define CMD_SET_DISTURBANCE_THRESHOLD_LONG "SET_DISTURBANCE_THRESHOLD"

/* Define New Shortened Command Strings */
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

/* Define Limits for New Parameters */
#define MIN_TAU                         0.001    /* Example: Minimum allowable tau */
#define MAX_TAU                         1.0      /* Example: Maximum allowable tau */

#define MIN_KAW                         0.0      /* Example: Minimum allowable Kaw */
#define MAX_KAW                         10.0     /* Example: Maximum allowable Kaw */

#define MIN_DISTURBANCE_THRESHOLD       0.0      /* Example: Minimum disturbance threshold */
#define MAX_DISTURBANCE_THRESHOLD       360.0    /* Example: Maximum disturbance threshold */

/* External FDCAN Handle (to be defined in your main application) */
extern FDCAN_HandleTypeDef hfdcan1;

/* CAN Tx and Rx Headers */
static FDCAN_TxHeaderTypeDef TxHeader;
static FDCAN_RxHeaderTypeDef RxHeader;

/* TX Buffer as Circular Buffer for non-urgent messages */
typedef struct {
    char buffer[CAN_LOG_BUFFER_SIZE];
    uint16_t head;
    uint16_t tail;
} CircularBuffer_t;

static CircularBuffer_t CAN_LogBuffer = { .head = 0, .tail = 0 };

/* Receive Buffer as Circular Buffer */
typedef struct {
    uint8_t data[CAN_RX_BUFFER_SIZE][CAN_MAX_MESSAGE_SIZE];
    uint8_t length[CAN_RX_BUFFER_SIZE];
    uint16_t head;
    uint16_t tail;
} CAN_RxCircularBuffer_t;

static CAN_RxCircularBuffer_t CAN_RxBuffer = { .head = 0, .tail = 0 };

/* Error Codes */
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

/* Diagnostic Counters */
static uint32_t CAN_ErrorCounters[CAN_ERROR_UNKNOWN + 1] = {0};

/* Data Length Code to Byte Count Mapping */
static uint8_t DLC_To_Bytes(uint32_t dlc) {
    switch (dlc) {
        case FDCAN_DLC_BYTES_0: return 0;
        case FDCAN_DLC_BYTES_1: return 1;
        case FDCAN_DLC_BYTES_2: return 2;
        case FDCAN_DLC_BYTES_3: return 3;
        case FDCAN_DLC_BYTES_4: return 4;
        case FDCAN_DLC_BYTES_5: return 5;
        case FDCAN_DLC_BYTES_6: return 6;
        case FDCAN_DLC_BYTES_7: return 7;
        case FDCAN_DLC_BYTES_8: return 8;
        case FDCAN_DLC_BYTES_12: return 12;
        case FDCAN_DLC_BYTES_16: return 16;
        case FDCAN_DLC_BYTES_20: return 20;
        case FDCAN_DLC_BYTES_24: return 24;
        case FDCAN_DLC_BYTES_32: return 32;
        case FDCAN_DLC_BYTES_48: return 48;
        case FDCAN_DLC_BYTES_64: return 64;
        default: return 8; // Default to 8 bytes if unknown
    }
}

/* Byte Count to DLC Mapping */
static uint32_t ByteToDLC(uint8_t byteCount) {
    switch(byteCount) {
        case 0: return FDCAN_DLC_BYTES_0;
        case 1: return FDCAN_DLC_BYTES_1;
        case 2: return FDCAN_DLC_BYTES_2;
        case 3: return FDCAN_DLC_BYTES_3;
        case 4: return FDCAN_DLC_BYTES_4;
        case 5: return FDCAN_DLC_BYTES_5;
        case 6: return FDCAN_DLC_BYTES_6;
        case 7: return FDCAN_DLC_BYTES_7;
        case 8: return FDCAN_DLC_BYTES_8;
        case 12: return FDCAN_DLC_BYTES_12;
        case 16: return FDCAN_DLC_BYTES_16;
        case 20: return FDCAN_DLC_BYTES_20;
        case 24: return FDCAN_DLC_BYTES_24;
        case 32: return FDCAN_DLC_BYTES_32;
        case 48: return FDCAN_DLC_BYTES_48;
        case 64: return FDCAN_DLC_BYTES_64;
        default: return FDCAN_DLC_BYTES_8; // Default to 8 bytes if unknown
    }
}

/* Function Prototypes */
static void CAN_SendError(CAN_ErrorCode_t errorCode);
static bool CAN_ValidateMessage(const char *message);
static bool CAN_ParseMessage(const char *message);
static bool CAN_RecoverBusOff(void);
static bool CAN_AddToLogBuffer(const char *message);
static bool CAN_GetFromLogBuffer(char *message);
static bool CAN_AddToRxBuffer(uint8_t *data, uint8_t length);
static bool CAN_GetFromRxBuffer(uint8_t *data, uint8_t *length);
static void CAN_TransmitWithRetry(const char *message);
static uint16_t CalculateChecksum(const char *message);
static bool ParseDouble(const char *str, double *value, double min, double max);

/* Helper Function to Parse Double Values */
static bool ParseDouble(const char *str, double *value, double min, double max) {
    char *endptr;
    errno = 0;
    *value = strtod(str, &endptr);
    if (endptr == str || *endptr != '\0' || errno == ERANGE) {
        return false;
    }
    if (*value < min || *value > max) {
        return false;
    }
    return true;
}

/* Initialization Function */
void CAN_Init(uint32_t StdId) {
    /* Configure the FDCAN Tx Header */
    TxHeader.Identifier = StdId;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;  // Default to 8 bytes
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;   // No bitrate switching
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;    // Change to FDCAN_FD_CAN if using CAN FD
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    /* Configure RX Filters to Accept Specific Messages */
    FDCAN_FilterTypeDef sFilterConfig;
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x100; // Example: Accept messages with ID >= 0x100
    sFilterConfig.FilterID2 = 0x7FF; // Mask to match the exact ID

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
        /* Filter configuration Error */
        CAN_SendError(CAN_ERROR_FILTER_CONFIG);
    }

    /* Configure Bit Timing */
    FDCAN_TimingTypeDef sTiming;
    sTiming.PropSeg = 1;
    sTiming.PhaseSeg1 = 14;
    sTiming.PhaseSeg2 = 6;
    sTiming.SyncJumpWidth = 1;
    sTiming.DataPropSeg = 1;
    sTiming.DataPhaseSeg1 = 14;
    sTiming.DataPhaseSeg2 = 6;
    sTiming.DataSyncJumpWidth = 1;

    if (HAL_FDCAN_ConfigNominalTime(&hfdcan1, &sTiming) != HAL_OK) {
        /* Bit Timing Configuration Error */
        CAN_SendError(CAN_ERROR_BITTIMING_CONFIG); // Separate error code
    }

    /* Start the FDCAN module */
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        /* Starting Error */
        CAN_SendError(CAN_ERROR_START);
    }

    /* Activate RX FIFO 0 New Message Notification and Error Notifications */
    if (HAL_FDCAN_ActivateNotification(&hfdcan1,
        FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
        FDCAN_IT_ERROR_WARNING |
        FDCAN_IT_ERROR_PASSIVE |
        FDCAN_IT_BUS_OFF, 0) != HAL_OK) {
        /* Notification Error */
        CAN_SendError(CAN_ERROR_ACTIVATE_NOTIFICATION);
    }

    /* Log Protocol Version */
    CAN_Log_Now("CAN Protocol Version: " CAN_PROTOCOL_VERSION);
}

/* Send High-Priority Message Immediately with Retry Mechanism */
static void CAN_TransmitWithRetry(const char *message) {
    if (message == NULL) {
        CAN_SendError(CAN_ERROR_TRANSMIT);
        return;
    }

    uint16_t len = strlen(message);
    uint16_t index = 0;
    uint16_t retries = 0;
    uint16_t max_retries = CAN_TX_RETRY_LIMIT;

    while (len > 0 && retries < max_retries) {
        uint8_t chunkSize = (len > CAN_MAX_MESSAGE_SIZE) ? CAN_MAX_MESSAGE_SIZE : len;
        memset(&TxHeader, 0, sizeof(TxHeader));        // Clear previous header
        TxHeader.Identifier = TxHeader.Identifier;     // Use the same identifier
        TxHeader.IdType = FDCAN_STANDARD_ID;
        TxHeader.TxFrameType = FDCAN_DATA_FRAME;
        TxHeader.DataLength = ByteToDLC(chunkSize);
        TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        TxHeader.BitRateSwitch = FDCAN_BRS_OFF;        // No bitrate switching
        TxHeader.FDFormat = FDCAN_CLASSIC_CAN;        // Change to FDCAN_FD_CAN if using CAN FD
        TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        TxHeader.MessageMarker = 0;

        uint8_t data[CAN_MAX_MESSAGE_SIZE] = {0};
        strncpy((char*)data, &message[index], chunkSize);

        /* Calculate and Append Checksum */
        uint16_t checksum = CalculateChecksum(message);
        // Assuming checksum is appended as last two bytes
        if (chunkSize >= CAN_MAX_MESSAGE_SIZE - 2) { // Reserve space for checksum
            data[chunkSize - 2] = (checksum >> 8) & 0xFF;
            data[chunkSize - 1] = checksum & 0xFF;
            TxHeader.DataLength = ByteToDLC(chunkSize);
        }

        /* Transmit the message */
        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, data) != HAL_OK) {
            /* Transmission request Error */
            CAN_SendError(CAN_ERROR_TRANSMIT);
            retries++;
            HAL_Delay(CAN_TX_RETRY_DELAY_MS); // Delay before retry
            continue;
        }

        /* Wait for transmission to complete or timeout */
        // Implement a timeout mechanism if necessary
        // For simplicity, assuming transmission is non-blocking

        index += chunkSize;
        len -= chunkSize;
        retries = 0; // Reset retries after a successful send
    }

    if (retries >= max_retries) {
        CAN_SendError(CAN_ERROR_TRANSMIT);
    }
}

/* Add Message to TX Circular Buffer */
static bool CAN_AddToLogBuffer(const char *message) {
    if (message == NULL) {
        CAN_SendError(CAN_ERROR_TRANSMIT);
        return false;
    }

    uint16_t msg_len = strlen(message);

    // Calculate available space
    uint16_t next_head = (CAN_LogBuffer.head + msg_len + 1) % CAN_LOG_BUFFER_SIZE; // +1 for null terminator
    if (next_head == CAN_LogBuffer.tail) {
        // Buffer is full
        CAN_SendError(CAN_ERROR_OVERRUN);
        return false;
    }

    // Add message to buffer
    memcpy(&CAN_LogBuffer.buffer[CAN_LogBuffer.head], message, msg_len);
    CAN_LogBuffer.head = (CAN_LogBuffer.head + msg_len) % CAN_LOG_BUFFER_SIZE;
    CAN_LogBuffer.buffer[CAN_LogBuffer.head] = '\0'; // Null-terminate
    CAN_LogBuffer.head = (CAN_LogBuffer.head + 1) % CAN_LOG_BUFFER_SIZE;

    return true;
}

/* Retrieve Message from TX Circular Buffer */
static bool CAN_GetFromLogBuffer(char *message) {
    if (CAN_LogBuffer.tail == CAN_LogBuffer.head) {
        // Buffer is empty
        return false;
    }

    // Read until null terminator
    uint16_t index = CAN_LogBuffer.tail;
    uint16_t msg_len = 0;
    while (CAN_LogBuffer.buffer[index] != '\0' && msg_len < CAN_MAX_MESSAGE_SIZE) {
        message[msg_len++] = CAN_LogBuffer.buffer[index];
        index = (index + 1) % CAN_LOG_BUFFER_SIZE;
    }
    message[msg_len] = '\0'; // Null-terminate

    // Update tail
    CAN_LogBuffer.tail = (index + 1) % CAN_LOG_BUFFER_SIZE;

    return true;
}

/* Add Message to RX Circular Buffer */
static bool CAN_AddToRxBuffer(uint8_t *data, uint8_t length) {
    if (length == 0 || length > CAN_MAX_MESSAGE_SIZE) {
        CAN_SendError(CAN_ERROR_INVALID_MESSAGE);
        return false;
    }

    uint16_t next_head = (CAN_RxBuffer.head + 1) % CAN_RX_BUFFER_SIZE;
    if (next_head == CAN_RxBuffer.tail) {
        // Buffer is full, overwrite oldest message
        CAN_RxBuffer.tail = (CAN_RxBuffer.tail + 1) % CAN_RX_BUFFER_SIZE;
        CAN_SendError(CAN_ERROR_OVERRUN);
    }

    memcpy(CAN_RxBuffer.data[CAN_RxBuffer.head], data, length);
    CAN_RxBuffer.length[CAN_RxBuffer.head] = length;
    CAN_RxBuffer.head = next_head;

    return true;
}

/* Retrieve Message from RX Circular Buffer */
static bool CAN_GetFromRxBuffer(uint8_t *data, uint8_t *length) {
    if (CAN_RxBuffer.tail == CAN_RxBuffer.head) {
        // Buffer is empty
        return false;
    }

    memcpy(data, CAN_RxBuffer.data[CAN_RxBuffer.tail], CAN_RxBuffer.length[CAN_RxBuffer.tail]);
    *length = CAN_RxBuffer.length[CAN_RxBuffer.tail];
    CAN_RxBuffer.tail = (CAN_RxBuffer.tail + 1) % CAN_RX_BUFFER_SIZE;

    return true;
}

/* Process TX Circular Buffer and Transmit Messages */
void CAN_ProcessLogBuffer(void) {
    char message[CAN_MAX_MESSAGE_SIZE + 1];
    while (CAN_GetFromLogBuffer(message)) {
        CAN_TransmitWithRetry(message);
    }
}

/* Callback Function for CAN RX Interrupt */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    if (hfdcan->Instance != hfdcan1.Instance) {
        return;
    }

    /* Handle New Message */
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) {
        uint8_t data[CAN_MAX_MESSAGE_SIZE];
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, data) != HAL_OK) {
            /* Reception Error */
            CAN_SendError(CAN_ERROR_RECEIVE);
            return;
        }

        /* Get the actual data length in bytes from the DLC code */
        uint8_t data_length = DLC_To_Bytes(RxHeader.DataLength);

        /* Ensure data_length does not exceed CAN_MAX_MESSAGE_SIZE */
        if (data_length > CAN_MAX_MESSAGE_SIZE) {
            data_length = CAN_MAX_MESSAGE_SIZE;
            CAN_SendError(CAN_ERROR_RECEIVE);
        }

        /* Save the received data and length into the receive buffer */
        CAN_AddToRxBuffer(data, data_length);
    }

    /* Handle CAN Bus Errors */
    if (RxFifo0ITs & FDCAN_IT_ERROR_WARNING) {
        CAN_SendError(CAN_ERROR_RECEIVE);
    }

    if (RxFifo0ITs & FDCAN_IT_ERROR_PASSIVE) {
        CAN_SendError(CAN_ERROR_RECEIVE);
    }

    if (RxFifo0ITs & FDCAN_IT_BUS_OFF) {
        CAN_SendError(CAN_ERROR_BUS_OFF);
        /* Attempt to recover from bus-off */
        if (!CAN_RecoverBusOff()) {
            /* If recovery fails, enter a safe state */
            while(1);
        }
    }
}

/* Process Received CAN Messages */
void CAN_ProcessReceivedMessages(void) {
    uint8_t data[CAN_MAX_MESSAGE_SIZE + 1];  // +1 for null-termination
    uint8_t length;

    while (CAN_GetFromRxBuffer(data, &length)) {
        /* Validate received data before processing */
        if (length == 0 || length > CAN_MAX_MESSAGE_SIZE) {
            CAN_SendError(CAN_ERROR_INVALID_MESSAGE);
            continue;
        }

        /* Ensure there is space for null-termination */
        if (length >= CAN_MAX_MESSAGE_SIZE) {
            length = CAN_MAX_MESSAGE_SIZE - 1;
        }

        /* Null-terminate the message for string operations */
        data[length] = '\0';
        char *message = (char *)data;

        /* Validate message format */
        if (!CAN_ValidateMessage(message)) {
            CAN_Log_Now("CMD ERROR");
            CAN_SendError(CAN_ERROR_INVALID_MESSAGE);
            continue;
        }

        /* Parse and apply the message */
        if (!CAN_ParseMessage(message)) {
            CAN_Log_Now("CMD ERROR");
            CAN_SendError(CAN_ERROR_INVALID_MESSAGE);
            continue;
        }

        /* Send Acknowledgment */
        char ackMessage[32];
        snprintf(ackMessage, sizeof(ackMessage), "ACK:%s", message);
        CAN_AddToLogBuffer(ackMessage);
    }
}

/* Validate Message Format */
static bool CAN_ValidateMessage(const char *message) {
    if (message == NULL) {
        return false;
    }

    /* Ensure message contains '=' */
    const char *equal_sign = strchr(message, '=');
    if (equal_sign == NULL) {
        return false;
    }

    /* Ensure all characters before '=' are alphanumeric or underscores */
    for (const char *p = message; p < equal_sign; p++) {
        if (!isalnum(*p) && *p != '_') {
            return false;
        }
    }

    /* Additional validations can be added here (e.g., message length, specific command formats) */

    return true;
}

/* Parse and Apply Message */
static bool CAN_ParseMessage(const char *message) {
    if (message == NULL) {
        return false;
    }

    /* Define maximum expected command length - now shorter! */
    const size_t MAX_COMMAND_LENGTH = 8;  // Reduced from 64
    char command[MAX_COMMAND_LENGTH];
    char value_str[MAX_COMMAND_LENGTH];

    /* Extract command and value using sscanf with error checking */
    if (sscanf(message, "%7[^=]=%7s", command, value_str) != 2) {
        return false;
    }

    /* Process commands using helper functions for robust parsing */
    if (strcmp(command, CMD_SET_MAX_RPM) == 0) {         // SMR
        char *endptr;
        errno = 0;
        long value = strtol(value_str, &endptr, 10);
        if (endptr == value_str || *endptr != '\0' || errno == ERANGE || value < 0 || value > 10000) {
            return false;
        }
        config.maxRPM = (int)value;
    }
    else if (strcmp(command, CMD_SET_ENC_TYPE) == 0) {   // SET
        char *endptr;
        errno = 0;
        long value = strtol(value_str, &endptr, 10);
        if (endptr == value_str || *endptr != '\0' || errno == ERANGE || value < 0 || value >= ENCODER_TYPE_COUNT) {
            return false;
        }
        config.encoderType = (EncoderType_t)value;
    }
    else if (strcmp(command, CMD_SET_ENC_COMM) == 0) {   // SEC
        char *endptr;
        errno = 0;
        long value = strtol(value_str, &endptr, 10);
        if (endptr == value_str || *endptr != '\0' || errno == ERANGE || value < 0 || value >= ENCODER_COMM_COUNT) {
            return false;
        }
        config.encoderComm = (EncoderComm_t)value;
    }
    else if (strcmp(command, CMD_SET_TRG_ANG) == 0) {    // STA
        double value;
        if (!ParseDouble(value_str, &value, 0.0, 360.0)) {
            return false;
        }
        config.targetAngle = value;
    }
    else if (strcmp(command, CMD_SET_SPEED) == 0) {      // SSP
        char *endptr;
        errno = 0;
        long value = strtol(value_str, &endptr, 10);
        if (endptr == value_str || *endptr != '\0' || errno == ERANGE || value < 0 || value > 5000) {
            return false;
        }
        config.maxSpeed = (int)value;
    }
    else if (strcmp(command, CMD_POWER_MODE) == 0) {     // PMD
        char *endptr;
        errno = 0;
        long value = strtol(value_str, &endptr, 10);
        if (endptr == value_str || *endptr != '\0' || errno == ERANGE || value < 0 || value >= POWER_MODE_COUNT) {
            return false;
        }
        config.powerMode = (PowerMode_t)value;
    }
    else if (strcmp(command, CMD_SET_ACC) == 0) {        // SAC
        char *endptr;
        errno = 0;
        long value = strtol(value_str, &endptr, 10);
        if (endptr == value_str || *endptr != '\0' || errno == ERANGE || value < 0 || value > 1000) {
            return false;
        }
        config.maxAcceleration = (int)value;
    }
    else if (strcmp(command, CMD_SET_JERK) == 0) {       // SJK
        char *endptr;
        errno = 0;
        long value = strtol(value_str, &endptr, 10);
        if (endptr == value_str || *endptr != '\0' || errno == ERANGE || value < 0 || value > 1000) {
            return false;
        }
        config.maxJerk = (int)value;
    }
    else if (strcmp(command, CMD_SET_LIM_UP) == 0) {     // SLU
        char *endptr;
        errno = 0;
        long value = strtol(value_str, &endptr, 10);
        if (endptr == value_str || *endptr != '\0' || errno == ERANGE || value < -1000 || value > 1000) {
            return false;
        }
        config.positionLimitUpper = (int)value;
    }
    else if (strcmp(command, CMD_SET_LIM_DN) == 0) {     // SLD
        char *endptr;
        errno = 0;
        long value = strtol(value_str, &endptr, 10);
        if (endptr == value_str || *endptr != '\0' || errno == ERANGE || value < -1000 || value > 1000) {
            return false;
        }
        config.positionLimitLower = (int)value;
    }
    else if (strcmp(command, CMD_SET_ERR_LIM) == 0) {    // SEL
        char *endptr;
        errno = 0;
        long value = strtol(value_str, &endptr, 10);
        if (endptr == value_str || *endptr != '\0' || errno == ERANGE || value < 0 || value > 100) {
            return false;
        }
        config.maxErrorThreshold = (int)value;
    }
    else if (strcmp(command, CMD_SET_FEEDBACK) == 0) {   // SFB
        char *endptr;
        errno = 0;
        long value = strtol(value_str, &endptr, 10);
        if (endptr == value_str || *endptr != '\0' || errno == ERANGE || (value != 0 && value != 1)) {
            return false;
        }
        config.feedbackOutput = (value == 1) ? true : false;
    }
    else if (strcmp(command, CMD_SET_PID_KP) == 0) {     // SKP
        double value;
        if (!ParseDouble(value_str, &value, 0.0, 100.0)) {
            return false;
        }
        config.kp = value;
    }
    else if (strcmp(command, CMD_SET_PID_KI) == 0) {     // SKI
        double value;
        if (!ParseDouble(value_str, &value, 0.0, 100.0)) {
            return false;
        }
        config.ki = value;
    }
    else if (strcmp(command, CMD_SET_PID_KD) == 0) {     // SKD
        double value;
        if (!ParseDouble(value_str, &value, 0.0, 100.0)) {
            return false;
        }
        config.kd = value;
    }
    else if (strcmp(command, CMD_SET_TAU) == 0) {        // STU
        double value;
        if (!ParseDouble(value_str, &value, MIN_TAU, MAX_TAU)) {
            return false;
        }
        config.tau = value;
    }
    else if (strcmp(command, CMD_SET_KAW) == 0) {        // SKW
        double value;
        if (!ParseDouble(value_str, &value, MIN_KAW, MAX_KAW)) {
            return false;
        }
        config.Kaw = value;
    }
    else if (strcmp(command, CMD_SET_DIST_THR) == 0) {   // SDT
        double value;
        if (!ParseDouble(value_str, &value, MIN_DISTURBANCE_THRESHOLD, MAX_DISTURBANCE_THRESHOLD)) {
            return false;
        }
        config.disturbanceThreshold = value;
    }
    else {
        /* Unknown command signaling */
        return false;
    }

    return true;
}

/* Send Error Messages Over CAN */
static void CAN_SendError(CAN_ErrorCode_t errorCode) {
    const char *errorMessages[] = {
        "CAN_ERROR_NONE",
        "CAN_ERROR_FILTER_CONFIG",
        "CAN_ERROR_BITTIMING_CONFIG",
        "CAN_ERROR_START",
        "CAN_ERROR_ACTIVATE_NOTIFICATION",
        "CAN_ERROR_TRANSMIT",
        "CAN_ERROR_RECEIVE",
        "CAN_ERROR_BUS_OFF",
        "CAN_ERROR_OVERRUN",
        "CAN_ERROR_INVALID_MESSAGE",
        "CAN_ERROR_UNKNOWN",
        // Add more messages corresponding to CAN_ErrorCode_t
    };

    if (errorCode >= CAN_ERROR_NONE && errorCode <= CAN_ERROR_UNKNOWN) {
        CAN_Log_Now(errorMessages[errorCode]);
        CAN_ErrorCounters[errorCode]++;
    } else {
        CAN_Log_Now("CAN_ERROR_UNKNOWN");
        CAN_ErrorCounters[CAN_ERROR_UNKNOWN]++;
    }

    // Implement further error handling as needed, such as resetting modules or entering safe states
}

/* Calculate Checksum for a Message */
static uint16_t CalculateChecksum(const char *message) {
    uint16_t checksum = 0;
    while (*message) {
        checksum += (uint8_t)(*message++);
    }
    return checksum;
}

/* Recover from Bus-Off State */
static bool CAN_RecoverBusOff(void) {
    /* Attempt to reset CAN peripheral */
    if (HAL_FDCAN_Stop(&hfdcan1) != HAL_OK) {
        CAN_SendError(CAN_ERROR_BUS_OFF);
        return false;
    }

    /* Re-initialize CAN */
    CAN_Init(TxHeader.Identifier);

    /* Check if recovery was successful */
    // This can be enhanced by checking CAN peripheral status
    return true;
}

/* Add a helper function for command debugging */
const char* CAN_GetLongCommandName(const char* shortCmd) {
    if (strcmp(shortCmd, "SMR") == 0) return "SET_MAX_RPM";
    if (strcmp(shortCmd, "SET") == 0) return "SET_ENCODER_TYPE";
    if (strcmp(shortCmd, "SEC") == 0) return "SET_ENCODER_COMM";
    if (strcmp(shortCmd, "STA") == 0) return "SET_TARGET_ANGLE";
    if (strcmp(shortCmd, "SSP") == 0) return "SET_SPEED";
    if (strcmp(shortCmd, "PMD") == 0) return "POWER_MODE";
    if (strcmp(shortCmd, "SAC") == 0) return "SET_ACC";
    if (strcmp(shortCmd, "SJK") == 0) return "SET_JERK";
    if (strcmp(shortCmd, "SLU") == 0) return "SET_LIMIT_UP";
    if (strcmp(shortCmd, "SLD") == 0) return "SET_LIMIT_DOWN";
    if (strcmp(shortCmd, "SEL") == 0) return "SET_ERROR_LIM";
    if (strcmp(shortCmd, "SFB") == 0) return "SET_FEEDBACK";
    if (strcmp(shortCmd, "SKP") == 0) return "SET_PID_KP";
    if (strcmp(shortCmd, "SKI") == 0) return "SET_PID_KI";
    if (strcmp(shortCmd, "SKD") == 0) return "SET_PID_KD";
    if (strcmp(shortCmd, "STU") == 0) return "SET_TAU";
    if (strcmp(shortCmd, "SKW") == 0) return "SET_KAW";
    if (strcmp(shortCmd, "SDT") == 0) return "SET_DISTURBANCE_THRESHOLD";
    return "UNKNOWN_COMMAND";
}

/* Add Message to TX Buffer (Circular Buffer) */
void CAN_Log(const char *message) {
    if (!CAN_AddToLogBuffer(message)) {
        // Handle buffer full scenario if necessary
    }
}

/* Send High-Priority Message Immediately with Checksum */
void CAN_Log_Now(const char *message) {
    CAN_TransmitWithRetry(message);
}

/* Transmit Messages from TX Buffer */
void CAN_TX(void) {
    CAN_ProcessLogBuffer();
}

/* Retrieve the Next Received CAN Message from Buffer */
static bool CAN_GetReceivedMessage(uint8_t *data, uint8_t *length) {
    return CAN_GetFromRxBuffer(data, length);
}

/* Data Conversion Functions */
/* Note: Since we switched to double, consider adding functions for double if needed */
double CAN_BytesToDouble(uint8_t *data) {
    double value;
    memcpy(&value, data, sizeof(double));
    return value;
}

int16_t CAN_BytesToInt16(uint8_t *data) {
    int16_t value;
    memcpy(&value, data, sizeof(int16_t));
    return value;
}

/* Error Handler */
void Error_Handler(void) {
    /* Implemented as CAN_SendError with specific error codes */
    CAN_SendError(CAN_ERROR_UNKNOWN);
    /* Optionally, enter a safe state or attempt recovery */
    while(1);
}
