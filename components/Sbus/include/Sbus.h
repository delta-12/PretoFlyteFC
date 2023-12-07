/**
 * @file Sbus.h
 *
 * @brief Implementation of SBUS protocol for the ESP32.
 *
 ******************************************************************************/

#ifndef SBUS_H
#define SBUS_H

/* Includes
 ******************************************************************************/
#include "driver/uart.h"

/* Defines
 ******************************************************************************/

#define SBUS_SERVO_CHANNELS 16U                                     /* 16 11-bit servo channels */
#define SBUS_DIGITAL_CHANNELS 2U                                    /* 2 digital channels */
#define SBUS_CHANNELS (SBUS_SERVO_CHANNELS + SBUS_DIGITAL_CHANNELS) /* Total number of SBUS servo and digital channels*/
#define SBUS_FRAME_SIZE 25U                                         /* SBUS frame is 25 bytes */

/* Typedefs
 ******************************************************************************/

typedef uint32_t Sbus_UartPin_t; /* GPIO pin for UART RX or TX */

/* Index of byte in SBUS frame buffer */
typedef enum
{
    SBUS_FRAME_BYTE_0,  /* Index of byte 0 in SBUS frame buffer */
    SBUS_FRAME_BYTE_1,  /* Index of byte 1 in SBUS frame buffer */
    SBUS_FRAME_BYTE_2,  /* Index of byte 2 in SBUS frame buffer */
    SBUS_FRAME_BYTE_3,  /* Index of byte 3 in SBUS frame buffer */
    SBUS_FRAME_BYTE_4,  /* Index of byte 4 in SBUS frame buffer */
    SBUS_FRAME_BYTE_5,  /* Index of byte 5 in SBUS frame buffer */
    SBUS_FRAME_BYTE_6,  /* Index of byte 6 in SBUS frame buffer */
    SBUS_FRAME_BYTE_7,  /* Index of byte 7 in SBUS frame buffer */
    SBUS_FRAME_BYTE_8,  /* Index of byte 8 in SBUS frame buffer */
    SBUS_FRAME_BYTE_9,  /* Index of byte 9 in SBUS frame buffer */
    SBUS_FRAME_BYTE_10, /* Index of byte 10 in SBUS frame buffer */
    SBUS_FRAME_BYTE_11, /* Index of byte 11 in SBUS frame buffer */
    SBUS_FRAME_BYTE_12, /* Index of byte 12 in SBUS frame buffer */
    SBUS_FRAME_BYTE_13, /* Index of byte 13 in SBUS frame buffer */
    SBUS_FRAME_BYTE_14, /* Index of byte 14 in SBUS frame buffer */
    SBUS_FRAME_BYTE_15, /* Index of byte 15 in SBUS frame buffer */
    SBUS_FRAME_BYTE_16, /* Index of byte 16 in SBUS frame buffer */
    SBUS_FRAME_BYTE_17, /* Index of byte 17 in SBUS frame buffer */
    SBUS_FRAME_BYTE_18, /* Index of byte 18 in SBUS frame buffer */
    SBUS_FRAME_BYTE_19, /* Index of byte 19 in SBUS frame buffer */
    SBUS_FRAME_BYTE_20, /* Index of byte 20 in SBUS frame buffer */
    SBUS_FRAME_BYTE_21, /* Index of byte 21 in SBUS frame buffer */
    SBUS_FRAME_BYTE_22, /* Index of byte 22 in SBUS frame buffer */
    SBUS_FRAME_BYTE_23, /* Index of byte 23 in SBUS frame buffer */
    SBUS_FRAME_BYTE_24  /* Index of byte 24 in SBUS frame buffer */
} Sbus_FrameByte_t;

/* Index of channels in payload channel buffer */
typedef enum
{
    SBUS_CHANNEL_1,  /* Index of channel 1 in payload channel buffer */
    SBUS_CHANNEL_2,  /* Index of channel 2 in payload channel buffer */
    SBUS_CHANNEL_3,  /* Index of channel 3 in payload channel buffer */
    SBUS_CHANNEL_4,  /* Index of channel 4 in payload channel buffer */
    SBUS_CHANNEL_5,  /* Index of channel 5 in payload channel buffer */
    SBUS_CHANNEL_6,  /* Index of channel 6 in payload channel buffer */
    SBUS_CHANNEL_7,  /* Index of channel 7 in payload channel buffer */
    SBUS_CHANNEL_8,  /* Index of channel 8 in payload channel buffer */
    SBUS_CHANNEL_9,  /* Index of channel 9 in payload channel buffer */
    SBUS_CHANNEL_10, /* Index of channel 10 in payload channel buffer */
    SBUS_CHANNEL_11, /* Index of channel 11 in payload channel buffer */
    SBUS_CHANNEL_12, /* Index of channel 12 in payload channel buffer */
    SBUS_CHANNEL_13, /* Index of channel 13 in payload channel buffer */
    SBUS_CHANNEL_14, /* Index of channel 14 in payload channel buffer */
    SBUS_CHANNEL_15, /* Index of channel 15 in payload channel buffer */
    SBUS_CHANNEL_16, /* Index of channel 16 in payload channel buffer */
    SBUS_CHANNEL_17, /* Index of channel 17 in payload channel buffer */
    SBUS_CHANNEL_18  /* Index of channel 18 in payload channel buffer */
} Sbus_Channel_t;

/* SBUS frame data */
typedef struct
{
    uint16_t (*Channels)[SBUS_CHANNELS]; /* SBUS servo and digital channel values */
    bool FrameLost;                      /* SBUS frame lost flag */
    bool FailsafeActivated;              /* SBUS failsafe activated flag */
} Sbus_Payload_t;

/* Handle for SBUS communication */
typedef struct
{
    uart_config_t UartConfig;             /* UART config for SBUS */
    uart_port_t UartPort;                 /* UART port number for SBUS UART */
    QueueHandle_t UartQueueHandle;        /* Handle for SBUS UART event queue */
    Sbus_UartPin_t RxPin;                 /* SBUS UART RX pin */
    Sbus_UartPin_t TxPin;                 /* SBUS UART TX pin */
    TaskHandle_t TaskHandle;              /* Task handle for SBUS UART event handler task */
    uint8_t (*RxBuffer)[SBUS_FRAME_SIZE]; /* Buffer for deserializing received SBUS frames */
    size_t RxState;                       /* Current byte deserialized in received SBUS frame */
    uint8_t (*TxBuffer)[SBUS_FRAME_SIZE]; /* Buffer for serializing transmitted SBUS frames */
} Sbus_Handle_t;

/* Function Prototypes
 ******************************************************************************/

void Sbus_InitHandle(Sbus_Handle_t *const handle, uart_port_t uartPort, const Sbus_UartPin_t rxPin, const Sbus_UartPin_t txPin, uint8_t (*const rxBuffer)[SBUS_FRAME_SIZE], uint8_t (*const txBuffer)[SBUS_FRAME_SIZE]);
void Sbus_DeInitHandle(Sbus_Handle_t *const handle);
void Sbus_InitPayload(Sbus_Payload_t *const payload, uint16_t (*const channelBuffer)[SBUS_CHANNELS]);
bool Sbus_Rx(Sbus_Handle_t *const handle, Sbus_Payload_t *const payload);
void Sbus_Tx(Sbus_Handle_t *const handle, const Sbus_Payload_t *const payload);
uint16_t Sbus_GetChannel(const Sbus_Payload_t *const payload, const Sbus_Channel_t channel);
void Sbus_SetChannel(Sbus_Payload_t *const payload, const Sbus_Channel_t channel, const uint16_t value);

#endif