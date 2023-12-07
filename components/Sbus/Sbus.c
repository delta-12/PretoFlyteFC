/**
 * @file Sbus.h
 *
 * @brief Implementation of SBUS protocol for the ESP32.
 *
 ******************************************************************************/

/* Includes
 ******************************************************************************/
#include "Sbus.h"

/* Defines
 ******************************************************************************/

#define SBUS_BAUD_RATE 100000U        /* SBUS protocol has baud rate of 100000 symbols/s */
#define SBUS_UART_BUFFER_SIZE 2500U   /* UART ring buffers can hold 100 SBUS packets with a length of 25 bytes */
#define SBUS_ESP_INTR_FLAG_DEFAULT 0U /* Uart interrupts allocated with the default flags */
#define SBUS_UART_QUEUE_SIZE 20U      /* Depth of 20 items for UART queues */
#define SBUS_TASK_STACK_DEPTH 2048U   /* Stack depth of 2048 bytes for RTOS tasks */
#define SBUS_TASK_PRIORITY 10U        /* Priority of 10 for RTOS tasks */

#define SBUS_HEADER 0x0FU                          /* SBUS frame header */
#define SBUS_END_BYTE 0x00U                        /* SBUS frame end byte */
#define SBUS_DIGITAL_CHANNEL_17_MASK 0x80U         /* Bit 7 of byte 23 of SBUS frame */
#define SBUS_DIGITAL_CHANNEL_18_MASK 0x40U         /* Bit 6 of byte 23 of SBUS frame */
#define SBUS_DIGITAL_FRAME_LOST_MASK 0x20U         /* Bit 5 of byte 23 of SBUS frame */
#define SBUS_DIGITAL_FAILSAFE_ACTIVATED_MASK 0x10U /* Bit 4 of byte 23 of SBUS frame */

/* Function Prototypes
 ******************************************************************************/

static void Sbus_UartEventHandlerTask(void *arg);

/* Function Definitions
 ******************************************************************************/

void Sbus_InitHandle(Sbus_Handle_t *const handle, uart_port_t uartPort, const Sbus_UartPin_t rxPin, const Sbus_UartPin_t txPin, uint8_t (*const rxBuffer)[SBUS_FRAME_SIZE], uint8_t (*const txBuffer)[SBUS_FRAME_SIZE])
{
    if (handle != NULL && rxBuffer != NULL && txBuffer != NULL)
    {
        handle->UartConfig.baud_rate = SBUS_BAUD_RATE;
        handle->UartConfig.data_bits = UART_DATA_8_BITS;
        handle->UartConfig.parity = UART_PARITY_EVEN;
        handle->UartConfig.stop_bits = UART_STOP_BITS_2;
        handle->UartConfig.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
        handle->UartConfig.rx_flow_ctrl_thresh = 0U;
        handle->UartConfig.source_clk = UART_SCLK_DEFAULT;

        handle->UartPort = uartPort;

        handle->RxPin = rxPin;
        handle->TxPin = txPin;

        handle->TaskHandle = NULL;

        handle->RxBuffer = rxBuffer;
        handle->RxState = 0U;
        handle->TxBuffer = txBuffer;

        /* TODO invert signal? */

        ESP_ERROR_CHECK(uart_driver_install(handle->UartPort, SBUS_UART_BUFFER_SIZE, SBUS_UART_BUFFER_SIZE, SBUS_UART_QUEUE_SIZE, &handle->UartQueueHandle, SBUS_ESP_INTR_FLAG_DEFAULT));
        ESP_ERROR_CHECK(uart_param_config(handle->UartPort, &handle->UartConfig));
        ESP_ERROR_CHECK(uart_set_pin(handle->UartPort, handle->TxPin, handle->RxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

        xTaskCreate(Sbus_UartEventHandlerTask, "Sbus_UartEventHandlerTask", SBUS_TASK_STACK_DEPTH, handle, SBUS_TASK_PRIORITY, &handle->TaskHandle);
    }
}

void Sbus_DeInitHandle(Sbus_Handle_t *const handle)
{
    if (handle != NULL)
    {
        uart_driver_delete(handle->UartPort);

        if (handle->TaskHandle != NULL)
        {
            vTaskDelete(handle->TaskHandle);
        }
    }
}

void Sbus_InitPayload(Sbus_Payload_t *const payload, uint16_t (*const channelBuffer)[SBUS_CHANNELS])
{
    if (payload != NULL && channelBuffer != NULL)
    {
        payload->Channels = channelBuffer;
        payload->FrameLost = false;
        payload->FailsafeActivated = false;
    }
}

bool Sbus_Rx(Sbus_Handle_t *const handle, Sbus_Payload_t *const payload)
{
    if (handle != NULL && payload != NULL)
    {
        size_t size;

        uart_get_buffered_data_len(handle->UartPort, &size);

        size_t remainingBytes = SBUS_FRAME_SIZE - handle->RxState;
        if (size > remainingBytes)
        {
            handle->RxState += uart_read_bytes(handle->UartPort, &((*handle->RxBuffer)[handle->RxState]), remainingBytes, portMAX_DELAY);
        }
        else
        {
            handle->RxState += uart_read_bytes(handle->UartPort, &((*handle->RxBuffer)[handle->RxState]), size, portMAX_DELAY);
        }
        handle->RxState %= SBUS_FRAME_SIZE;
    }

    return false;
}

void Sbus_Tx(Sbus_Handle_t *const handle, const Sbus_Payload_t *const payload)
{
    if (handle != NULL && payload != NULL)
    {
        // (*payload->Channels)[SBUS_CHANNEL_1];

        (*handle->TxBuffer)[SBUS_FRAME_BYTE_0] = SBUS_HEADER;

        (*handle->TxBuffer)[SBUS_FRAME_BYTE_1] = (*payload->Channels)[SBUS_CHANNEL_1] & 0x07FF;
        (*handle->TxBuffer)[SBUS_FRAME_BYTE_2] = ((*payload->Channels)[SBUS_CHANNEL_1] & 0x07FF) >> 8 | ((*payload->Channels)[SBUS_CHANNEL_2] & 0x07FF) << 3;

        (*handle->TxBuffer)[SBUS_FRAME_BYTE_23] = 0U;
        if ((*payload->Channels)[SBUS_CHANNEL_17] > 0U)
        {
            (*handle->TxBuffer)[SBUS_FRAME_BYTE_23] |= SBUS_DIGITAL_CHANNEL_17_MASK;
        }
        if ((*payload->Channels)[SBUS_CHANNEL_18] > 0U)
        {
            (*handle->TxBuffer)[SBUS_FRAME_BYTE_23] |= SBUS_DIGITAL_CHANNEL_17_MASK;
        }
        if (payload->FrameLost)
        {
            (*handle->TxBuffer)[SBUS_FRAME_BYTE_23] |= SBUS_DIGITAL_FRAME_LOST_MASK;
        }
        if (payload->FailsafeActivated)
        {
            (*handle->TxBuffer)[SBUS_FRAME_BYTE_23] |= SBUS_DIGITAL_FAILSAFE_ACTIVATED_MASK;
        }

        (*handle->TxBuffer)[SBUS_FRAME_BYTE_24] = SBUS_END_BYTE;
    }
}

/* TODO inline */
uint16_t Sbus_GetChannel(const Sbus_Payload_t *const payload, const Sbus_Channel_t channel)
{
    return (*payload->Channels)[channel];
}

/* TODO inline */
void Sbus_SetChannel(Sbus_Payload_t *const payload, const Sbus_Channel_t channel, const uint16_t value)
{
    (*payload->Channels)[channel] = value;
}

static void Sbus_UartEventHandlerTask(void *arg)
{
    if (arg != NULL)
    {
        Sbus_Handle_t *handle = (Sbus_Handle_t *)arg;
        uart_event_t event;

        for (;;)
        {
            if (xQueueReceive(handle->UartQueueHandle, &event, portMAX_DELAY))
            {
                switch (event.type)
                {
                case UART_FIFO_OVF:
                    uart_flush_input(handle->UartPort);
                    xQueueReset(handle->UartQueueHandle);
                    break;
                case UART_BUFFER_FULL:
                    uart_flush_input(handle->UartPort);
                    xQueueReset(handle->UartQueueHandle);
                    break;
                default:
                    break;
                }
            }
        }
    }

    vTaskDelete(NULL);
}
