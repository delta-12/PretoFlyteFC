/* Includes
 ******************************************************************************/
#include "driver/gpio.h"
#include "Sbus.h"

/* Defines
 ******************************************************************************/

#define SBUS_UART UART_NUM_2
#define SBUS_RX_PIN GPIO_NUM_23
#define SBUS_TX_PIN GPIO_NUM_19

/* Globals
 ******************************************************************************/

static uint8_t SbusRxBuffer[SBUS_FRAME_SIZE] = {0U};
static uint8_t SbusTxBuffer[SBUS_FRAME_SIZE] = {0U};
static Sbus_Handle_t SbusHandle;

static uint16_t SbusChannelBuffer[SBUS_CHANNELS] = {0U};
static Sbus_Payload_t SbusPayload;

/* Function Definitions
 ******************************************************************************/

void app_main(void)
{
    Sbus_InitHandle(&SbusHandle, SBUS_UART, SBUS_RX_PIN, SBUS_TX_PIN, &SbusRxBuffer, &SbusTxBuffer);
    Sbus_InitPayload(&SbusPayload, &SbusChannelBuffer);

    while (1U)
    {
        Sbus_SetChannel(&SbusPayload, SBUS_CHANNEL_1, 1700U);
        Sbus_Tx(&SbusHandle, &SbusPayload);
        Sbus_Rx(&SbusHandle, &SbusPayload);

        vTaskDelay(10U / portTICK_PERIOD_MS);
    }
}
