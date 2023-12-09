/* Includes
 ******************************************************************************/
#include "driver/gpio.h"
#include "Lsm9ds1.h"
#include "Sbus.h"

/* Defines
 ******************************************************************************/

#define SBUS_UART UART_NUM_2
#define SBUS_RX_PIN GPIO_NUM_23
#define SBUS_TX_PIN GPIO_NUM_19
#define SBUS_ROLL_CHANNEL SBUS_CHANNEL_1
#define SBUS_PITCH_CHANNEL SBUS_CHANNEL_2
#define SBUS_YAW_CHANNEL SBUS_CHANNEL_3
#define SBUS_THROTTLE_CHANNEL SBUS_CHANNEL_4
#define SBUS_CHANNEL_MIN 172U
#define SBUS_CHANNEL_MAX 1811U

#define SBUS_CHANNEL_VALUE(percentage) (uint16_t)((double)percentage * (double)(SBUS_CHANNEL_MAX - SBUS_CHANNEL_MIN) + (double)SBUS_CHANNEL_MIN)

/* Globals
 ******************************************************************************/

static Lsm9ds1_Handle_t Lsm9ds1Handle;

static uint8_t SbusRxBuffer[SBUS_FRAME_SIZE] = {0U};
static uint8_t SbusTxBuffer[SBUS_FRAME_SIZE] = {0U};
static Sbus_Handle_t SbusHandle;

static uint16_t SbusChannelBuffer[SBUS_CHANNELS] = {0U};
static Sbus_Payload_t SbusPayload;

/* Function Definitions
 ******************************************************************************/

void app_main(void)
{
    Lsm9ds1_PinConfig_t lsm9ds1PinConfig = {
        .Mosi = GPIO_NUM_23,
        .Miso = GPIO_NUM_19,
        .Clk = GPIO_NUM_18,
        .CsAccelGyro = GPIO_NUM_5,
        .CsMag = GPIO_NUM_22,
        .Int1 = GPIO_NUM_26,
        .Int2 = GPIO_NUM_25,
        .IntM = GPIO_NUM_27,
        .Rdy = GPIO_NUM_15,
    };
    Lsm9ds1_Init(&Lsm9ds1Handle, SPI3_HOST, &lsm9ds1PinConfig);

    Sbus_InitHandle(&SbusHandle, SBUS_UART, SBUS_RX_PIN, SBUS_TX_PIN, false, &SbusRxBuffer, &SbusTxBuffer);
    Sbus_InitPayload(&SbusPayload, &SbusChannelBuffer);

    while (1U)
    {
        Sbus_SetChannel(&SbusPayload, SBUS_ROLL_CHANNEL, SBUS_CHANNEL_VALUE(0.75));
        Sbus_SetChannel(&SbusPayload, SBUS_PITCH_CHANNEL, SBUS_CHANNEL_VALUE(0.75));
        Sbus_SetChannel(&SbusPayload, SBUS_YAW_CHANNEL, SBUS_CHANNEL_VALUE(0.75));
        Sbus_SetChannel(&SbusPayload, SBUS_THROTTLE_CHANNEL, SBUS_CHANNEL_VALUE(0.75));
        Sbus_Tx(&SbusHandle, &SbusPayload);
        Sbus_Rx(&SbusHandle, &SbusPayload);

        vTaskDelay(10U / portTICK_PERIOD_MS);
    }
}