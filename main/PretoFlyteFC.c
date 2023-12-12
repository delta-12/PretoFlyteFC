/* Includes
 ******************************************************************************/
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "Lsm9ds1.h"
#include "math.h"
#include "Sbus.h"

/* Defines
 ******************************************************************************/

#define SBUS_UART UART_NUM_2
#define SBUS_RX_PIN GPIO_NUM_32
#define SBUS_TX_PIN GPIO_NUM_33
#define SBUS_ROLL_CHANNEL SBUS_CHANNEL_1
#define SBUS_PITCH_CHANNEL SBUS_CHANNEL_2
#define SBUS_YAW_CHANNEL SBUS_CHANNEL_4
#define SBUS_THROTTLE_CHANNEL SBUS_CHANNEL_3
#define SBUS_AUX_1_CHANNEL SBUS_CHANNEL_5
#define SBUS_AUX_2_CHANNEL SBUS_CHANNEL_6
#define SBUS_CHANNEL_MIN 172U
#define SBUS_CHANNEL_MAX 1811U

#define SBUS_CHANNEL_VALUE(percentage) (uint16_t)((double)percentage * (double)(SBUS_CHANNEL_MAX - SBUS_CHANNEL_MIN) + (double)SBUS_CHANNEL_MIN)

#define US_PER_MS 1000ULL

/* Globals
 ******************************************************************************/

static const char *PretoFlyteFC_LogTag = "PretoFlyteFC";

static Lsm9ds1_Handle_t PretoFlyteFC_Lsm9ds1Handle;

static uint8_t PretoFlyteFC_SbusRxBuffer[SBUS_FRAME_SIZE] = {0U};
static uint8_t PretoFlyteFC_SbusTxBuffer[SBUS_FRAME_SIZE] = {0U};
static Sbus_Handle_t PretoFlyteFC_SbusHandle;

static uint16_t PretoFlyteFC_SbusChannelBuffer[SBUS_CHANNELS] = {0U};
static Sbus_Payload_t PretoFlyteFC_SbusPayload;

/* Function Prototypes
 ******************************************************************************/

static void PretoFlyteFC_LogAccelGyroTask(void *arg);
static uint32_t PretoFlyteFC_Millis(void);

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
    Lsm9ds1_Init(&PretoFlyteFC_Lsm9ds1Handle, SPI3_HOST, &lsm9ds1PinConfig);
    Lsm9ds1_GyroInit(&PretoFlyteFC_Lsm9ds1Handle);
    Lsm9ds1_AccelInit(&PretoFlyteFC_Lsm9ds1Handle);
    Lsm9ds1_MagInit(&PretoFlyteFC_Lsm9ds1Handle);
    // Lsm9ds1_AccelGyroCalibrate(&PretoFlyteFC_Lsm9ds1Handle);
    // xTaskCreate(PretoFlyteFC_LogAccelGyroTask, "PretoFlyteFC_LogAccelGyroTask", 2048U, NULL, 10U, NULL);

    Sbus_InitHandle(&PretoFlyteFC_SbusHandle, SBUS_UART, SBUS_RX_PIN, SBUS_TX_PIN, false, &PretoFlyteFC_SbusRxBuffer, &PretoFlyteFC_SbusTxBuffer);
    Sbus_InitPayload(&PretoFlyteFC_SbusPayload, &PretoFlyteFC_SbusChannelBuffer);

    /* Set initial channel values */
    Sbus_SetChannel(&PretoFlyteFC_SbusPayload, SBUS_ROLL_CHANNEL, SBUS_CHANNEL_VALUE(0.50));
    Sbus_SetChannel(&PretoFlyteFC_SbusPayload, SBUS_PITCH_CHANNEL, SBUS_CHANNEL_VALUE(0.50));
    Sbus_SetChannel(&PretoFlyteFC_SbusPayload, SBUS_YAW_CHANNEL, SBUS_CHANNEL_VALUE(0.50));
    Sbus_SetChannel(&PretoFlyteFC_SbusPayload, SBUS_THROTTLE_CHANNEL, SBUS_CHANNEL_VALUE(0.00));
    Sbus_SetChannel(&PretoFlyteFC_SbusPayload, SBUS_AUX_1_CHANNEL, SBUS_CHANNEL_VALUE(0.00));
    Sbus_SetChannel(&PretoFlyteFC_SbusPayload, SBUS_AUX_2_CHANNEL, SBUS_CHANNEL_VALUE(1.00));

    uint32_t endTime = PretoFlyteFC_Millis() + 10000U;
    while (PretoFlyteFC_Millis() < endTime)
    {
        Sbus_Tx(&PretoFlyteFC_SbusHandle, &PretoFlyteFC_SbusPayload);
        Sbus_Rx(&PretoFlyteFC_SbusHandle, &PretoFlyteFC_SbusPayload);

        vTaskDelay(10U / portTICK_PERIOD_MS);
    }

    /* Arm */
    Sbus_SetChannel(&PretoFlyteFC_SbusPayload, SBUS_AUX_1_CHANNEL, SBUS_CHANNEL_VALUE(0.75));
    Sbus_Tx(&PretoFlyteFC_SbusHandle, &PretoFlyteFC_SbusPayload);

    /* Increase throttle 10% fevery 2s until it reaches 50% */
    endTime = PretoFlyteFC_Millis() + 2000U;
    double throttle = 0.00;
    while (throttle < 0.50)
    {
        if (PretoFlyteFC_Millis() > endTime)
        {
            throttle += 0.10;
            endTime = PretoFlyteFC_Millis() + 2000U;
        }

        Sbus_SetChannel(&PretoFlyteFC_SbusPayload, SBUS_THROTTLE_CHANNEL, SBUS_CHANNEL_VALUE(throttle));
        Sbus_Tx(&PretoFlyteFC_SbusHandle, &PretoFlyteFC_SbusPayload);
        Sbus_Rx(&PretoFlyteFC_SbusHandle, &PretoFlyteFC_SbusPayload);

        vTaskDelay(10U / portTICK_PERIOD_MS);
    }

    /* Hold throttle at 50%  */
    while (PretoFlyteFC_Millis() < endTime)
    {
        Sbus_Tx(&PretoFlyteFC_SbusHandle, &PretoFlyteFC_SbusPayload);
        Sbus_Rx(&PretoFlyteFC_SbusHandle, &PretoFlyteFC_SbusPayload);

        vTaskDelay(10U / portTICK_PERIOD_MS);
    }

    /* Hold throttle at 0% */
    throttle = 0.00;
    Sbus_SetChannel(&PretoFlyteFC_SbusPayload, SBUS_THROTTLE_CHANNEL, SBUS_CHANNEL_VALUE(throttle));
    while (1U)
    {
        Sbus_Tx(&PretoFlyteFC_SbusHandle, &PretoFlyteFC_SbusPayload);
        Sbus_Rx(&PretoFlyteFC_SbusHandle, &PretoFlyteFC_SbusPayload);

        vTaskDelay(10U / portTICK_PERIOD_MS);
    }
}

static void PretoFlyteFC_LogAccelGyroTask(void *arg)
{
    double pitch;
    double roll;

    for (;;)
    {
        if (Lsm9ds1_AccelAvailable(&PretoFlyteFC_Lsm9ds1Handle))
        {
            Lsm9ds1_AccelRead(&PretoFlyteFC_Lsm9ds1Handle);
        }
        if (Lsm9ds1_GyroAvailable(&PretoFlyteFC_Lsm9ds1Handle))
        {
            Lsm9ds1_GyroRead(&PretoFlyteFC_Lsm9ds1Handle);
        }

        /* Pitch = atan(-ax, sqrt(ay^2, az^2)) */
        pitch = atan2(PretoFlyteFC_Lsm9ds1Handle.Accel.Reading.X * -1.0,
                      sqrt(PretoFlyteFC_Lsm9ds1Handle.Accel.Reading.Y * PretoFlyteFC_Lsm9ds1Handle.Accel.Reading.Y +
                           PretoFlyteFC_Lsm9ds1Handle.Accel.Reading.Z * PretoFlyteFC_Lsm9ds1Handle.Accel.Reading.Z));

        /* Roll = atan2(ay, az) */
        roll = atan2(PretoFlyteFC_Lsm9ds1Handle.Accel.Reading.Y, PretoFlyteFC_Lsm9ds1Handle.Accel.Reading.Z);

        ESP_LOGI(PretoFlyteFC_LogTag, "Ax: %.02lf, Ay: %.02lf, Az: %.02lf",
                 PretoFlyteFC_Lsm9ds1Handle.Accel.Reading.X,
                 PretoFlyteFC_Lsm9ds1Handle.Accel.Reading.Y,
                 PretoFlyteFC_Lsm9ds1Handle.Accel.Reading.Z);
        ESP_LOGI(PretoFlyteFC_LogTag, "Gx: %.02lf, Gy: %.02lf, Gz: %.02lf",
                 PretoFlyteFC_Lsm9ds1Handle.Gyro.Reading.X,
                 PretoFlyteFC_Lsm9ds1Handle.Gyro.Reading.Y,
                 PretoFlyteFC_Lsm9ds1Handle.Gyro.Reading.Z);

        /* Convert from radians to degrees */
        pitch *= 180.0 / M_PI;
        roll *= 180.0 / M_PI;
        ESP_LOGI(PretoFlyteFC_LogTag, "Pitch: %.02lf deg, Roll: %.02lf deg", pitch, roll);

        vTaskDelay(250U / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

static uint32_t PretoFlyteFC_Millis(void)
{
    return (uint32_t)(esp_timer_get_time() / US_PER_MS);
}
