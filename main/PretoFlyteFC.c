/* Includes
 ******************************************************************************/
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "Kalman.h"
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

#define ANGLE_MIN (-50.0)
#define ANGLE_MAX (50.0)

#define US_PER_MS 1000ULL
#define US_TO_S 0.000001

#define RX_ROLL_PIN GPIO_NUM_25
#define RX_PITCH_PIN GPIO_NUM_26
#define RX_YAW_PIN GPIO_NUM_15
#define RX_THROTTLE_PIN GPIO_NUM_14
#define RX_ARM_PIN GPIO_NUM_27
#define GPIO_BIT 1ULL
#define RECEIVER_GPIO_PIN_SEL ((GPIO_BIT << RX_ROLL_PIN) | (GPIO_BIT << RX_PITCH_PIN) | (GPIO_BIT << RX_YAW_PIN) | (GPIO_BIT << RX_THROTTLE_PIN) | (GPIO_BIT << RX_ARM_PIN))
#define RECEIVER_EVENT_QUEUE_SIZE 36U
#define RECEIVER_FREQUENCY 50.0
#define RECEIVER_ROLL_MIN 0.065500
#define RECEIVER_ROLL_MAX 0.084850
#define RECEIVER_PITCH_MIN 0.066000
#define RECEIVER_PITCH_MAX 0.082900
#define RECEIVER_YAW_MIN 0.065100
#define RECEIVER_YAW_MAX 0.084200
#define RECEIVER_THROTTLE_MIN 0.049550
#define RECEIVER_THROTTLE_MAX 0.100350
#define RECEIVER_ARM_MIN 0.049550
#define RECEIVER_ARM_MAX 0.072250

/* Typedef
 ******************************************************************************/

typedef struct
{
    double Kp;
    double Ki;
    double Kd;
    double Integral;
    double PrevError;
    double Output;
} PretoFlyteFC_Pid_t;

typedef struct
{
    uint32_t GpioNum;
    int64_t Time;
    uint8_t Level;
} PretoFlyteFC_ReceiverEvent_t;

/* Globals
 ******************************************************************************/

static const char *PretoFlyteFC_LogTag = "PretoFlyteFC";

static Lsm9ds1_Handle_t PretoFlyteFC_Lsm9ds1Handle;

static uint8_t PretoFlyteFC_SbusRxBuffer[SBUS_FRAME_SIZE] = {0U};
static uint8_t PretoFlyteFC_SbusTxBuffer[SBUS_FRAME_SIZE] = {0U};
static Sbus_Handle_t PretoFlyteFC_SbusHandle;

static uint16_t PretoFlyteFC_SbusChannelBuffer[SBUS_CHANNELS] = {0U};
static Sbus_Payload_t PretoFlyteFC_SbusPayload;

static Kalman_1dFilterContext_t PretoFlyteFC_PitchFilter;
static Kalman_1dFilterContext_t PretoFlyteFC_RollFilter;

static StaticQueue_t ReceiverEventQueue;
static uint8_t ReceiverEventQueueBuffer[RECEIVER_EVENT_QUEUE_SIZE * sizeof(PretoFlyteFC_ReceiverEvent_t)];
static QueueHandle_t ReceiverEventQueueHandle = NULL;
static int64_t ReceiverPitchPrevTime = 0U;
static int64_t ReceiverRollPrevTime = 0U;
static int64_t ReceiverYawPrevTime = 0U;
static int64_t ReceiverThrottlePrevTime = 0U;
static int64_t ReceiverArmPrevTime = 0U;
static double ReceiverRollPercentage = 0.00;
static double ReceiverPitchPercentage = 0.00;
static double ReceiverYawPercentage = 0.00;
static double ReceiverThrottlePercentage = 0.00;
static double ReceiverArmPercentage = 0.00;

/* Function Prototypes
 ******************************************************************************/

static void PretoFlyteFC_Pid(PretoFlyteFC_Pid_t *const pid, const double setpoint, const double process, const double sampleTime);
static void PretoFlyteFC_ReceiverIsrHandler(void *arg);
static void PretoFlyteFC_ReadReceiverTask(void *arg);
static inline double PretoFlyteFC_ReceiverPercentage(const int64_t time, const int64_t prevTime, const double min, const double max);
static inline void PretoFlyteFC_Constrain(double *const value, const double min, const double max);
static inline int64_t PretoFlyteFC_Micros(void);
static inline int64_t PretoFlyteFC_Millis(void);

/* Function Definitions
 ******************************************************************************/

void app_main(void)
{
    /* Configure GPIO */
    gpio_config_t receiverGpioConfig = {
        .pin_bit_mask = RECEIVER_GPIO_PIN_SEL,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    gpio_config(&receiverGpioConfig);
    ReceiverEventQueueHandle = xQueueCreateStatic(RECEIVER_EVENT_QUEUE_SIZE, sizeof(PretoFlyteFC_ReceiverEvent_t), ReceiverEventQueueBuffer, &ReceiverEventQueue);
    xTaskCreate(PretoFlyteFC_ReadReceiverTask, "PretoFlyteFC_ReadReceiverTask", 2048U, NULL, 10U, NULL);
    gpio_install_isr_service(0U);
    gpio_isr_handler_add(RX_ROLL_PIN, PretoFlyteFC_ReceiverIsrHandler, (void *)RX_ROLL_PIN);
    gpio_isr_handler_add(RX_PITCH_PIN, PretoFlyteFC_ReceiverIsrHandler, (void *)RX_PITCH_PIN);
    gpio_isr_handler_add(RX_YAW_PIN, PretoFlyteFC_ReceiverIsrHandler, (void *)RX_YAW_PIN);
    gpio_isr_handler_add(RX_THROTTLE_PIN, PretoFlyteFC_ReceiverIsrHandler, (void *)RX_THROTTLE_PIN);
    gpio_isr_handler_add(RX_ARM_PIN, PretoFlyteFC_ReceiverIsrHandler, (void *)RX_ARM_PIN);

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
    if (!Lsm9ds1_Init(&PretoFlyteFC_Lsm9ds1Handle, SPI3_HOST, &lsm9ds1PinConfig))
    {
        /* Dead loop if IMU failed to initialize */
        while (1u)
        {
            vTaskDelay(10U / portTICK_PERIOD_MS);
        }
    }
    Lsm9ds1_AccelInit(&PretoFlyteFC_Lsm9ds1Handle);
    Lsm9ds1_GyroInit(&PretoFlyteFC_Lsm9ds1Handle);
    Lsm9ds1_AccelGyroCalibrate(&PretoFlyteFC_Lsm9ds1Handle);

    Sbus_InitHandle(&PretoFlyteFC_SbusHandle, SBUS_UART, SBUS_RX_PIN, SBUS_TX_PIN, false, &PretoFlyteFC_SbusRxBuffer, &PretoFlyteFC_SbusTxBuffer);
    Sbus_InitPayload(&PretoFlyteFC_SbusPayload, &PretoFlyteFC_SbusChannelBuffer);

    /* Set initial channel values */
    Sbus_SetChannel(&PretoFlyteFC_SbusPayload, SBUS_ROLL_CHANNEL, SBUS_CHANNEL_VALUE(0.50));
    Sbus_SetChannel(&PretoFlyteFC_SbusPayload, SBUS_PITCH_CHANNEL, SBUS_CHANNEL_VALUE(0.50));
    Sbus_SetChannel(&PretoFlyteFC_SbusPayload, SBUS_YAW_CHANNEL, SBUS_CHANNEL_VALUE(0.50));
    Sbus_SetChannel(&PretoFlyteFC_SbusPayload, SBUS_THROTTLE_CHANNEL, SBUS_CHANNEL_VALUE(0.00));
    Sbus_SetChannel(&PretoFlyteFC_SbusPayload, SBUS_AUX_1_CHANNEL, SBUS_CHANNEL_VALUE(0.00));
    Sbus_SetChannel(&PretoFlyteFC_SbusPayload, SBUS_AUX_2_CHANNEL, SBUS_CHANNEL_VALUE(1.00));

    double pitch;
    double roll;
    double pitchRate;
    double rollRate;
    double sampleTime = 0;
    int64_t previousSampleTime = 0;
    int64_t currentTime = 0;

    double targetPitch = 0;
    double targetRoll = 0;

    double pitchCmd;
    double rollCmd;

    PretoFlyteFC_Pid_t pitchPid = {0U};
    pitchPid.Kp = 0.45;
    pitchPid.Ki = 0.3;
    pitchPid.Kd = 0.5;
    PretoFlyteFC_Pid_t rollPid = {0U};
    rollPid.Kp = 0.45;
    rollPid.Ki = 0.3;
    rollPid.Kd = 0.5;

    Kalman_1dFilterInit(&PretoFlyteFC_PitchFilter, 0, 4, 4, 3);
    Kalman_1dFilterInit(&PretoFlyteFC_RollFilter, 0, 4, 4, 3);

    for (;;)
    {
        /* Read accelerometer and gyroscope */
        if (Lsm9ds1_AccelAvailable(&PretoFlyteFC_Lsm9ds1Handle))
        {
            Lsm9ds1_AccelRead(&PretoFlyteFC_Lsm9ds1Handle);
        }
        if (Lsm9ds1_GyroAvailable(&PretoFlyteFC_Lsm9ds1Handle))
        {
            Lsm9ds1_GyroRead(&PretoFlyteFC_Lsm9ds1Handle);
        }

        /* Calculate and update sample time */
        currentTime = PretoFlyteFC_Micros();
        sampleTime = (double)(currentTime - previousSampleTime) * US_TO_S;
        previousSampleTime = currentTime;

        /* Pitch = atan(-ax, sqrt(ay^2, az^2)) */
        pitch = atan2(PretoFlyteFC_Lsm9ds1Handle.Accel.Reading.X * -1.0,
                      sqrt(PretoFlyteFC_Lsm9ds1Handle.Accel.Reading.Y * PretoFlyteFC_Lsm9ds1Handle.Accel.Reading.Y +
                           PretoFlyteFC_Lsm9ds1Handle.Accel.Reading.Z * PretoFlyteFC_Lsm9ds1Handle.Accel.Reading.Z));
        pitchRate = -PretoFlyteFC_Lsm9ds1Handle.Gyro.Reading.Y;

        /* Roll = atan2(ay, az) */
        roll = atan2(PretoFlyteFC_Lsm9ds1Handle.Accel.Reading.Y, PretoFlyteFC_Lsm9ds1Handle.Accel.Reading.Z);
        rollRate = -PretoFlyteFC_Lsm9ds1Handle.Gyro.Reading.X;

        // ESP_LOGI(PretoFlyteFC_LogTag, "Ax: %.02lf, Ay: %.02lf, Az: %.02lf",
        //          PretoFlyteFC_Lsm9ds1Handle.Accel.Reading.X,
        //          PretoFlyteFC_Lsm9ds1Handle.Accel.Reading.Y,
        //          PretoFlyteFC_Lsm9ds1Handle.Accel.Reading.Z);
        // ESP_LOGI(PretoFlyteFC_LogTag, "Gx: %.02lf, Gy: %.02lf, Gz: %.02lf",
        //          PretoFlyteFC_Lsm9ds1Handle.Gyro.Reading.X,
        //          PretoFlyteFC_Lsm9ds1Handle.Gyro.Reading.Y,
        //          PretoFlyteFC_Lsm9ds1Handle.Gyro.Reading.Z);

        /* Convert from radians to degrees */
        pitch *= 180.0 / M_PI;
        roll *= 180.0 / M_PI;
        // ESP_LOGI(PretoFlyteFC_LogTag, "Pitch[Raw]: %.02lf deg, Roll[Raw]: %.02lf deg", pitch, roll);
        // ESP_LOGI(PretoFlyteFC_LogTag, "Pitch[Rate]: %.02lf deg, Roll[Rate]: %.02lf deg", pitchRate, rollRate);

        /* Correct DC? */
        pitch -= 1.25;
        roll += 0.7;

        /* Apply 1D Kalman filter to pitch and roll */
        Kalman_1dFilter(&PretoFlyteFC_PitchFilter, pitchRate, pitch, sampleTime);
        Kalman_1dFilter(&PretoFlyteFC_RollFilter, rollRate, roll, sampleTime);
        // ESP_LOGI(PretoFlyteFC_LogTag, "Pitch[Filter]: %.02lf deg, Roll[Filter]: %.02lf deg", PretoFlyteFC_PitchFilter.State, PretoFlyteFC_RollFilter.State);

        /* Apply PID controller */
        targetPitch = (ReceiverPitchPercentage * (ANGLE_MAX - ANGLE_MIN)) - ANGLE_MAX;
        targetRoll = (ReceiverRollPercentage * (ANGLE_MAX - ANGLE_MIN)) - ANGLE_MAX;
        PretoFlyteFC_Pid(&pitchPid, targetPitch, PretoFlyteFC_PitchFilter.State, sampleTime);
        PretoFlyteFC_Pid(&rollPid, targetRoll, PretoFlyteFC_RollFilter.State, sampleTime);
        // ESP_LOGI(PretoFlyteFC_LogTag, "Pitch[Pid]: %.02lf deg, Roll[Pid]: %.02lf deg", pitchPid.Output, rollPid.Output);
        pitchCmd = pitchPid.Output;
        rollCmd = rollPid.Output;

        /* Constrain angle values */
        PretoFlyteFC_Constrain(&pitchCmd, ANGLE_MIN, ANGLE_MAX);
        PretoFlyteFC_Constrain(&rollCmd, ANGLE_MIN, ANGLE_MAX);

        /* Convert angle values to percentage */
        pitchCmd = (pitchCmd + ANGLE_MAX) / (ANGLE_MAX - ANGLE_MIN);
        rollCmd = (rollCmd + ANGLE_MAX) / (ANGLE_MAX - ANGLE_MIN);

        // ESP_LOGI(PretoFlyteFC_LogTag, "Pitch[Cmd]: %.02lf deg, Roll[Cmd]: %.02lf deg", pitchCmd, rollCmd);

        /* Transmit angle percentages */
        Sbus_SetChannel(&PretoFlyteFC_SbusPayload, SBUS_PITCH_CHANNEL, SBUS_CHANNEL_VALUE(pitchCmd));
        Sbus_SetChannel(&PretoFlyteFC_SbusPayload, SBUS_ROLL_CHANNEL, SBUS_CHANNEL_VALUE(rollCmd));
        Sbus_SetChannel(&PretoFlyteFC_SbusPayload, SBUS_YAW_CHANNEL, SBUS_CHANNEL_VALUE(ReceiverYawPercentage));
        Sbus_SetChannel(&PretoFlyteFC_SbusPayload, SBUS_THROTTLE_CHANNEL, SBUS_CHANNEL_VALUE(ReceiverThrottlePercentage));
        Sbus_SetChannel(&PretoFlyteFC_SbusPayload, SBUS_AUX_1_CHANNEL, SBUS_CHANNEL_VALUE(ReceiverArmPercentage));
        Sbus_Tx(&PretoFlyteFC_SbusHandle, &PretoFlyteFC_SbusPayload);
        Sbus_Rx(&PretoFlyteFC_SbusHandle, &PretoFlyteFC_SbusPayload);

        vTaskDelay(10U / portTICK_PERIOD_MS);
    }
}

static void PretoFlyteFC_Pid(PretoFlyteFC_Pid_t *const pid, const double setpoint, const double process, const double sampleTime)
{
    if (pid != NULL)
    {
        double error = setpoint - process;
        double derivative = (error - pid->PrevError) / sampleTime;
        pid->Integral += error * sampleTime;

        pid->Output = (pid->Kp * error) + (pid->Ki * pid->Integral) + (pid->Kd * derivative);

        pid->PrevError = error;
    }
}

static void IRAM_ATTR PretoFlyteFC_ReceiverIsrHandler(void *arg)
{
    uint32_t gpioNum = (uint32_t)arg;
    PretoFlyteFC_ReceiverEvent_t receiverEvent = {
        .GpioNum = gpioNum,
        .Time = PretoFlyteFC_Micros(),
        .Level = (uint8_t)gpio_get_level(gpioNum),
    };

    xQueueSendFromISR(ReceiverEventQueueHandle, &receiverEvent, NULL);
}

static void PretoFlyteFC_ReadReceiverTask(void *arg)
{
    PretoFlyteFC_ReceiverEvent_t receiverEvent;

    for (;;)
    {
        if (xQueueReceive(ReceiverEventQueueHandle, &receiverEvent, portMAX_DELAY))
        {
            switch (receiverEvent.GpioNum)
            {
            case RX_ROLL_PIN:
                if (receiverEvent.Level)
                {
                    ReceiverRollPrevTime = receiverEvent.Time;
                }
                else
                {
                    ReceiverRollPercentage = PretoFlyteFC_ReceiverPercentage(receiverEvent.Time, ReceiverRollPrevTime, RECEIVER_ROLL_MIN, RECEIVER_ROLL_MAX);
                }
                break;
            case RX_PITCH_PIN:
                if (receiverEvent.Level)
                {
                    ReceiverPitchPrevTime = receiverEvent.Time;
                }
                else
                {
                    ReceiverPitchPercentage = PretoFlyteFC_ReceiverPercentage(receiverEvent.Time, ReceiverPitchPrevTime, RECEIVER_PITCH_MIN, RECEIVER_PITCH_MAX);
                }
                break;
            case RX_YAW_PIN:
                if (receiverEvent.Level)
                {
                    ReceiverYawPrevTime = receiverEvent.Time;
                }
                else
                {
                    ReceiverYawPercentage = PretoFlyteFC_ReceiverPercentage(receiverEvent.Time, ReceiverYawPrevTime, RECEIVER_YAW_MIN, RECEIVER_YAW_MAX);
                }
                break;
            case RX_THROTTLE_PIN:
                if (receiverEvent.Level)
                {
                    ReceiverThrottlePrevTime = receiverEvent.Time;
                }
                else
                {
                    ReceiverThrottlePercentage = PretoFlyteFC_ReceiverPercentage(receiverEvent.Time, ReceiverThrottlePrevTime, RECEIVER_THROTTLE_MIN, RECEIVER_THROTTLE_MAX);
                }
                break;
            case RX_ARM_PIN:
                if (receiverEvent.Level)
                {
                    ReceiverArmPrevTime = receiverEvent.Time;
                }
                else
                {
                    ReceiverArmPercentage = PretoFlyteFC_ReceiverPercentage(receiverEvent.Time, ReceiverArmPrevTime, RECEIVER_ARM_MIN, RECEIVER_ARM_MAX);
                }
                break;
            default:
                break;
            }
        }
    }

    vTaskDelete(NULL);
}

static inline double PretoFlyteFC_ReceiverPercentage(const int64_t time, const int64_t prevTime, const double min, const double max)
{
    return (((time - prevTime) * US_TO_S * RECEIVER_FREQUENCY) - min) / (max - min);
}

static inline void PretoFlyteFC_Constrain(double *const value, const double min, const double max)
{
    if (*value < min)
    {
        *value = min;
    }

    if (*value > max)
    {
        *value = max;
    }
}

static inline int64_t PretoFlyteFC_Micros(void)
{
    return esp_timer_get_time();
}

static inline int64_t PretoFlyteFC_Millis(void)
{
    return (int64_t)(PretoFlyteFC_Micros() / US_PER_MS);
}
