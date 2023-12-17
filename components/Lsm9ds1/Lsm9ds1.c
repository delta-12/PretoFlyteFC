/**
 * @file Lsm9ds1.h
 *
 * @brief Lsm9ds1 driver for the ESP32.
 *
 ******************************************************************************/

/* Includes
 ******************************************************************************/
#include "esp_err.h"
#include "esp_log.h"
#include "Lsm9ds1.h"
#include "Lsm9ds1_Registers.h"
#include <string.h>

/* Defines
 ******************************************************************************/

#define LSM9DS1_SPI_GPIO_UNUSED (-1)          /* Unused GPIO in SPI config should be set to -1 */
#define LSM9DS1_SPI_MAX_TRANSFER_SZ 0U        /* Maximum size of SPI transfer, 0 defaults to 4092 when DMA is enabled */
#define LSM9DS1_SPI_CLK_FREQ_HZ 10000000U     /* 10MHz SPI clock frequency */
#define LSM9DS1_SPI_MODE_0 0U                 /* CPOL = 0, CPHA = 0 */
#define LSM9DS1_SPI_QUEUE_SIZE 10U            /* Depth of 10 items for SPI queues */
#define LSM9DS1_SPI_TRANSACTION_LENGTH_8 8U   /* SPI transaction 8 bits in length */
#define LSM9DS1_SPI_TRANSACTION_LENGTH_16 16U /* SPI transaction 16 bits in length */

#define LSM9DS1_AG_SPI_COMMAND_BITS 1U /* 1 read/write command bit */
#define LSM9DS1_AG_SPI_ADDRESS_BITS 7U /* 7 address bits */
#define LSM9DS1_AG_SPI_CMD_READ 0x01U  /* SPI cmd bit is 1 to read from accel/gyro */
#define LSM9DS1_AG_SPI_CMD_WRITE 0x00U /* SPI cmd bit is 0 to write to accel/gyro */
#define LSM9DS1_AG_ADDRESS_MASK 0x7FU  /* Accel/gyro addresses are 7 bits */

#define LSM9DS1_M_SPI_COMMAND_BITS 2U /* 1 read/write command bit, 1 MS command bit */
#define LSM9DS1_M_SPI_ADDRESS_BITS 6U /* 6 address bits */
#define LSM9DS1_M_SPI_CMD_READ 0x02U  /* SPI cmd bit is 1 to read from mag */
#define LSM9DS1_M_SPI_CMD_WRITE 0x00U /* SPI cmd bit is 0 to write to mag */
#define LSM9DS1_M_SPI_CMD_MS 0x01U    /* Auto-increment address in multiple read/write SPI cmds for mag */
#define LSM9DS1_M_ADDRESS_MASK 0x3FU  /* Mag addresses are 6 bits */

#define LSM9DS1_AG_WHO_AM_I_RSP 0x68U /* WHO_AM_I device identification register value */
#define LSM9DS1_M_WHO_AM_RSP 0x3DU    /* WHO_AM_I_M device identification register value  */

#define LSM9DS1_FIFO_MODE_BYPASS 0U               /* Bypass mode. FIFO turned off. */
#define LSM9DS1_FIFO_MODE_FIFO 1U                 /* FIFO mode. Stops collecting data when FIFO is full. */
#define LSM9DS1_FIFO_MODE_CONTINUOUS_TO_FIFO 3U   /* Continuous mode until trigger is deasserted, then FIFO mode. */
#define LSM9DS1_FIFO_MODE_BYPASS_TO_CONTINUOUS 4U /* Bypass mode until trigger is deasserted, then Continuous mode. */
#define LSM9DS1_FIFO_MODE_CONTINUOUS 6U           /* Continuous mode. If the FIFO is full, the new sample overwrites the older sample. */

/* Typedefs
 ******************************************************************************/

typedef uint8_t Lsm9ds1_FifoMode_t;
typedef uint8_t Lsm9ds1_FifoThreshold_t;

/* Globals
 ******************************************************************************/

static const char *Lsm9ds1_LogTag = "LSM9DS1";

/* Function Prototypes
 ******************************************************************************/

static inline bool Lsm9ds1_AccelCalc(const Lsm9ds1_Handle_t *const handle, const Lsm9ds1_AxisRaw_t axisRaw, Lsm9ds1_AxisCalc_t *const result);
static inline bool Lsm9ds1_GyroCalc(const Lsm9ds1_Handle_t *const handle, const Lsm9ds1_AxisRaw_t axisRaw, Lsm9ds1_AxisCalc_t *const result);
static inline bool Lsm9ds1_MagCalc(const Lsm9ds1_Handle_t *const handle, const Lsm9ds1_AxisRaw_t axisRaw, Lsm9ds1_AxisCalc_t *const result);
static inline esp_err_t Lsm9ds1_AccelGyroSpiTransmit(const Lsm9ds1_Handle_t *const handle, spi_transaction_t *const spiTranscation, const uint64_t address, const size_t length);
static esp_err_t Lsm9ds1_AccelGyroSpiRead(const Lsm9ds1_Handle_t *const handle, spi_transaction_t *const spiTranscation, const uint64_t address, const size_t length);
static esp_err_t Lsm9ds1_AccelGyroSpiWrite(const Lsm9ds1_Handle_t *const handle, spi_transaction_t *const spiTranscation, const uint64_t address, const size_t length);
static void Lsm9ds1_EnableFifo(const Lsm9ds1_Handle_t *const handle, const bool enable);
static void Lsm9ds1_SetFifo(const Lsm9ds1_Handle_t *const handle, const Lsm9ds1_FifoMode_t fifoMode, const Lsm9ds1_FifoThreshold_t fifoThreshold);
static inline esp_err_t Lsm9ds1_MagSpiTransmit(const Lsm9ds1_Handle_t *const handle, spi_transaction_t *const spiTranscation, const uint64_t address, const size_t length);
static esp_err_t Lsm9ds1_MagSpiRead(const Lsm9ds1_Handle_t *const handle, spi_transaction_t *const spiTranscation, const bool ms, const uint64_t address, const size_t length);
static esp_err_t Lsm9ds1_MagSpiWrite(const Lsm9ds1_Handle_t *const handle, spi_transaction_t *const spiTranscation, const bool ms, const uint64_t address, const size_t length);

/* Function Definitions
 ******************************************************************************/

bool Lsm9ds1_Init(Lsm9ds1_Handle_t *const handle, const Lsm9ds1_SpiHost_t spiHost, const Lsm9ds1_PinConfig_t *const pinConfig)
{
    bool success = false;

    if (pinConfig != NULL)
    {
        spi_bus_config_t busConfig = {
            .mosi_io_num = pinConfig->Mosi,
            .miso_io_num = pinConfig->Miso,
            .sclk_io_num = pinConfig->Clk,
            .quadhd_io_num = LSM9DS1_SPI_GPIO_UNUSED,
            .quadwp_io_num = LSM9DS1_SPI_GPIO_UNUSED,
            .max_transfer_sz = LSM9DS1_SPI_MAX_TRANSFER_SZ,
        };

        spi_device_interface_config_t accelGyroConfig = {
            .clock_speed_hz = LSM9DS1_SPI_CLK_FREQ_HZ,
            .mode = LSM9DS1_SPI_MODE_0,
            .spics_io_num = pinConfig->CsAccelGyro,
            .queue_size = LSM9DS1_SPI_QUEUE_SIZE,
            .command_bits = LSM9DS1_AG_SPI_COMMAND_BITS,
            .address_bits = LSM9DS1_AG_SPI_ADDRESS_BITS,
        };

        spi_device_interface_config_t magConfig = {
            .clock_speed_hz = LSM9DS1_SPI_CLK_FREQ_HZ,
            .mode = LSM9DS1_SPI_MODE_0,
            .spics_io_num = pinConfig->CsMag,
            .queue_size = LSM9DS1_SPI_QUEUE_SIZE,
            .command_bits = LSM9DS1_M_SPI_COMMAND_BITS,
            .address_bits = LSM9DS1_M_SPI_ADDRESS_BITS,
        };

        ESP_ERROR_CHECK(spi_bus_initialize(spiHost, &busConfig, SPI_DMA_CH_AUTO));
        ESP_ERROR_CHECK(spi_bus_add_device(spiHost, &accelGyroConfig, &handle->AccelGyroSpiHandle));
        ESP_ERROR_CHECK(spi_bus_add_device(spiHost, &magConfig, &handle->MagSpiHandle));

        success = Lsm9ds1_WhoAmI(handle);
        if (success)
        {
            ESP_LOGI(Lsm9ds1_LogTag, "LSM9DS1 successfully initialized");
        }
        else
        {
            ESP_LOGE(Lsm9ds1_LogTag, "LSM9DS1 failed to initialize");
        }

        memset(&(handle->Accel.BiasRaw), 0U, sizeof(handle->Accel.BiasRaw));
        memset(&(handle->Gyro.BiasRaw), 0U, sizeof(handle->Gyro.BiasRaw));
        memset(&(handle->Mag.BiasRaw), 0U, sizeof(handle->Mag.BiasRaw));
        memset(&(handle->Accel.Bias), 0U, sizeof(handle->Accel.Bias));
        memset(&(handle->Gyro.Bias), 0U, sizeof(handle->Gyro.Bias));
        memset(&(handle->Mag.Bias), 0U, sizeof(handle->Mag.Bias));
    }

    return success;
}

bool Lsm9ds1_WhoAmI(const Lsm9ds1_Handle_t *const handle)
{
    bool success = false;

    if (handle != NULL)
    {
        spi_transaction_t spiTransactionAccelGyro = {0U};
        spi_transaction_t spiTransactionMag = {0U};

        if (Lsm9ds1_AccelGyroSpiRead(handle, &spiTransactionAccelGyro, LSM9DS1_WHO_AM_I_XG, LSM9DS1_SPI_TRANSACTION_LENGTH_8) == ESP_OK &&
            Lsm9ds1_MagSpiRead(handle, &spiTransactionMag, false, LSM9DS1_WHO_AM_I_M, LSM9DS1_SPI_TRANSACTION_LENGTH_8) == ESP_OK)
        {
            /* TODO remove magic numbers */
            success = (((spiTransactionAccelGyro.rx_data[0U] << 8U) | spiTransactionMag.rx_data[0U]) == ((LSM9DS1_AG_WHO_AM_I_RSP << 8) | LSM9DS1_M_WHO_AM_RSP));
        }
    }

    return success;
}

void Lsm9ds1_AccelInit(Lsm9ds1_Handle_t *const handle)
{
    if (handle != NULL)
    {
        uint8_t registerValue;
        spi_transaction_t spiTransaction;

        /* CTRL_REG5_XL */
        registerValue = 0U;
        registerValue |= (1U << 5U); /* Zen_XL */
        registerValue |= (1U << 4U); /* Yen_XL */
        registerValue |= (1U << 3U); /* Xen_XL */
        memset(&spiTransaction, 0U, sizeof(spiTransaction));
        spiTransaction.tx_data[0U] = registerValue;
        Lsm9ds1_AccelGyroSpiWrite(handle, &spiTransaction, LSM9DS1_CTRL_REG5_XL, LSM9DS1_SPI_TRANSACTION_LENGTH_8);

        /* CTRL_REG6_XL */
        registerValue = 0U;
        registerValue |= (6U << 5U); /* ODR_XL */
        registerValue |= (0U << 3U); /* FS_XL */
        memset(&spiTransaction, 0U, sizeof(spiTransaction));
        spiTransaction.tx_data[0U] = registerValue;
        Lsm9ds1_AccelGyroSpiWrite(handle, &spiTransaction, LSM9DS1_CTRL_REG6_XL, LSM9DS1_SPI_TRANSACTION_LENGTH_8);

        /* CTRL_REG7_XL */
        registerValue = 0U; /* Default values */
        memset(&spiTransaction, 0U, sizeof(spiTransaction));
        spiTransaction.tx_data[0U] = registerValue;
        Lsm9ds1_AccelGyroSpiWrite(handle, &spiTransaction, LSM9DS1_CTRL_REG7_XL, LSM9DS1_SPI_TRANSACTION_LENGTH_8);

        /* Set resolution */
        handle->Accel.Resolution = 0.000061;
    }
}

void Lsm9ds1_GyroInit(Lsm9ds1_Handle_t *const handle)
{
    /* TODO
    set operating mode
    set power mode
    set buffer mode

    programmable fifo threshold status, fif overrun events, and number of unread sample stored in FIFO_SRC
    can generate interrupts on INTx_A/G pins

    Modes:
        Bypass mode- new data overwrites old data, only first address is used
        FIFO mode- data is stored in FIFO until overwritten, reset FIFO by selecting Bypass mode, can resize FIFO depth, FIFO threshold interrupt can be enabled
        Continuous mode- old data is discarded as new data arrives, FIFO threshold flag for unread samples, can be routed to INTx_A/G pins
        Continuous-to-FIFO mode- operates according to bit in INT_GEN_SRC_XL
        Bypass-to-continuous mode- operates according to bit in INT_GEN_SRC_XL
     */

    if (handle != NULL)
    {
        uint8_t registerValue;
        spi_transaction_t spiTransaction;

        /* CTRL_REG1_G */
        registerValue = 0U;
        registerValue |= (6U << 5U); /* ODR_G */
        registerValue |= (0U << 3U); /* FS_G */
        registerValue |= 0U;         /* BW_G */
        memset(&spiTransaction, 0U, sizeof(spiTransaction));
        spiTransaction.tx_data[0U] = registerValue;
        Lsm9ds1_AccelGyroSpiWrite(handle, &spiTransaction, LSM9DS1_CTRL_REG1_G, LSM9DS1_SPI_TRANSACTION_LENGTH_8);

        /* CTRL_REG2_G */
        registerValue = 0U; /* Default values */
        memset(&spiTransaction, 0U, sizeof(spiTransaction));
        spiTransaction.tx_data[0U] = registerValue;
        Lsm9ds1_AccelGyroSpiWrite(handle, &spiTransaction, LSM9DS1_CTRL_REG2_G, LSM9DS1_SPI_TRANSACTION_LENGTH_8);

        /* CTRL_REG3_G */
        registerValue = 0U; /* Default values */
        memset(&spiTransaction, 0U, sizeof(spiTransaction));
        spiTransaction.tx_data[0U] = registerValue;
        Lsm9ds1_AccelGyroSpiWrite(handle, &spiTransaction, LSM9DS1_CTRL_REG3_G, LSM9DS1_SPI_TRANSACTION_LENGTH_8);

        /* CTRL_REG4 */
        registerValue = 0U;
        registerValue |= (1U << 5U); /* Zen_G */
        registerValue |= (1U << 4U); /* Yen_G */
        registerValue |= (1U << 3U); /* Xen_G */
        registerValue |= (1U << 1U); /* LIR_XL1 */
        memset(&spiTransaction, 0U, sizeof(spiTransaction));
        spiTransaction.tx_data[0U] = registerValue;
        Lsm9ds1_AccelGyroSpiWrite(handle, &spiTransaction, LSM9DS1_CTRL_REG4, LSM9DS1_SPI_TRANSACTION_LENGTH_8);

        /* ORIENT_CFG_G */
        registerValue = 0U; /* Default values */
        memset(&spiTransaction, 0U, sizeof(spiTransaction));
        spiTransaction.tx_data[0U] = registerValue;
        Lsm9ds1_AccelGyroSpiWrite(handle, &spiTransaction, LSM9DS1_ORIENT_CFG_G, LSM9DS1_SPI_TRANSACTION_LENGTH_8);

        /* Set resolution */
        handle->Gyro.Resolution = 0.00875;
    }
}

void Lsm9ds1_MagInit(Lsm9ds1_Handle_t *const handle)
{
    if (handle != NULL)
    {
        uint8_t registerValue;
        spi_transaction_t spiTransaction;

        /* CTRL_REG1_M */
        registerValue = 0U;
        registerValue |= (3U << 5U); /* OM */
        registerValue |= (7U << 2U); /* DO */
        memset(&spiTransaction, 0U, sizeof(spiTransaction));
        spiTransaction.tx_data[0U] = registerValue;
        Lsm9ds1_MagSpiWrite(handle, &spiTransaction, false, LSM9DS1_CTRL_REG1_M, LSM9DS1_SPI_TRANSACTION_LENGTH_8);

        /* CTRL_REG2_M */
        registerValue = 0U; /* Default values */
        memset(&spiTransaction, 0U, sizeof(spiTransaction));
        spiTransaction.tx_data[0U] = registerValue;
        Lsm9ds1_MagSpiWrite(handle, &spiTransaction, false, LSM9DS1_CTRL_REG2_M, LSM9DS1_SPI_TRANSACTION_LENGTH_8);

        /* CTRL_REG3_M */
        registerValue = 0U; /* Default values */
        memset(&spiTransaction, 0U, sizeof(spiTransaction));
        spiTransaction.tx_data[0U] = registerValue;
        Lsm9ds1_MagSpiWrite(handle, &spiTransaction, false, LSM9DS1_CTRL_REG3_M, LSM9DS1_SPI_TRANSACTION_LENGTH_8);

        /* CTRL_REG4_M */
        registerValue = 0U;
        registerValue |= (3U << 2U); /* OMZ */
        memset(&spiTransaction, 0U, sizeof(spiTransaction));
        spiTransaction.tx_data[0U] = registerValue;
        Lsm9ds1_MagSpiWrite(handle, &spiTransaction, false, LSM9DS1_CTRL_REG4_M, LSM9DS1_SPI_TRANSACTION_LENGTH_8);

        /* CTRL_REG5_M */
        registerValue = 0U; /* Default values */
        memset(&spiTransaction, 0U, sizeof(spiTransaction));
        spiTransaction.tx_data[0U] = registerValue;
        Lsm9ds1_MagSpiWrite(handle, &spiTransaction, false, LSM9DS1_CTRL_REG5_M, LSM9DS1_SPI_TRANSACTION_LENGTH_8);

        /* Set resolution */
        handle->Mag.Resolution = 0.00014;
    }
}

/* TODO fix calibration */
void Lsm9ds1_AccelGyroCalibrate(Lsm9ds1_Handle_t *const handle)
{
    if (handle != NULL)
    {
        spi_transaction_t spiTransaction;
        uint8_t samples = 0U;

        Lsm9ds1_EnableFifo(handle, true);
        Lsm9ds1_SetFifo(handle, LSM9DS1_FIFO_MODE_FIFO, 0x1FU);

        /* Fill FIFO */
        while (samples < 0x1F)
        {
            memset(&spiTransaction, 0U, sizeof(spiTransaction));
            Lsm9ds1_AccelGyroSpiRead(handle, &spiTransaction, LSM9DS1_FIFO_SRC, LSM9DS1_SPI_TRANSACTION_LENGTH_8);
            samples = spiTransaction.rx_data[0U];
            vTaskDelay(10U / portTICK_PERIOD_MS);
        }

        /* Calculate raw biases */
        for (uint8_t i = 0U; i < samples; i++)
        {
            Lsm9ds1_AccelRead(handle);
            handle->Accel.BiasRaw.X += handle->Accel.ReadingRaw.X;
            handle->Accel.BiasRaw.Y += handle->Accel.ReadingRaw.Y;
            handle->Accel.BiasRaw.Z += handle->Accel.ReadingRaw.Z - (int16_t)(1. / handle->Accel.Resolution);

            Lsm9ds1_GyroRead(handle);
            handle->Gyro.BiasRaw.X += handle->Gyro.ReadingRaw.X;
            handle->Gyro.BiasRaw.Y += handle->Gyro.ReadingRaw.Y;
            handle->Gyro.BiasRaw.Z += handle->Gyro.ReadingRaw.Z;
        }
        handle->Accel.BiasRaw.X = handle->Accel.BiasRaw.X / samples;
        handle->Accel.BiasRaw.Y = handle->Accel.BiasRaw.Y / samples;
        handle->Accel.BiasRaw.Z = handle->Accel.BiasRaw.Z / samples;
        handle->Gyro.BiasRaw.X = handle->Gyro.BiasRaw.X / samples;
        handle->Gyro.BiasRaw.Y = handle->Gyro.BiasRaw.Y / samples;
        handle->Gyro.BiasRaw.Z = handle->Gyro.BiasRaw.Z / samples;

        /* Calculate biases */
        Lsm9ds1_AccelCalc(handle, handle->Accel.BiasRaw.X, &(handle->Accel.Bias.X));
        Lsm9ds1_AccelCalc(handle, handle->Accel.BiasRaw.Y, &(handle->Accel.Bias.Y));
        Lsm9ds1_AccelCalc(handle, handle->Accel.BiasRaw.Z, &(handle->Accel.Bias.Z));
        Lsm9ds1_GyroCalc(handle, handle->Gyro.BiasRaw.X, &(handle->Gyro.Bias.X));
        Lsm9ds1_GyroCalc(handle, handle->Gyro.BiasRaw.Y, &(handle->Gyro.Bias.Y));
        Lsm9ds1_GyroCalc(handle, handle->Gyro.BiasRaw.Z, &(handle->Gyro.Bias.Z));

        Lsm9ds1_EnableFifo(handle, false);
        Lsm9ds1_SetFifo(handle, LSM9DS1_FIFO_MODE_BYPASS, 0x00U);

        ESP_LOGI(Lsm9ds1_LogTag, "Acceleromter and Gyroscope calibration complete");
        ESP_LOGI(Lsm9ds1_LogTag, "Accelerometer bias [x, y, z]: %lf, %lf, %lf", handle->Accel.Bias.X, handle->Accel.Bias.Y, handle->Accel.Bias.Z);
        ESP_LOGI(Lsm9ds1_LogTag, "Gyroscope bias [x, y, z]: %lf, %lf, %lf", handle->Gyro.Bias.X, handle->Gyro.Bias.Y, handle->Gyro.Bias.Z);
    }
}

void Lsm9ds1_MagCalibrate(Lsm9ds1_Handle_t *const handle)
{
    if (handle != NULL)
    {
        Lsm9ds1_SensorReadingRaw_t magMin = {0U};
        Lsm9ds1_SensorReadingRaw_t magMax = {0U};

        for (uint8_t i = 0U; i < 128U; i++)
        {
            while (!Lsm9ds1_MagAvailable(handle))
            {
            }
            Lsm9ds1_MagRead(handle);

            if (handle->Mag.ReadingRaw.X < magMin.X)
            {
                magMin.X = handle->Mag.ReadingRaw.X;
            }
            if (handle->Mag.ReadingRaw.Y < magMin.Y)
            {
                magMin.Y = handle->Mag.ReadingRaw.Y;
            }
            if (handle->Mag.ReadingRaw.Z < magMin.Z)
            {
                magMin.Z = handle->Mag.ReadingRaw.Z;
            }

            if (handle->Mag.ReadingRaw.X > magMax.X)
            {
                magMax.X = handle->Mag.ReadingRaw.X;
            }
            if (handle->Mag.ReadingRaw.Y > magMax.Y)
            {
                magMax.Y = handle->Mag.ReadingRaw.Y;
            }
            if (handle->Mag.ReadingRaw.Z > magMax.Z)
            {
                magMax.Z = handle->Mag.ReadingRaw.Z;
            }
        }

        /* Calculate raw biases */
        handle->Mag.BiasRaw.X = (magMax.X - magMin.X) / 2U;
        handle->Mag.BiasRaw.Y = (magMax.Y - magMin.Y) / 2U;
        handle->Mag.BiasRaw.Z = (magMax.Z - magMin.Z) / 2U;

        /* Calculate biases */
        Lsm9ds1_MagCalc(handle, handle->Mag.BiasRaw.X, &(handle->Mag.Bias.X));
        Lsm9ds1_MagCalc(handle, handle->Mag.BiasRaw.Y, &(handle->Mag.Bias.Y));
        Lsm9ds1_MagCalc(handle, handle->Mag.BiasRaw.Z, &(handle->Mag.Bias.Z));

        /* Load mag offset */
        /* TODO */
    }
}

bool Lsm9ds1_AccelAvailable(const Lsm9ds1_Handle_t *const handle)
{
    bool available = false;

    if (handle != NULL)
    {
        spi_transaction_t spiTransaction = {0U};

        Lsm9ds1_AccelGyroSpiRead(handle, &spiTransaction, LSM9DS1_STATUS_REG_1, LSM9DS1_SPI_TRANSACTION_LENGTH_8);

        available = ((spiTransaction.rx_data[0U] & (1U << 0U)) == (1U << 0U));
    }

    return available;
}

bool Lsm9ds1_GyroAvailable(const Lsm9ds1_Handle_t *const handle)
{
    bool available = false;

    if (handle != NULL)
    {
        spi_transaction_t spiTransaction = {0U};

        Lsm9ds1_AccelGyroSpiRead(handle, &spiTransaction, LSM9DS1_STATUS_REG_1, LSM9DS1_SPI_TRANSACTION_LENGTH_8);

        available = ((spiTransaction.rx_data[0U] & (1U << 1U)) == (1U << 1U));
    }

    return available;
}

bool Lsm9ds1_MagAvailable(const Lsm9ds1_Handle_t *const handle)
{
    bool available = false;

    if (handle != NULL)
    {
        /* TODO */
    }

    return available;
}

bool Lsm9ds1_TempAvailable(const Lsm9ds1_Handle_t *const handle)
{
    bool available = false;

    if (handle != NULL)
    {
        spi_transaction_t spiTransaction = {0U};

        Lsm9ds1_AccelGyroSpiRead(handle, &spiTransaction, LSM9DS1_STATUS_REG_1, LSM9DS1_SPI_TRANSACTION_LENGTH_8);

        available = ((spiTransaction.rx_data[0U] & (1U << 2U)) == (1U << 2U));
    }

    return available;
}

void Lsm9ds1_AccelRead(Lsm9ds1_Handle_t *const handle)
{
    if (handle != NULL)
    {
        spi_transaction_t spiTransaction;

        /* TODO perform all reads in single transmit */

        /* Read X */
        memset(&spiTransaction, 0U, sizeof(spiTransaction));
        Lsm9ds1_AccelGyroSpiRead(handle, &spiTransaction, LSM9DS1_OUT_X_L_XL, LSM9DS1_SPI_TRANSACTION_LENGTH_16);
        handle->Accel.ReadingRaw.X = (spiTransaction.rx_data[1U] << 8U) | spiTransaction.rx_data[0U];
        /* Read Y */
        memset(&spiTransaction, 0U, sizeof(spiTransaction));
        Lsm9ds1_AccelGyroSpiRead(handle, &spiTransaction, LSM9DS1_OUT_Y_L_XL, LSM9DS1_SPI_TRANSACTION_LENGTH_16);
        handle->Accel.ReadingRaw.Y = (spiTransaction.rx_data[1U] << 8U) | spiTransaction.rx_data[0U];

        /* Read Z */
        memset(&spiTransaction, 0U, sizeof(spiTransaction));
        Lsm9ds1_AccelGyroSpiRead(handle, &spiTransaction, LSM9DS1_OUT_Z_L_XL, LSM9DS1_SPI_TRANSACTION_LENGTH_16);
        handle->Accel.ReadingRaw.Z = (spiTransaction.rx_data[1U] << 8U) | spiTransaction.rx_data[0U];

        handle->Accel.ReadingRaw.X -= handle->Accel.BiasRaw.X;
        handle->Accel.ReadingRaw.Y -= handle->Accel.BiasRaw.Y;
        handle->Accel.ReadingRaw.Z -= handle->Accel.BiasRaw.Z;

        Lsm9ds1_AccelCalc(handle, handle->Accel.ReadingRaw.X, &(handle->Accel.Reading.X));
        Lsm9ds1_AccelCalc(handle, handle->Accel.ReadingRaw.Y, &(handle->Accel.Reading.Y));
        Lsm9ds1_AccelCalc(handle, handle->Accel.ReadingRaw.Z, &(handle->Accel.Reading.Z));
    }
}

void Lsm9ds1_GyroRead(Lsm9ds1_Handle_t *const handle)
{
    if (handle != NULL)
    {
        spi_transaction_t spiTransaction;

        /* TODO perform all reads in single transmit */

        /* Read X */
        memset(&spiTransaction, 0U, sizeof(spiTransaction));
        Lsm9ds1_AccelGyroSpiRead(handle, &spiTransaction, LSM9DS1_OUT_X_L_G, LSM9DS1_SPI_TRANSACTION_LENGTH_16);
        handle->Gyro.ReadingRaw.X = (spiTransaction.rx_data[1U] << 8U) | spiTransaction.rx_data[0U];

        /* Read Y */
        memset(&spiTransaction, 0U, sizeof(spiTransaction));
        Lsm9ds1_AccelGyroSpiRead(handle, &spiTransaction, LSM9DS1_OUT_Y_L_G, LSM9DS1_SPI_TRANSACTION_LENGTH_16);
        handle->Gyro.ReadingRaw.Y = (spiTransaction.rx_data[1U] << 8U) | spiTransaction.rx_data[0U];

        /* Read Z */
        memset(&spiTransaction, 0U, sizeof(spiTransaction));
        Lsm9ds1_AccelGyroSpiRead(handle, &spiTransaction, LSM9DS1_OUT_Z_L_G, LSM9DS1_SPI_TRANSACTION_LENGTH_16);
        handle->Gyro.ReadingRaw.Z = (spiTransaction.rx_data[1U] << 8U) | spiTransaction.rx_data[0U];

        handle->Gyro.ReadingRaw.X -= handle->Gyro.BiasRaw.X;
        handle->Gyro.ReadingRaw.Y -= handle->Gyro.BiasRaw.Y;
        handle->Gyro.ReadingRaw.Z -= handle->Gyro.BiasRaw.Z;

        Lsm9ds1_GyroCalc(handle, handle->Gyro.ReadingRaw.X, &(handle->Gyro.Reading.X));
        Lsm9ds1_GyroCalc(handle, handle->Gyro.ReadingRaw.Y, &(handle->Gyro.Reading.Y));
        Lsm9ds1_GyroCalc(handle, handle->Gyro.ReadingRaw.Z, &(handle->Gyro.Reading.Z));
    }
}

void Lsm9ds1_MagRead(Lsm9ds1_Handle_t *const handle)
{
    if (handle != NULL)
    {
        /* TODO */

        handle->Mag.ReadingRaw.X -= handle->Mag.BiasRaw.X;
        handle->Mag.ReadingRaw.Y -= handle->Mag.BiasRaw.Y;
        handle->Mag.ReadingRaw.Z -= handle->Mag.BiasRaw.Z;

        Lsm9ds1_MagCalc(handle, handle->Mag.ReadingRaw.X, &(handle->Mag.Reading.X));
        Lsm9ds1_MagCalc(handle, handle->Mag.ReadingRaw.Y, &(handle->Mag.Reading.Y));
        Lsm9ds1_MagCalc(handle, handle->Mag.ReadingRaw.Z, &(handle->Mag.Reading.Z));
    }
}

void Lsm9ds1_TempRead(Lsm9ds1_Handle_t *const handle)
{
    if (handle != NULL)
    {
    }
}

static inline bool Lsm9ds1_AccelCalc(const Lsm9ds1_Handle_t *const handle, const Lsm9ds1_AxisRaw_t axisRaw, Lsm9ds1_AxisCalc_t *const result)
{
    bool success = false;

    if (handle != NULL && result != NULL)
    {
        *result = axisRaw * handle->Accel.Resolution;
        success = true;
    }

    return success;
}

static inline bool Lsm9ds1_GyroCalc(const Lsm9ds1_Handle_t *const handle, const Lsm9ds1_AxisRaw_t axisRaw, Lsm9ds1_AxisCalc_t *const result)
{
    bool success = false;

    if (handle != NULL && result != NULL)
    {
        *result = axisRaw * handle->Gyro.Resolution;
        success = true;
    }

    return success;
}

static inline bool Lsm9ds1_MagCalc(const Lsm9ds1_Handle_t *const handle, const Lsm9ds1_AxisRaw_t axisRaw, Lsm9ds1_AxisCalc_t *const result)
{
    bool success = false;

    if (handle != NULL && result != NULL)
    {
        *result = axisRaw * handle->Mag.Resolution;
        success = true;
    }

    return success;
}

/* TODO check for null pointers */
static inline esp_err_t Lsm9ds1_AccelGyroSpiTransmit(const Lsm9ds1_Handle_t *const handle, spi_transaction_t *const spiTranscation, const uint64_t address, const size_t length)
{
    esp_err_t err = ESP_OK;

    spiTranscation->addr = address & LSM9DS1_AG_ADDRESS_MASK;
    spiTranscation->length = length;

    spi_device_acquire_bus(handle->AccelGyroSpiHandle, portMAX_DELAY);
    err = spi_device_polling_transmit(handle->AccelGyroSpiHandle, spiTranscation);
    spi_device_release_bus(handle->AccelGyroSpiHandle);

    return err;
}

/* TODO check for null pointers */
static esp_err_t Lsm9ds1_AccelGyroSpiRead(const Lsm9ds1_Handle_t *const handle, spi_transaction_t *const spiTranscation, const uint64_t address, const size_t length)
{
    spiTranscation->cmd = LSM9DS1_AG_SPI_CMD_READ;
    spiTranscation->flags = SPI_TRANS_USE_RXDATA;

    return Lsm9ds1_AccelGyroSpiTransmit(handle, spiTranscation, address, length);
}

/* TODO check for null pointers */
static esp_err_t Lsm9ds1_AccelGyroSpiWrite(const Lsm9ds1_Handle_t *const handle, spi_transaction_t *const spiTranscation, const uint64_t address, const size_t length)
{
    spiTranscation->cmd = LSM9DS1_AG_SPI_CMD_WRITE;
    spiTranscation->flags = SPI_TRANS_USE_TXDATA;

    return Lsm9ds1_AccelGyroSpiTransmit(handle, spiTranscation, address, length);
}

static void Lsm9ds1_EnableFifo(const Lsm9ds1_Handle_t *const handle, const bool enable)
{
    if (handle != NULL)
    {
        spi_transaction_t spiTransaction;
        uint8_t registerValue;

        memset(&spiTransaction, 0U, sizeof(spiTransaction));
        Lsm9ds1_AccelGyroSpiRead(handle, &spiTransaction, LSM9DS1_CTRL_REG9, LSM9DS1_SPI_TRANSACTION_LENGTH_8);
        registerValue = spiTransaction.rx_data[0U];

        if (enable)
        {
            registerValue |= (1U << 1U);
        }
        else
        {
            registerValue &= ~(1U << 1U);
        }

        memset(&spiTransaction, 0U, sizeof(spiTransaction));
        spiTransaction.tx_data[0U] = registerValue;
        Lsm9ds1_AccelGyroSpiWrite(handle, &spiTransaction, LSM9DS1_CTRL_REG9, LSM9DS1_SPI_TRANSACTION_LENGTH_8);
    }
}

static void Lsm9ds1_SetFifo(const Lsm9ds1_Handle_t *const handle, const Lsm9ds1_FifoMode_t fifoMode, const Lsm9ds1_FifoThreshold_t fifoThreshold)
{
    if (handle != NULL)
    {
        spi_transaction_t spiTransaction;
        uint8_t registerValue = (fifoMode << 5U);

        if (fifoThreshold > 0x1FU)
        {
            registerValue |= 0x1FU;
        }
        else
        {
            registerValue |= fifoThreshold;
        }

        memset(&spiTransaction, 0U, sizeof(spiTransaction));
        spiTransaction.tx_data[0U] = registerValue;
        Lsm9ds1_AccelGyroSpiWrite(handle, &spiTransaction, LSM9DS1_FIFO_CTRL, LSM9DS1_SPI_TRANSACTION_LENGTH_8);
    }
}

/* TODO check for null pointers */
static inline esp_err_t Lsm9ds1_MagSpiTransmit(const Lsm9ds1_Handle_t *const handle, spi_transaction_t *const spiTranscation, const uint64_t address, const size_t length)
{
    esp_err_t err = ESP_OK;

    spiTranscation->addr = address & LSM9DS1_M_ADDRESS_MASK;
    spiTranscation->length = length;

    spi_device_acquire_bus(handle->MagSpiHandle, portMAX_DELAY);
    err = spi_device_polling_transmit(handle->MagSpiHandle, spiTranscation);
    spi_device_release_bus(handle->MagSpiHandle);

    return err;
}

/* TODO check for null pointers */
static esp_err_t Lsm9ds1_MagSpiRead(const Lsm9ds1_Handle_t *const handle, spi_transaction_t *const spiTranscation, const bool ms, const uint64_t address, const size_t length)
{
    spiTranscation->cmd = LSM9DS1_M_SPI_CMD_READ,
    spiTranscation->flags = SPI_TRANS_USE_RXDATA;

    if (ms)
    {
        spiTranscation->cmd |= LSM9DS1_M_SPI_CMD_MS;
    }

    return Lsm9ds1_MagSpiTransmit(handle, spiTranscation, address, length);
}

/* TODO check for null pointers */
static esp_err_t Lsm9ds1_MagSpiWrite(const Lsm9ds1_Handle_t *const handle, spi_transaction_t *const spiTranscation, const bool ms, const uint64_t address, const size_t length)
{
    spiTranscation->cmd = LSM9DS1_M_SPI_CMD_WRITE,
    spiTranscation->flags = SPI_TRANS_USE_TXDATA;

    if (ms)
    {
        spiTranscation->cmd |= LSM9DS1_M_SPI_CMD_MS;
    }

    return Lsm9ds1_MagSpiTransmit(handle, spiTranscation, address, length);
}
