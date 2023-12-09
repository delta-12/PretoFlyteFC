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

/* Defines
 ******************************************************************************/

#define LSM9DS1_SPI_GPIO_UNUSED (-1)        /* Unused GPIO in SPI config should be set to -1 */
#define LSM9DS1_SPI_MAX_TRANSFER_SZ 0U      /* Maximum size of SPI transfer, 0 defaults to 4092 when DMA is enabled */
#define LSM9DS1_SPI_CLK_FREQ_HZ 10000000U   /* 10MHz SPI clock frequency */
#define LSM9DS1_SPI_MODE_0 0U               /* CPOL = 0, CPHA = 0 */
#define LSM9DS1_SPI_QUEUE_SIZE 10U          /* Depth of 10 items for SPI queues */
#define LSM9DS1_SPI_TRANSACTION_LENGTH_8 8U /* SPI transaction 8 bits in length */

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

#define LSM9DS1_AG_WHO_AM_I 0x0FU
#define LSM9DS1_AG_WHO_AM_I_RSP 0x68U

#define LSM9DS1_M_WHO_AM_I 0x0FU
#define LSM9DS1_M_WHO_AM_RSP 0x3D

/* Globals
 ******************************************************************************/

static const char *Lsm9ds1_LogTag = "LSM9DS1";

/* Function Prototypes
 ******************************************************************************/

static inline esp_err_t Lsm9ds1_AccelGyroTransmit(const Lsm9ds1_Handle_t *const handle, spi_transaction_t *const spiTranscation, const uint64_t address, const size_t length);
static esp_err_t Lsm9ds1_AccelGyroRead(const Lsm9ds1_Handle_t *const handle, spi_transaction_t *const spiTranscation, const uint64_t address, const size_t length);
static esp_err_t Lsm9ds1_AccelGyroWrite(const Lsm9ds1_Handle_t *const handle, spi_transaction_t *const spiTranscation, const uint64_t address, const size_t length);
static inline esp_err_t Lsm9ds1_MagTransmit(const Lsm9ds1_Handle_t *const handle, spi_transaction_t *const spiTranscation, const uint64_t address, const size_t length);
static esp_err_t Lsm9ds1_MagRead(const Lsm9ds1_Handle_t *const handle, spi_transaction_t *const spiTranscation, const bool ms, const uint64_t address, const size_t length);
static esp_err_t Lsm9ds1_MagWrite(const Lsm9ds1_Handle_t *const handle, spi_transaction_t *const spiTranscation, const bool ms, const uint64_t address, const size_t length);

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
    }

    return success;
}

bool Lsm9ds1_WhoAmI(const Lsm9ds1_Handle_t *const handle)
{
    bool success = false;
    spi_transaction_t spiTransactionAccelGyro = {0U};
    spi_transaction_t spiTransactionMag = {0U};

    if (Lsm9ds1_AccelGyroRead(handle, &spiTransactionAccelGyro, LSM9DS1_AG_WHO_AM_I, LSM9DS1_SPI_TRANSACTION_LENGTH_8) == ESP_OK &&
        Lsm9ds1_MagRead(handle, &spiTransactionMag, false, LSM9DS1_AG_WHO_AM_I, LSM9DS1_SPI_TRANSACTION_LENGTH_8) == ESP_OK)
    {
        /* TODO remove magic numbers */
        success = (((spiTransactionAccelGyro.rx_data[0U] << 8U) | spiTransactionMag.rx_data[0U]) == ((LSM9DS1_AG_WHO_AM_I_RSP << 8) | LSM9DS1_M_WHO_AM_RSP));
    }

    return success;
}

static inline esp_err_t Lsm9ds1_AccelGyroTransmit(const Lsm9ds1_Handle_t *const handle, spi_transaction_t *const spiTranscation, const uint64_t address, const size_t length)
{
    esp_err_t err = ESP_OK;

    spiTranscation->addr = address & LSM9DS1_AG_ADDRESS_MASK;
    spiTranscation->length = length;

    spi_device_acquire_bus(handle->AccelGyroSpiHandle, portMAX_DELAY);
    err = spi_device_polling_transmit(handle->AccelGyroSpiHandle, spiTranscation);
    spi_device_release_bus(handle->AccelGyroSpiHandle);

    return err;
}

static esp_err_t Lsm9ds1_AccelGyroRead(const Lsm9ds1_Handle_t *const handle, spi_transaction_t *const spiTranscation, const uint64_t address, const size_t length)
{
    spiTranscation->cmd = LSM9DS1_AG_SPI_CMD_READ;
    spiTranscation->flags = SPI_TRANS_USE_RXDATA;

    return Lsm9ds1_AccelGyroTransmit(handle, spiTranscation, address, length);
}

static esp_err_t Lsm9ds1_AccelGyroWrite(const Lsm9ds1_Handle_t *const handle, spi_transaction_t *const spiTranscation, const uint64_t address, const size_t length)
{
    spiTranscation->cmd = LSM9DS1_AG_SPI_CMD_WRITE;
    spiTranscation->flags = SPI_TRANS_USE_TXDATA;

    return Lsm9ds1_AccelGyroTransmit(handle, spiTranscation, address, length);
}

static inline esp_err_t Lsm9ds1_MagTransmit(const Lsm9ds1_Handle_t *const handle, spi_transaction_t *const spiTranscation, const uint64_t address, const size_t length)
{
    esp_err_t err = ESP_OK;

    spiTranscation->addr = address & LSM9DS1_M_ADDRESS_MASK;
    spiTranscation->length = length;

    spi_device_acquire_bus(handle->MagSpiHandle, portMAX_DELAY);
    err = spi_device_polling_transmit(handle->MagSpiHandle, spiTranscation);
    spi_device_release_bus(handle->MagSpiHandle);

    return err;
}

static esp_err_t Lsm9ds1_MagRead(const Lsm9ds1_Handle_t *const handle, spi_transaction_t *const spiTranscation, const bool ms, const uint64_t address, const size_t length)
{
    spiTranscation->cmd = LSM9DS1_M_SPI_CMD_READ,
    spiTranscation->flags = SPI_TRANS_USE_RXDATA;

    if (ms)
    {
        spiTranscation->cmd |= LSM9DS1_M_SPI_CMD_MS;
    }

    return Lsm9ds1_MagTransmit(handle, spiTranscation, address, length);
}

static esp_err_t Lsm9ds1_MagWrite(const Lsm9ds1_Handle_t *const handle, spi_transaction_t *const spiTranscation, const bool ms, const uint64_t address, const size_t length)
{
    spiTranscation->cmd = LSM9DS1_M_SPI_CMD_WRITE,
    spiTranscation->flags = SPI_TRANS_USE_TXDATA;

    if (ms)
    {
        spiTranscation->cmd |= LSM9DS1_M_SPI_CMD_MS;
    }

    return Lsm9ds1_MagTransmit(handle, spiTranscation, address, length);
}
