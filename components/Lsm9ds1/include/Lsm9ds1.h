/**
 * @file Lsm9ds1.h
 *
 * @brief Lsm9ds1 driver for the ESP32.
 *
 ******************************************************************************/

#ifndef LSM9DS1_H
#define LSM9DS1_H

/* Includes
 ******************************************************************************/
#include "driver/spi_master.h"

/* Defines
 ******************************************************************************/

/* Typedefs
 ******************************************************************************/

typedef uint8_t Lsm9ds1_SpiHost_t;
typedef uint32_t Lsm9ds1_SpiPin_t;

typedef struct
{
    Lsm9ds1_SpiPin_t Mosi;
    Lsm9ds1_SpiPin_t Miso;
    Lsm9ds1_SpiPin_t Clk;
    Lsm9ds1_SpiPin_t CsAccelGyro;
    Lsm9ds1_SpiPin_t CsMag;
    Lsm9ds1_SpiPin_t Int1;
    Lsm9ds1_SpiPin_t Int2;
    Lsm9ds1_SpiPin_t IntM;
    Lsm9ds1_SpiPin_t Rdy;
} Lsm9ds1_PinConfig_t;

typedef struct
{
    spi_device_handle_t AccelGyroSpiHandle;
    spi_device_handle_t MagSpiHandle;
} Lsm9ds1_Handle_t;

/* Function Prototypes
 ******************************************************************************/

bool Lsm9ds1_Init(Lsm9ds1_Handle_t *const handle, const Lsm9ds1_SpiHost_t spiHost, const Lsm9ds1_PinConfig_t *const pinConfig);
bool Lsm9ds1_WhoAmI(const Lsm9ds1_Handle_t *const handle);

#endif