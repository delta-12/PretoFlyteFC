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

/* Typedefs
 ******************************************************************************/

typedef uint8_t Lsm9ds1_SpiHost_t;
typedef uint32_t Lsm9ds1_SpiPin_t;
typedef int16_t Lsm9ds1_AxisRaw_t;
typedef double Lsm9ds1_AxisCalc_t;
typedef double Lsm9ds1_SensorResolution_t;
typedef int16_t Lsm9ds1_Temperature_t;

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
    Lsm9ds1_AxisRaw_t X;
    Lsm9ds1_AxisRaw_t Y;
    Lsm9ds1_AxisRaw_t Z;
} Lsm9ds1_SensorReadingRaw_t;

typedef struct
{
    Lsm9ds1_AxisRaw_t X;
    Lsm9ds1_AxisRaw_t Y;
    Lsm9ds1_AxisRaw_t Z;
} Lsm9ds1_SensorBiasRaw_t;

typedef struct
{
    Lsm9ds1_AxisCalc_t X;
    Lsm9ds1_AxisCalc_t Y;
    Lsm9ds1_AxisCalc_t Z;
} Lsm9ds1_SensorReading_t;

typedef struct
{
    Lsm9ds1_AxisCalc_t X;
    Lsm9ds1_AxisCalc_t Y;
    Lsm9ds1_AxisCalc_t Z;
} Lsm9ds1_SensorBias_t;

typedef struct
{
    Lsm9ds1_SensorReadingRaw_t ReadingRaw;
    Lsm9ds1_SensorBiasRaw_t BiasRaw;
    Lsm9ds1_SensorReading_t Reading;
    Lsm9ds1_SensorBias_t Bias;
    Lsm9ds1_SensorResolution_t Resolution;
} Lsm9ds1_Sensor_t;

typedef struct
{
    spi_device_handle_t AccelGyroSpiHandle;
    spi_device_handle_t MagSpiHandle;
    Lsm9ds1_Sensor_t Accel;
    Lsm9ds1_Sensor_t Gyro;
    Lsm9ds1_Sensor_t Mag;
    Lsm9ds1_Temperature_t Temperature;
} Lsm9ds1_Handle_t;

/* Function Prototypes
 ******************************************************************************/

bool Lsm9ds1_Init(Lsm9ds1_Handle_t *const handle, const Lsm9ds1_SpiHost_t spiHost, const Lsm9ds1_PinConfig_t *const pinConfig);
bool Lsm9ds1_WhoAmI(const Lsm9ds1_Handle_t *const handle);
void Lsm9ds1_AccelInit(Lsm9ds1_Handle_t *const handle);
void Lsm9ds1_GyroInit(Lsm9ds1_Handle_t *const handle);
void Lsm9ds1_MagInit(Lsm9ds1_Handle_t *const handle);
void Lsm9ds1_AccelGyroCalibrate(Lsm9ds1_Handle_t *const handle);
void Lsm9ds1_MagCalibrate(Lsm9ds1_Handle_t *const handle);
bool Lsm9ds1_AccelAvailable(const Lsm9ds1_Handle_t *const handle);
bool Lsm9ds1_GyroAvailable(const Lsm9ds1_Handle_t *const handle);
bool Lsm9ds1_MagAvailable(const Lsm9ds1_Handle_t *const handle);
bool Lsm9ds1_TempAvailable(const Lsm9ds1_Handle_t *const handle);
void Lsm9ds1_AccelRead(Lsm9ds1_Handle_t *const handle);
void Lsm9ds1_GyroRead(Lsm9ds1_Handle_t *const handle);
void Lsm9ds1_MagRead(Lsm9ds1_Handle_t *const handle);
void Lsm9ds1_TempRead(Lsm9ds1_Handle_t *const handle);

#endif