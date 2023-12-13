/**
 * @file Kalman.h
 *
 * @brief Kalman filter implementation.
 *
 ******************************************************************************/

#ifndef KALMAN_H
#define KALMAN_H

/* Typedefs
 ******************************************************************************/

typedef double Kalman_State_t;
typedef double Kalman_Uncertainty_t;
typedef double Kalman_Rate_t;
typedef double Kalman_Measurement_t;
typedef double Kalman_RateVariation_t;
typedef double Kalman_MeasurementVariation_t;
typedef double Kalman_SampleTimeSeconds_t;

typedef struct
{
    Kalman_State_t State;
    Kalman_Uncertainty_t Uncertainty;
    Kalman_RateVariation_t RateVariation;
    Kalman_MeasurementVariation_t MeasurementVariation;
} Kalman_1dFilterContext_t;

/* Function Prototypes
 ******************************************************************************/

void Kalman_1dFilterInit(Kalman_1dFilterContext_t *context, const Kalman_State_t state, const Kalman_Uncertainty_t uncertainty, const Kalman_RateVariation_t rateVariation, const Kalman_MeasurementVariation_t measurementVariation);
void Kalman_1dFilter(Kalman_1dFilterContext_t *const context, const Kalman_Rate_t rate, const Kalman_Measurement_t measurement, const Kalman_SampleTimeSeconds_t sampleTime);

#endif