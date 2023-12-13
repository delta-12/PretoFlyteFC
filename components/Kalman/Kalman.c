/**
 * @file Kalman.h
 *
 * @brief Kalman filter implementation.
 *
 ******************************************************************************/

/* Includes
 ******************************************************************************/
#include "Kalman.h"
#include <stddef.h>

/* Function Definitions
 ******************************************************************************/

void Kalman_1dFilterInit(Kalman_1dFilterContext_t *context, const Kalman_State_t state, const Kalman_Uncertainty_t uncertainty, const Kalman_RateVariation_t rateVariation, const Kalman_MeasurementVariation_t measurementVariation)
{
    if (context != NULL)
    {
        context->State = state;
        context->Uncertainty = uncertainty;
        context->RateVariation = rateVariation;
        context->MeasurementVariation = measurementVariation;
    }
}

void Kalman_1dFilter(Kalman_1dFilterContext_t *const context, const Kalman_Rate_t rate, const Kalman_Measurement_t measurement, const Kalman_SampleTimeSeconds_t sampleTime)
{
    if (context != NULL)
    {
        /* Predict current state */
        // KalmanState = KalmanState + 0.004 * KalmanInput;
        context->State += sampleTime * rate;

        /* Calculate uncertainty of prediction */
        // KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
        context->Uncertainty += (sampleTime * sampleTime) + (context->RateVariation * context->RateVariation);

        /* Calculate Kalman gain */
        // float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3);
        double kalmanGain = context->Uncertainty / (context->Uncertainty + (context->MeasurementVariation * context->MeasurementVariation));

        /* Update prediction with measurement through the Kalman gain */
        // KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
        context->State += kalmanGain * (measurement - context->State);

        /* Update uncertainty of prediction */
        // KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
        context->Uncertainty = (1 - kalmanGain) * context->Uncertainty;
    }
}