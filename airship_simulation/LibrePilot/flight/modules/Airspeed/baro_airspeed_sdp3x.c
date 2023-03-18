/**
 ******************************************************************************
 * @addtogroup OpenPilotModules OpenPilot Modules
 * @{
 * @addtogroup AirspeedModule Airspeed Module
 * @brief Communicate with airspeed sensors and return values
 * @{
 *
 * @file       baro_airspeed.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      Airspeed module, handles temperature and pressure readings from BMP085
 *
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

/**
 * Output object: BaroAirspeed
 *
 * This module will periodically update the value of the BaroAirspeed object.
 *
 */

#include "openpilot.h"
#include "hwsettings.h"
#include "airspeedsettings.h"
#include "airspeedsensor.h" // object that will be updated by the module
#include "airspeedalarm.h"

#if defined(PIOS_INCLUDE_SDP3X)

#define CALIBRATION_IDLE_MS  2000   // Time to wait before calibrating, in [ms]
#define CALIBRATION_COUNT_MS 2000 // Time to spend calibrating, in [ms]

#define P0                   101325.0f           // standard pressure
#define CCEXPONENT           0.2857142857f       // exponent of compressibility correction 2/7
#define CASFACTOR            760.8802669f        // sqrt(5) * speed of sound at standard
#define TASFACTOR            0.05891022589f      // 1/sqrt(T0)

// Private types

// Private variables

// Private functions

static volatile struct PIOS_SDP3X_STATE state = { .status = SDP3X_STATUS_NOTFOUND, .pressure = 0, .temperature = 0 };

void baro_airspeedGetSDP3X(AirspeedSensorData *airspeedSensor, AirspeedSettingsData *airspeedSettings)
{
    // Check to see if airspeed sensor is returning airspeedSensor
    PIOS_SDP3X_ReadAirspeed(&state);
    airspeedSensor->SensorValue = state.pressure;
    airspeedSensor->SensorValueTemperature = state.temperature;

    if (state.status != SDP3X_STATUS_READY) {
        airspeedSensor->SensorConnected    = AIRSPEEDSENSOR_SENSORCONNECTED_FALSE;
        airspeedSensor->CalibratedAirspeed = 0;
        AirspeedAlarm(SYSTEMALARMS_ALARM_ERROR);
        // At boot time, sensor needs time for self calibration during which it may not be disturbed. Wait 2 seconds before next attempt
        vTaskDelay(2000 / portTICK_RATE_MS);
        return;
    }

    // No calibration, sensor comes factory calibrated!

    // Compute airspeed
    airspeedSensor->Temperature          = ((float)state.temperature / 200.0f) + 273.15f; // convert to kelvin
    airspeedSensor->DifferentialPressure = (1.0f / (((float)state.scale) + 1e-9f)) * ((float)(state.pressure) - (float)((int16_t)(airspeedSettings->ZeroPoint)));
    airspeedSensor->CalibratedAirspeed   = airspeedSettings->Scale * CASFACTOR * sqrtf(powf(fabsf(airspeedSensor->DifferentialPressure) / P0 + 1.0f, CCEXPONENT) - 1.0f);
    airspeedSensor->TrueAirspeed         = airspeedSensor->CalibratedAirspeed * TASFACTOR * sqrtf(airspeedSensor->Temperature);

    airspeedSensor->SensorConnected      = AIRSPEEDSENSOR_SENSORCONNECTED_TRUE;
    AirspeedAlarm(SYSTEMALARMS_ALARM_OK);
}


#endif /* if defined(PIOS_INCLUDE_SDP3X) */

/**
 * @}
 * @}
 */
