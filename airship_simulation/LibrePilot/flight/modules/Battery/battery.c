/**
 ******************************************************************************
 * @addtogroup OpenPilotModules OpenPilot Modules
 * @{
 * @addtogroup BatteryModule Battery Module
 * @brief Measures battery voltage and current
 * Updates the FlightBatteryState object
 * @{
 *
 * @file       battery.c
 * @author     The LibrePilot Project, http://www.librepilot.org Copyright (C) 2016.
 *             The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      Module to read the battery Voltage and Current periodically and set alarms appropriately.
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
 * Output object: FlightBatteryState
 *
 * This module will periodically generate information on the battery state.
 *
 * UAVObjects are automatically generated by the UAVObjectGenerator from
 * the object definition XML file.
 *
 * Modules have no API, all communication to other modules is done through UAVObjects.
 * However modules may use the API exposed by shared libraries.
 * See the OpenPilot wiki for more details.
 * http://www.openpilot.org/OpenPilot_Application_Architecture
 *
 */

#include "openpilot.h"

#include "flightstatus.h"
#include "flightbatterystate.h"
#include "flightbatterysettings.h"
#include "hwsettings.h"
#include "systemstats.h"

//
// Configuration
//
#define SAMPLE_PERIOD_MS     500

// Time since power on the cells detection is active
#define DETECTION_TIMEFRAME  60000

#ifndef PIOS_ADC_VOLTAGE_PIN
#define PIOS_ADC_VOLTAGE_PIN -1
#endif

#ifndef PIOS_ADC_CURRENT_PIN
#define PIOS_ADC_CURRENT_PIN -1
#endif


// THESE COULD BE BETTER AS SOME KIND OF UNION OR STRUCT, BY WHICH 4 BITS ARE USED FOR EACH
// PIN VARIABLE, ONE OF WHICH INDICATES SIGN, AND THE OTHER 3 BITS INDICATE POSITION. THIS WILL
// WORK FOR QUITE SOMETIME, UNTIL MORE THAN 8 ADC ARE AVAILABLE. EVEN AT THIS POINT, THE STRUCTURE
// CAN SIMPLY BE MODIFIED TO SUPPORT 15 ADC PINS, BY USING ALL AVAILABLE BITS.
static int8_t voltageADCPin = PIOS_ADC_VOLTAGE_PIN; // ADC pin for voltage
static int8_t currentADCPin = PIOS_ADC_CURRENT_PIN; // ADC pin for current

// Private functions
static void onTimer(UAVObjEvent *ev);
static void GetNbCells(const FlightBatterySettingsData *batterySettings, FlightBatteryStateData *flightBatteryData);

/**
 * Initialise the module, called on startup
 * \returns 0 on success or -1 if initialisation failed
 */
int32_t BatteryInitialize(void)
{
    bool batteryEnabled;

    HwSettingsOptionalModulesData optionalModules;

    HwSettingsOptionalModulesGet(&optionalModules);

#ifdef MODULE_BATTERY_BUILTIN
    batteryEnabled = true;
    optionalModules.Battery = HWSETTINGS_OPTIONALMODULES_ENABLED;
    HwSettingsOptionalModulesSet(&optionalModules);
#else
    if (optionalModules.Battery == HWSETTINGS_OPTIONALMODULES_ENABLED) {
        batteryEnabled = true;
    } else {
        batteryEnabled = false;
    }
#endif

    uint8_t adcRouting[HWSETTINGS_ADCROUTING_NUMELEM];
    HwSettingsADCRoutingArrayGet(adcRouting);

    // Determine if the battery sensors are routed to ADC pins
    for (int i = 0; i < HWSETTINGS_ADCROUTING_NUMELEM; i++) {
        if (adcRouting[i] == HWSETTINGS_ADCROUTING_BATTERYVOLTAGE) {
            voltageADCPin = i;
        }
        if (adcRouting[i] == HWSETTINGS_ADCROUTING_BATTERYCURRENT) {
            currentADCPin = i;
        }
    }

    // Don't enable module if no ADC pins are routed to the sensors
    if (voltageADCPin < 0 && currentADCPin < 0) {
        batteryEnabled = false;
    }

    // Start module
    if (batteryEnabled) {
        FlightBatteryStateInitialize();
        SystemStatsInitialize();

        static UAVObjEvent ev;

        memset(&ev, 0, sizeof(UAVObjEvent));
        EventPeriodicCallbackCreate(&ev, onTimer, SAMPLE_PERIOD_MS / portTICK_RATE_MS);
    }

    return 0;
}

MODULE_INITCALL(BatteryInitialize, 0);
static void onTimer(__attribute__((unused)) UAVObjEvent *ev)
{
    static FlightBatterySettingsData batterySettings;
    static FlightBatteryStateData flightBatteryData;

    FlightBatterySettingsGet(&batterySettings);
    FlightBatteryStateGet(&flightBatteryData);

    const float dT = SAMPLE_PERIOD_MS / 1000.0f;
    float energyRemaining;

    // Reset ConsumedEnergy counter
    if (batterySettings.ResetConsumedEnergy) {
        flightBatteryData.ConsumedEnergy    = 0;
        batterySettings.ResetConsumedEnergy = false;
        FlightBatterySettingsSet(&batterySettings);
    }
#ifdef PIOS_INCLUDE_ADC
    // calculate the battery parameters
    if (voltageADCPin >= 0) {
        flightBatteryData.Voltage = (PIOS_ADC_PinGetVolt(voltageADCPin) - batterySettings.SensorCalibrations.VoltageZero) * batterySettings.SensorCalibrations.VoltageFactor; // in Volts
    } else {
        flightBatteryData.Voltage = 0; // Dummy placeholder value. This is in case we get another source of battery current which is not from the ADC
    }
#else
    flightBatteryData.Voltage = 0;
#endif /* PIOS_INCLUDE_ADC */
    // voltage available: get the number of cells if possible, desired and not armed
    GetNbCells(&batterySettings, &flightBatteryData);
#ifdef PIOS_INCLUDE_ADC
    // ad a plausibility check: zero voltage => zero current
    if (currentADCPin >= 0 && flightBatteryData.Voltage > 0.f) {
        flightBatteryData.Current = (PIOS_ADC_PinGetVolt(currentADCPin) - batterySettings.SensorCalibrations.CurrentZero) * batterySettings.SensorCalibrations.CurrentFactor; // in Amps
        if (flightBatteryData.Current > flightBatteryData.PeakCurrent) {
            flightBatteryData.PeakCurrent = flightBatteryData.Current; // in Amps
        }
    } else { // If there's no current measurement, we still need to assign one. Make it negative, so it can never trigger an alarm
        flightBatteryData.Current = -0; // Dummy placeholder value. This is in case we get another source of battery current which is not from the ADC
    }
#else
    flightBatteryData.Current = -0;
#endif /* PIOS_INCLUDE_ADC */

    // For safety reasons consider only positive currents in energy comsumption, i.e. no charging up.
    // necesary when sensor are not perfectly calibrated
    if (flightBatteryData.Current > 0) {
        flightBatteryData.ConsumedEnergy += (flightBatteryData.Current * dT * 1000.0f / 3600.0f); // in mAh
    }

    // Apply a 2 second rise time low-pass filter to average the current
    float alpha = 1.0f - dT / (dT + 2.0f);
    flightBatteryData.AvgCurrent = alpha * flightBatteryData.AvgCurrent + (1 - alpha) * flightBatteryData.Current; // in Amps

    /*The motor could regenerate power. Or we could have solar cells.
       In short, is there any likelihood of measuring negative current? If it's a bad current reading we want to check, then
       it makes sense to saturate at max and min values, because a misreading could as easily be very large, as negative. The simple
       sign check doesn't catch this.*/
    energyRemaining = batterySettings.Capacity - flightBatteryData.ConsumedEnergy; // in mAh
    if (batterySettings.Capacity > 0 && flightBatteryData.AvgCurrent > 0) {
        flightBatteryData.EstimatedFlightTime = (energyRemaining / (flightBatteryData.AvgCurrent * 1000.0f)) * 3600.0f; // in Sec
    } else {
        flightBatteryData.EstimatedFlightTime = 0;
    }

    // generate alarms where needed...
    if ((flightBatteryData.Voltage <= 0) && (flightBatteryData.Current <= 0)) {
        // FIXME: There's no guarantee that a floating ADC will give 0. So this
        // check might fail, even when there's nothing attached.
        AlarmsSet(SYSTEMALARMS_ALARM_BATTERY, SYSTEMALARMS_ALARM_ERROR);
        AlarmsSet(SYSTEMALARMS_ALARM_FLIGHTTIME, SYSTEMALARMS_ALARM_ERROR);
    } else {
        // FIXME: should make the timer alarms user configurable
        if (batterySettings.Capacity > 0 && flightBatteryData.EstimatedFlightTime < 30) {
            AlarmsSet(SYSTEMALARMS_ALARM_FLIGHTTIME, SYSTEMALARMS_ALARM_CRITICAL);
        } else if (batterySettings.Capacity > 0 && flightBatteryData.EstimatedFlightTime < 120) {
            AlarmsSet(SYSTEMALARMS_ALARM_FLIGHTTIME, SYSTEMALARMS_ALARM_WARNING);
        } else {
            AlarmsClear(SYSTEMALARMS_ALARM_FLIGHTTIME);
        }

        // FIXME: should make the battery voltage detection dependent on battery type.
        /*Not so sure. Some users will want to run their batteries harder than others, so it should be the user's choice. [KDS]*/
        if (flightBatteryData.Voltage < batterySettings.CellVoltageThresholds.Critical * flightBatteryData.NbCells) {
            AlarmsSet(SYSTEMALARMS_ALARM_BATTERY, SYSTEMALARMS_ALARM_CRITICAL);
        } else if (flightBatteryData.Voltage < batterySettings.CellVoltageThresholds.Warning * flightBatteryData.NbCells) {
            AlarmsSet(SYSTEMALARMS_ALARM_BATTERY, SYSTEMALARMS_ALARM_WARNING);
        } else {
            AlarmsClear(SYSTEMALARMS_ALARM_BATTERY);
        }
    }

    FlightBatteryStateSet(&flightBatteryData);
}


static void GetNbCells(const FlightBatterySettingsData *batterySettings, FlightBatteryStateData *flightBatteryData)
{
    // get flight status to check for armed
    uint8_t armed = 0;
    static bool detected = false;

    // prevent the cell number to change once the board is armed at least once
    if (detected) {
        return;
    }

    FlightStatusArmedGet(&armed);

    // check only if not armed
    if (armed == FLIGHTSTATUS_ARMED_ARMED) {
        detected = true;
        return;
    }

    // prescribed number of cells?
    if (batterySettings->NbCells != 0) {
        flightBatteryData->NbCells = batterySettings->NbCells;
        flightBatteryData->NbCellsAutodetected = 0;
        return;
    }

    // plausibility check
    if (flightBatteryData->Voltage <= 0.5f) {
        // cannot detect number of cells
        flightBatteryData->NbCellsAutodetected = 0;
        return;
    }

    float voltageMin = 0.f, voltageMax = 0.f;

    // Cell type specific values
    // TODO: could be implemented as constant arrays indexed by cellType
    // or could be part of the UAVObject definition
    switch (batterySettings->Type) {
    case FLIGHTBATTERYSETTINGS_TYPE_LIPO:
    case FLIGHTBATTERYSETTINGS_TYPE_LICO:
        voltageMin = 3.6f;
        voltageMax = 4.2f;
        break;
    case FLIGHTBATTERYSETTINGS_TYPE_LIHV:
        voltageMin = 3.6f;
        voltageMax = 4.35f;
        break;
    case FLIGHTBATTERYSETTINGS_TYPE_A123:
        voltageMin = 2.01f;
        voltageMax = 3.59f;
        break;
    case FLIGHTBATTERYSETTINGS_TYPE_LIFESO4:
    default:
        flightBatteryData->NbCellsAutodetected = 0;
        return;
    }

    // uniquely measurable under any condition iff n * voltageMax < (n+1) * voltageMin
    // or n < voltageMin / (voltageMax-voltageMin)
    // weaken condition by setting n <= voltageMin / (voltageMax-voltageMin) and
    // checking for v <= voltageMin * voltageMax / (voltageMax-voltageMin)
    if (flightBatteryData->Voltage > voltageMin * voltageMax / (voltageMax - voltageMin)) {
        flightBatteryData->NbCellsAutodetected = 0;
        return;
    }

    // Prevent the battery discharging on the ground to change the detected number of cells:
    // Detection is enabled in the first 60 seconds from powerup
    uint32_t flightTime;
    SystemStatsFlightTimeGet(&flightTime);
    if (flightTime > DETECTION_TIMEFRAME) {
        detected = true;
    }

    flightBatteryData->NbCells = (int8_t)(flightBatteryData->Voltage / voltageMin);
    flightBatteryData->NbCellsAutodetected = 1;
}

/**
 * @}
 */

/**
 * @}
 */
