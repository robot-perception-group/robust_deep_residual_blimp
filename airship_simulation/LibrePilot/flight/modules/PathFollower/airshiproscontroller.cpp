/*
 ******************************************************************************
 *
 * @file       AirshipROSController.cpp
 * @author     The LibrePilot Project, http://www.librepilot.org Copyright (C) 2016.
 *             The OpenPilot Team, http://www.openpilot.org Copyright (C) 2015.
 * @brief      Fixed wing fly controller implementation
 * @see        The GNU Public License (GPL) Version 3
 *
 * @addtogroup LibrePilot LibrePilotModules Modules PathFollower Navigation
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

extern "C" {
#include <openpilot.h>

#include <pid.h>
#include <sin_lookup.h>
#include <pathdesired.h>
#include <paths.h>
#include <airshippathfollowersettings.h>
#include <fixedwingpathfollowerstatus.h>
#include <flightstatus.h>
#include <pathstatus.h>
#include <positionstate.h>
#include <velocitystate.h>
#include <velocitydesired.h>
#include <stabilizationdesired.h>
#include <airspeedstate.h>
#include <attitudestate.h>
#include <systemsettings.h>
}

// C++ includes
#include "airshiproscontroller.h"

// Private constants

// pointer to a singleton instance
AirshipROSController *AirshipROSController::p_inst = 0;

AirshipROSController::AirshipROSController()
    : airshipSettings(NULL), mActive(false), mMode(0), indicatedAirspeedStateBias(0.0f), previousCourseError(0.0f)
{}

// Called when mode first engaged
void AirshipROSController::Activate(void)
{
    if (!mActive) {
        mActive = true;
        SettingsUpdated();
        resetGlobals();
        mMode   = pathDesired->Mode;
        lastAirspeedUpdate = 0;
    }
}

uint8_t AirshipROSController::IsActive(void)
{
    return mActive;
}

uint8_t AirshipROSController::Mode(void)
{
    return mMode;
}

// Objective updated in pathdesired
void AirshipROSController::ObjectiveUpdated(void)
{}

void AirshipROSController::Deactivate(void)
{
    if (mActive) {
        mActive = false;
        resetGlobals();
    }
}


void AirshipROSController::SettingsUpdated(void)
{
    pid_configure(&PIDalt, airshipSettings->AltPI.Kp, airshipSettings->AltPI.Ki, 0.0f, airshipSettings->AltPI.ILimit);
    pid_configure(&PIDpower, airshipSettings->PowerPI.Kp, airshipSettings->PowerPI.Ki, 0.0f, airshipSettings->PowerPI.ILimit);
}

/**
 * Initialise the module, called on startup
 * \returns 0 on success or -1 if initialisation failed
 */
int32_t AirshipROSController::Initialize(AirshipPathFollowerSettingsData *ptr_airshipSettings)
{
    PIOS_Assert(ptr_airshipSettings);

    airshipSettings = ptr_airshipSettings;

    resetGlobals();

    return 0;
}

/**
 * reset integrals
 */
void AirshipROSController::resetGlobals()
{
    pid_zero(&PIDalt);
    pid_zero(&PIDpower);
    pathStatus->path_time = 0.0f;
}

void AirshipROSController::UpdateAutoPilot()
{
    uint8_t result = updateAutoPilotAirship();

    if (result) {
        AlarmsSet(SYSTEMALARMS_ALARM_GUIDANCE, SYSTEMALARMS_ALARM_OK);
    } else {
        pathStatus->Status = PATHSTATUS_STATUS_CRITICAL;
        AlarmsSet(SYSTEMALARMS_ALARM_GUIDANCE, SYSTEMALARMS_ALARM_WARNING);
    }

    PathStatusSet(pathStatus);
}

/**
 * ros autopilot for airship
 */
uint8_t AirshipROSController::updateAutoPilotAirship()
{
    uint8_t result = 1;
    bool cutThrust = false;

    const float dT = airshipSettings->UpdatePeriod / 1000.0f;

    VelocityDesiredData velocityDesired;
    VelocityStateData velocityState;
    StabilizationDesiredData stabDesired;
    AttitudeStateData attitudeState;
    FixedWingPathFollowerStatusData fixedWingPathFollowerStatus;
    SystemSettingsData systemSettings;

    float groundspeedProjection;
    float indicatedAirspeedState;
    float indicatedAirspeedDesired;
    float airspeedError = 0.0f;

    float pitchCommand;

    float descentspeedDesired;
    float descentspeedError;
    float powerCommand;

    float airspeedVector[2];
    float courseCommand;

    FixedWingPathFollowerStatusGet(&fixedWingPathFollowerStatus);

    VelocityStateGet(&velocityState);
    StabilizationDesiredGet(&stabDesired);
    VelocityDesiredGet(&velocityDesired);
    AttitudeStateGet(&attitudeState);
    SystemSettingsGet(&systemSettings);

    /* unused */
    velocityDesired.North = 0;
    velocityDesired.East  = 0;
    velocityDesired.Down  = 0;
    VelocityDesiredSet(&velocityDesired);

    /**
     * Compute speed error and course
     */

    // check for airspeed sensor
    fixedWingPathFollowerStatus.Errors.AirspeedSensor = 0;
    if (airshipSettings->UseAirspeedSensor == AIRSHIPPATHFOLLOWERSETTINGS_USEAIRSPEEDSENSOR_FALSE) {
        // fallback algo triggered voluntarily
        indicatedAirspeedStateBias = 0;
        fixedWingPathFollowerStatus.Errors.AirspeedSensor = 1;
    } else if (PIOS_DELAY_GetuSSince(lastAirspeedUpdate) > 1000000) {
        // no airspeed update in one second, assume airspeed sensor failure
        indicatedAirspeedStateBias = 0;
        result = 0;
        fixedWingPathFollowerStatus.Errors.AirspeedSensor = 1;
    }

    // missing sensors for airspeed-direction we have to assume within
    // reasonable error that measured airspeed is actually the airspeed
    // component in forward pointing direction
    // airspeedVector is normalized
    airspeedVector[0]     = cos_lookup_deg(attitudeState.Yaw);
    airspeedVector[1]     = sin_lookup_deg(attitudeState.Yaw);

    // current ground speed projected in forward direction
    groundspeedProjection = velocityState.North * airspeedVector[0] + velocityState.East * airspeedVector[1];

    // note that airspeedStateBias is ( calibratedAirspeed - groundspeedProjection ) at the time of measurement,
    // but thanks to accelerometers,  groundspeedProjection reacts faster to changes in direction
    // than airspeed and gps sensors alone
    indicatedAirspeedState   = groundspeedProjection + indicatedAirspeedStateBias;

    indicatedAirspeedDesired = pathDesired->ModeParameters[0];

    // Airspeed error
    // fprintf(stderr,"Desired Airspeed: %f\tActual Airspeed: %f\n",indicatedAirspeedDesired, indicatedAirspeedState);
    airspeedError = indicatedAirspeedDesired - indicatedAirspeedState;

    // Error condition: plane too slow or too fast
    fixedWingPathFollowerStatus.Errors.Highspeed = 0;
    fixedWingPathFollowerStatus.Errors.Lowspeed  = 0;
    if (indicatedAirspeedState > systemSettings.AirSpeedMax * airshipSettings->Safetymargins.Overspeed) {
        fixedWingPathFollowerStatus.Errors.Overspeed = 1;
        result = 0;
    }
    if (indicatedAirspeedState > airshipSettings->HorizontalVelMax * airshipSettings->Safetymargins.Highspeed) {
        fixedWingPathFollowerStatus.Errors.Highspeed = 1;
        result = 0;
        cutThrust = true;
    }
    if (indicatedAirspeedState < airshipSettings->HorizontalVelMin * airshipSettings->Safetymargins.Lowspeed) {
        fixedWingPathFollowerStatus.Errors.Lowspeed = 1;
        result = 0;
    }

    // Vertical speed error
    descentspeedDesired = boundf(
        pathDesired->ModeParameters[2],
        -airshipSettings->VerticalVelMax,
        airshipSettings->VerticalVelMax);
    descentspeedError = descentspeedDesired - velocityState.Down;

    /**
     * Compute desired pitch command
     */

    // Compute the pitch command as err*Kp + errInt*Ki
    pitchCommand = -pid_apply(&PIDalt, descentspeedError, dT);

    fixedWingPathFollowerStatus.Error.Speed    = airspeedError;
    fixedWingPathFollowerStatus.ErrorInt.Speed = PIDalt.iAccumulator;
    fixedWingPathFollowerStatus.Command.Speed  = pitchCommand;

    stabDesired.Pitch = boundf(airshipSettings->PitchLimit.Neutral + pitchCommand,
                               airshipSettings->PitchLimit.Min,
                               airshipSettings->PitchLimit.Max);

    // Error condition: pitch way out of wack
    if (airshipSettings->Safetymargins.Pitchcontrol > 0.5f &&
        (attitudeState.Pitch < airshipSettings->PitchLimit.Min - airshipSettings->SafetyCutoffLimits.PitchDeg ||
         attitudeState.Pitch > airshipSettings->PitchLimit.Max + airshipSettings->SafetyCutoffLimits.PitchDeg)) {
        fixedWingPathFollowerStatus.Errors.Pitchcontrol = 1;
        result = 0;
        cutThrust = true;
    }

    /**
     * Compute desired thrust command
     */

    // Compute final thrust response
    powerCommand  = pid_apply(&PIDpower, airspeedError, dT);

    // compute pitch to power crossfeed
    powerCommand += boundf(airshipSettings->PitchToPowerCrossFeed.Kp * pitchCommand,
                           airshipSettings->PitchToPowerCrossFeed.Min,
                           airshipSettings->PitchToPowerCrossFeed.Max);

    // Output internal state to telemetry
    fixedWingPathFollowerStatus.Error.Power    = descentspeedError;
    fixedWingPathFollowerStatus.ErrorInt.Power = PIDpower.iAccumulator;
    fixedWingPathFollowerStatus.Command.Power  = powerCommand;

    // set thrust
    stabDesired.Thrust = boundf(airshipSettings->ThrustLimit.Neutral + powerCommand,
                                airshipSettings->ThrustLimit.Min,
                                airshipSettings->ThrustLimit.Max);

    // Error condition: plane cannot hold altitude at current speed.
    fixedWingPathFollowerStatus.Errors.Lowpower = 0;
    if (airshipSettings->ThrustLimit.Neutral + powerCommand >= airshipSettings->ThrustLimit.Max && // thrust at maximum
        velocityState.Down > 0.0f && // we ARE going down
        descentspeedDesired < 0.0f && // we WANT to go up
        airspeedError > 0.0f) { // we are too slow already
        fixedWingPathFollowerStatus.Errors.Lowpower = 1;

        if (airshipSettings->Safetymargins.Lowpower > 0.5f) { // alarm switched on
            result = 0;
        }
    }
    // Error condition: plane keeps climbing despite minimum thrust (opposite of above)
    fixedWingPathFollowerStatus.Errors.Highpower = 0;


    /**
     * Compute desired yaw command
     */

    courseCommand = pathDesired->ModeParameters[1];

    fixedWingPathFollowerStatus.Error.Course    = 0.0;
    fixedWingPathFollowerStatus.ErrorInt.Course = 0.0;
    fixedWingPathFollowerStatus.Command.Course  = courseCommand;

    stabDesired.Yaw = boundf(airshipSettings->YawLimit.Neutral +
                             courseCommand,
                             airshipSettings->YawLimit.Min,
                             airshipSettings->YawLimit.Max);

    // Error condition: roll way out of wack
    fixedWingPathFollowerStatus.Errors.Rollcontrol = 0;
    if (airshipSettings->Safetymargins.Rollcontrol > 0.5f &&
        (attitudeState.Roll < airshipSettings->RollLimit.Min - airshipSettings->SafetyCutoffLimits.RollDeg ||
         attitudeState.Roll > airshipSettings->RollLimit.Max + airshipSettings->SafetyCutoffLimits.RollDeg)) {
        fixedWingPathFollowerStatus.Errors.Rollcontrol = 1;
        result = 0;
        cutThrust = true;
    }

    /**
     * Compute desired roll command
     */
    stabDesired.Roll = boundf(airshipSettings->ThrustVector.Neutral + pitchCommand * airshipSettings->ThrustVector.Kp, airshipSettings->ThrustVector.Min, airshipSettings->ThrustVector.Max);

    // safety cutoff condition
    if (cutThrust) {
        stabDesired.Thrust = 0.0f;
    }

    stabDesired.StabilizationMode.Roll   = STABILIZATIONDESIRED_STABILIZATIONMODE_MANUAL;
    stabDesired.StabilizationMode.Pitch  = STABILIZATIONDESIRED_STABILIZATIONMODE_ATTITUDE;
    stabDesired.StabilizationMode.Yaw    = STABILIZATIONDESIRED_STABILIZATIONMODE_RATE;
    stabDesired.StabilizationMode.Thrust = STABILIZATIONDESIRED_STABILIZATIONMODE_MANUAL;

    StabilizationDesiredSet(&stabDesired);

    FixedWingPathFollowerStatusSet(&fixedWingPathFollowerStatus);

    return result;
}

void AirshipROSController::AirspeedStateUpdatedCb(__attribute__((unused)) UAVObjEvent *ev)
{
    AirspeedStateData airspeedState;
    VelocityStateData velocityState;

    AirspeedStateGet(&airspeedState);
    VelocityStateGet(&velocityState);
    float airspeedVector[2];
    float yaw;
    AttitudeStateYawGet(&yaw);
    airspeedVector[0] = cos_lookup_deg(yaw);
    airspeedVector[1] = sin_lookup_deg(yaw);
    // vector projection of groundspeed on airspeed vector to handle both forward and backwards movement
    float groundspeedProjection = velocityState.North * airspeedVector[0] + velocityState.East * airspeedVector[1];

    indicatedAirspeedStateBias = airspeedState.CalibratedAirspeed - groundspeedProjection;
    // note - we do fly by Indicated Airspeed (== calibrated airspeed) however
    // since airspeed is updated less often than groundspeed, we use sudden
    // changes to groundspeed to offset the airspeed by the same measurement.
    // This has a side effect that in the absence of any airspeed updates, the
    // pathfollower will fly using groundspeed.

    lastAirspeedUpdate = PIOS_DELAY_GetuS();
}
