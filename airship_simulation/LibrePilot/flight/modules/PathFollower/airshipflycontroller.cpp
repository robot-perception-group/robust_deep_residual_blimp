/*
 ******************************************************************************
 *
 * @file       AirshipFlyController.cpp
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
#include "airshipflycontroller.h"

// Private constants

// pointer to a singleton instance
AirshipFlyController *AirshipFlyController::p_inst = 0;

AirshipFlyController::AirshipFlyController()
    : airshipSettings(NULL), mActive(false), mMode(0), indicatedAirspeedStateBias(0.0f), previousCourseError(0.0f)
{}

// Called when mode first engaged
void AirshipFlyController::Activate(void)
{
    if (!mActive) {
        mActive = true;
        SettingsUpdated();
        resetGlobals();
        mMode   = pathDesired->Mode;
        lastAirspeedUpdate = 0;
    }
}

uint8_t AirshipFlyController::IsActive(void)
{
    return mActive;
}

uint8_t AirshipFlyController::Mode(void)
{
    return mMode;
}

// Objective updated in pathdesired
void AirshipFlyController::ObjectiveUpdated(void)
{}

void AirshipFlyController::Deactivate(void)
{
    if (mActive) {
        mActive = false;
        resetGlobals();
    }
}


void AirshipFlyController::SettingsUpdated(void)
{
    // fixed wing PID only
    pid_configure(&PIDposH[0], airshipSettings->HorizontalPosP, 0.0f, 0.0f, 0.0f);
    pid_configure(&PIDposH[1], airshipSettings->HorizontalPosP, 0.0f, 0.0f, 0.0f);
    pid_configure(&PIDposV, airshipSettings->VerticalPosP, 0.0f, 0.0f, 0.0f);

    pid_configure(&PIDalt, airshipSettings->AltPI.Kp, airshipSettings->AltPI.Ki, 0.0f, airshipSettings->AltPI.ILimit);
    pid_configure(&PIDpower, airshipSettings->PowerPI.Kp, airshipSettings->PowerPI.Ki, 0.0f, airshipSettings->PowerPI.ILimit);
}

/**
 * Initialise the module, called on startup
 * \returns 0 on success or -1 if initialisation failed
 */
int32_t AirshipFlyController::Initialize(AirshipPathFollowerSettingsData *ptr_airshipSettings)
{
    PIOS_Assert(ptr_airshipSettings);

    airshipSettings = ptr_airshipSettings;

    resetGlobals();

    return 0;
}

/**
 * reset integrals
 */
void AirshipFlyController::resetGlobals()
{
    pid_zero(&PIDposH[0]);
    pid_zero(&PIDposH[1]);
    pid_zero(&PIDposV);
    pid_zero(&PIDalt);
    pid_zero(&PIDpower);
    pathStatus->path_time = 0.0f;
}

void AirshipFlyController::UpdateAutoPilot()
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
 * fixed wing autopilot:
 * straight forward:
 * 1. update path velocity for limited motion crafts
 * 2. update attitude according to default fixed wing pathfollower algorithm
 */
uint8_t AirshipFlyController::updateAutoPilotAirship()
{
    updatePathVelocity(airshipSettings->CourseFeedForward, true);
    return updateFixedDesiredAttitude();
}

/**
 * Compute desired velocity from the current position and path
 */
void AirshipFlyController::updatePathVelocity(float kFF, bool limited)
{
    PositionStateData positionState;

    PositionStateGet(&positionState);
    VelocityStateData velocityState;
    VelocityStateGet(&velocityState);
    VelocityDesiredData velocityDesired;

    const float dT = airshipSettings->UpdatePeriod / 1000.0f;

    // look ahead kFF seconds
    float cur[3]   = { positionState.North + (velocityState.North * kFF),
                       positionState.East + (velocityState.East * kFF),
                       positionState.Down + (velocityState.Down * kFF) };
    struct path_status progress;
    path_progress(pathDesired, cur, &progress, true);

    // calculate velocity - can be zero if waypoints are too close
    velocityDesired.North = progress.path_vector[0];
    velocityDesired.East  = progress.path_vector[1];
    velocityDesired.Down  = progress.path_vector[2];

    if (limited &&
        // if a plane is crossing its desired flightpath facing the wrong way (away from flight direction)
        // it would turn towards the flightpath to get on its desired course. This however would reverse the correction vector
        // once it crosses the flightpath again, which would make it again turn towards the flightpath (but away from its desired heading)
        // leading to an S-shape snake course the wrong way
        // this only happens especially if HorizontalPosP is too high, as otherwise the angle between velocity desired and path_direction won't
        // turn steep unless there is enough space complete the turn before crossing the flightpath
        // in this case the plane effectively needs to be turned around
        // indicators:
        // difference between correction_direction and velocitystate >90 degrees and
        // difference between path_direction and velocitystate >90 degrees  ( 4th sector, facing away from everything )
        // fix: ignore correction, steer in path direction until the situation has become better (condition doesn't apply anymore)
        // calculating angles < 90 degrees through dot products
        (vector_lengthf(progress.path_vector, 2) > 1e-6f) &&
        ((progress.path_vector[0] * velocityState.North + progress.path_vector[1] * velocityState.East) < 0.0f) &&
        ((progress.correction_vector[0] * velocityState.North + progress.correction_vector[1] * velocityState.East) < 0.0f)) {
        ;
    } else {
        // calculate correction
        velocityDesired.North += pid_apply(&PIDposH[0], progress.correction_vector[0], dT);
        velocityDesired.East  += pid_apply(&PIDposH[1], progress.correction_vector[1], dT);
    }
    velocityDesired.Down += pid_apply(&PIDposV, progress.correction_vector[2], dT);

    // update pathstatus
    pathStatus->error     = progress.error;
    pathStatus->fractional_progress  = progress.fractional_progress;
    pathStatus->path_direction_north = progress.path_vector[0];
    pathStatus->path_direction_east  = progress.path_vector[1];
    pathStatus->path_direction_down  = progress.path_vector[2];

    pathStatus->correction_direction_north = progress.correction_vector[0];
    pathStatus->correction_direction_east  = progress.correction_vector[1];
    pathStatus->correction_direction_down  = progress.correction_vector[2];


    VelocityDesiredSet(&velocityDesired);
}


/**
 * Compute desired attitude from the desired velocity for fixed wing craft
 */
uint8_t AirshipFlyController::updateFixedDesiredAttitude()
{
    uint8_t result = 1;
    bool cutThrust = false;

    __attribute__((unused)) bool hasAirspeed = true;

    const float dT = airshipSettings->UpdatePeriod / 1000.0f;

    VelocityDesiredData velocityDesired;
    VelocityStateData velocityState;
    StabilizationDesiredData stabDesired;
    AttitudeStateData attitudeState;
    FixedWingPathFollowerStatusData fixedWingPathFollowerStatus;
    AirspeedStateData airspeedState;
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
    float fluidMovement[2];
    float courseComponent[2];
    float courseError;
    float courseCommand;

    FixedWingPathFollowerStatusGet(&fixedWingPathFollowerStatus);

    VelocityStateGet(&velocityState);
    StabilizationDesiredGet(&stabDesired);
    VelocityDesiredGet(&velocityDesired);
    AttitudeStateGet(&attitudeState);
    AirspeedStateGet(&airspeedState);
    SystemSettingsGet(&systemSettings);


    /**
     * Compute speed error and course
     */

    // check for airspeed sensor
    fixedWingPathFollowerStatus.Errors.AirspeedSensor = 0;
    if (airshipSettings->UseAirspeedSensor == AIRSHIPPATHFOLLOWERSETTINGS_USEAIRSPEEDSENSOR_FALSE) {
        // fallback algo triggered voluntarily
        hasAirspeed = false;
        indicatedAirspeedStateBias = 0;
        fixedWingPathFollowerStatus.Errors.AirspeedSensor = 1;
    } else if (PIOS_DELAY_GetuSSince(lastAirspeedUpdate) > 1000000) {
        // no airspeed update in one second, assume airspeed sensor failure
        hasAirspeed = false;
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
    indicatedAirspeedState = groundspeedProjection + indicatedAirspeedStateBias;

    // fluidMovement is a vector describing the aproximate movement vector of
    // the surrounding fluid in 2d space (aka wind vector)
    fluidMovement[0] = velocityState.North - (indicatedAirspeedState * airspeedVector[0]);
    fluidMovement[1] = velocityState.East - (indicatedAirspeedState * airspeedVector[1]);

    // calculate the movement vector we need to fly to reach velocityDesired -
    // taking fluidMovement into account
    courseComponent[0] = velocityDesired.North - fluidMovement[0];
    courseComponent[1] = velocityDesired.East - fluidMovement[1];

    indicatedAirspeedDesired = boundf(sqrtf(courseComponent[0] * courseComponent[0] + courseComponent[1] * courseComponent[1]),
                                      airshipSettings->HorizontalVelMin,
                                      airshipSettings->HorizontalVelMax);

    // if we could fly at arbitrary speeds, we'd just have to move towards the
    // courseComponent vector as previously calculated and we'd be fine
    // unfortunately however we are bound by min and max air speed limits, so
    // we need to recalculate the correct course to meet at least the
    // velocityDesired vector direction at our current speed
    // this overwrites courseComponent
    bool valid = correctCourse(courseComponent, (float *)&velocityDesired.North, fluidMovement, indicatedAirspeedDesired);

    // Error condition: wind speed too high, we can't go where we want anymore
    fixedWingPathFollowerStatus.Errors.Wind = 0;
    if ((!valid) &&
        airshipSettings->Safetymargins.Wind > 0.5f) { // alarm switched on
        fixedWingPathFollowerStatus.Errors.Wind = 1;
        result = 0;
    }

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
        velocityDesired.Down,
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
    courseError = RAD2DEG(atan2f(courseComponent[1], courseComponent[0])) - attitudeState.Yaw;

    if (courseError < -180.0f) {
        courseError += 360.0f;
    }
    if (courseError > 180.0f) {
        courseError -= 360.0f;
    }

    // overlap calculation. Theres a dead zone behind the craft where the
    // counter-yawing of some craft could render a desired right
    // turn into a desired left turn. Making the turn direction based on
    // previous error current roll angle keeps the plane committed to a direction once chosen
    if (courseError < -180.0f + (airshipSettings->ReverseCourseOverlap * 0.5f)
        && previousCourseError > 0.0f) {
        courseError += 360.0f;
    }
    if (courseError > 180.0f - (airshipSettings->ReverseCourseOverlap * 0.5f)
        && previousCourseError < 0.0f) {
        courseError -= 360.0f;
    }
    previousCourseError = courseError;

    courseCommand = airshipSettings->CourseP * courseError;

    fixedWingPathFollowerStatus.Error.Course    = courseError;
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


/**
 * Function to calculate course vector C based on airspeed s, fluid movement F
 * and desired movement vector V
 * parameters in: V,F,s
 * parameters out: C
 * returns true if a valid solution could be found for V,F,s, false if not
 * C will be set to a best effort attempt either way
 */
bool AirshipFlyController::correctCourse(float *C, float *V, float *F, float s)
{
    // Approach:
    // Let Sc be a circle around origin marking possible movement vectors
    // of the craft with airspeed s (all possible options for C)
    // Let Vl be a line through the origin along movement vector V where fr any
    // point v on line Vl v = k * (V / |V|) = k' * V
    // Let Wl be a line parallel to Vl where for any point v on line Vl exists
    // a point w on WL with w = v - F
    // Then any intersection between circle Sc and line Wl represents course
    // vector which would result in a movement vector
    // V' = k * ( V / |V|) = k' * V
    // If there is no intersection point, S is insufficient to compensate
    // for F and we can only try to fly in direction of V (thus having wind drift
    // but at least making progress orthogonal to wind)

    s = fabsf(s);
    float f = vector_lengthf(F, 2);

    // normalize Cn=V/|V|, |V| must be >0
    float v = vector_lengthf(V, 2);
    if (v < 1e-6f) {
        // if |V|=0, we aren't supposed to move, turn into the wind
        // (this allows hovering)
        C[0] = -F[0];
        C[1] = -F[1];
        // if desired airspeed matches fluidmovement a hover is actually
        // intended so return true
        return fabsf(f - s) < 1e-3f;
    }
    float Vn[2] = { V[0] / v, V[1] / v };

    // project F on V
    float fp    = F[0] * Vn[0] + F[1] * Vn[1];

    // find component Fo of F that is orthogonal to V
    // (which is exactly the distance between Vl and Wl)
    float Fo[2] = { F[0] - (fp * Vn[0]), F[1] - (fp * Vn[1]) };
    float fo2   = Fo[0] * Fo[0] + Fo[1] * Fo[1];

    // find k where k * Vn = C - Fo
    // |C|=s is the hypothenuse in any rectangular triangle formed by k * Vn and Fo
    // so k^2 + fo^2 = s^2 (since |Vn|=1)
    float k2 = s * s - fo2;
    if (k2 <= -1e-3f) {
        // there is no solution, we will be drifted off either way
        // fallback: fly stupidly in direction of V and hope for the best
        C[0] = V[0];
        C[1] = V[1];
        return false;
    } else if (k2 <= 1e-3f) {
        // there is exactly one solution: -Fo
        C[0] = -Fo[0];
        C[1] = -Fo[1];
        return true;
    }
    // we have two possible solutions k positive and k negative as there are
    // two intersection points between Wl and Sc
    // which one is better? two criteria:
    // 1. we MUST move in the right direction, if any k leads to -v its invalid
    // 2. we should minimize the speed error
    float k     = sqrt(k2);
    float C1[2] = { -k * Vn[0] - Fo[0], -k * Vn[1] - Fo[1] };
    float C2[2] = { k *Vn[0] - Fo[0], k * Vn[1] - Fo[1] };
    // project C+F on Vn to find signed resulting movement vector length
    float vp1   = (C1[0] + F[0]) * Vn[0] + (C1[1] + F[1]) * Vn[1];
    float vp2   = (C2[0] + F[0]) * Vn[0] + (C2[1] + F[1]) * Vn[1];
    if (vp1 >= 0.0f && fabsf(v - vp1) < fabsf(v - vp2)) {
        // in this case the angle between course and resulting movement vector
        // is greater than 90 degrees - so we actually fly backwards
        C[0] = C1[0];
        C[1] = C1[1];
        return true;
    }
    C[0] = C2[0];
    C[1] = C2[1];
    if (vp2 >= 0.0f) {
        // in this case the angle between course and movement vector is less than
        // 90 degrees, but we do move in the right direction
        return true;
    } else {
        // in this case we actually get driven in the opposite direction of V
        // with both solutions for C
        // this might be reached in headwind stronger than maximum allowed
        // airspeed.
        return false;
    }
}


void AirshipFlyController::AirspeedStateUpdatedCb(__attribute__((unused)) UAVObjEvent *ev)
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
