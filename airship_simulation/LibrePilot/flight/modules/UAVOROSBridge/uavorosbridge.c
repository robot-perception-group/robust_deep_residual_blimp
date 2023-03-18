/**
 ******************************************************************************
 * @addtogroup OpenPilotModules OpenPilot Modules
 * @{
 * @addtogroup UAVOROSBridge UAVO to ROS Bridge Module
 * @{
 *
 * @file       uavorosridge.c
 * @author     The LibrePilot Project, http://www.librepilot.org Copyright (C) 2016.
 *             Max Planck Institute for intelligent systems, http://www.is.mpg.de Copyright (C) 2016.
 * @brief      Bridges certain UAVObjects to ROS on USB VCP
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
 *
 * Additional note on redistribution: The copyright and license notices above
 * must be maintained in each individual source file that is a derivative work
 * of this source file; otherwise redistribution is prohibited.
 */

#include "openpilot.h"
#include "hwsettings.h"
#include "taskinfo.h"
#include "callbackinfo.h"
#include "insgps.h"
#include "CoordinateConversions.h"
/*
   #include "receiverstatus.h"
   #include "flightmodesettings.h"
   #include "flightbatterysettings.h"
   #include "flightbatterystate.h"
   #include "gpspositionsensor.h"
   #include "manualcontrolsettings.h"
   #include "oplinkstatus.h"
   #include "accessorydesired.h"
   #include "actuatorsettings.h"
   #include "systemstats.h"
   #include "systemalarms.h"
   #include "takeofflocation.h"
   #include "homelocation.h"
   #include "stabilizationdesired.h"
   #include "stabilizationsettings.h"
   #include "stabilizationbank.h"
   #include "stabilizationsettingsbank1.h"
   #include "stabilizationsettingsbank2.h"
   #include "stabilizationsettingsbank3.h"
   #include "magstate.h"
 */

#include "revosettings.h"
#include "airspeedstate.h"
#include "manualcontrolcommand.h"
#include "accessorydesired.h"
#include "actuatordesired.h"
#include "actuatorcommand.h"
#include "actuatoroverride.h"
#include "auxpositionsensor.h"
#include "auxvelocitysensor.h"
#include "pathdesired.h"
#include "pathstatus.h"
#include "poilocation.h"
#include "flightmodesettings.h"
#include "flightstatus.h"
#include "positionstate.h"
#include "velocitystate.h"
#include "velocitydesired.h"
#include "attitudestate.h"
#include "gyrostate.h"
#include "gyrosensor.h"
#include "accelstate.h"
#include "accelsensor.h"
#include "rosbridgesettings.h"
#include "rosbridgestatus.h"

#include "pios_sensors.h"

#include <pios_board_io.h>

#if defined(PIOS_INCLUDE_ROS_BRIDGE)

#include <uavorosbridgemessage_priv.h>

struct ros_bridge {
    uintptr_t     com;

    uint32_t      lastPingTimestamp;
    uint8_t       myPingSequence;
    uint8_t       remotePingSequence;
    PiOSDeltatimeConfig roundtrip;
    double        roundTripTime;
    int32_t       pingTimer;
    int32_t       stateTimer;
    int32_t       biasTimer;
    int32_t       autopilotTimer;
    int32_t       rateTimer;
    float         gyrRef[3];
    float         gyrBias[3];
    float         rateAccumulator[3];
    float         rawGyrAccumulator[3];
    float         rawAccAccumulator[3];
    int32_t       gyrTimer;
    int32_t       accTimer;
    uint8_t       rx_buffer[ROSBRIDGEMESSAGE_BUFFERSIZE];
    size_t        rx_length;
    volatile bool scheduled[ROSBRIDGEMESSAGE_END_ARRAY_SIZE];
};

#if defined(PIOS_ROS_STACK_SIZE)
#define STACK_SIZE_BYTES  PIOS_ROS_STACK_SIZE
#else
#define STACK_SIZE_BYTES  2048
#endif
#define TASK_PRIORITY     CALLBACK_TASK_AUXILIARY
#define CALLBACK_PRIORITY CALLBACK_PRIORITY_REGULAR
#define CBTASK_PRIORITY   CALLBACK_TASK_AUXILIARY

static bool module_enabled = false;
static struct ros_bridge *ros;
int32_t UAVOROSBridgeInitialize(void);
static void UAVOROSBridgeRxTask(void *parameters);
static void UAVOROSBridgeTxTask(void);
static DelayedCallbackInfo *callbackHandle;
static rosbridgemessage_handler ping_handler, ping_r_handler, pong_handler, pong_r_handler, fullstate_estimate_handler, imu_average_handler, gyro_bias_handler, gimbal_estimate_handler, flightcontrol_r_handler, pos_estimate_r_handler, vel_estimate_r_handler, actuators_handler, actuators_r_handler, fullstate_estimate_r_handler, autopilot_info_handler;
static ROSBridgeSettingsData settings;
void AttitudeCb(__attribute__((unused)) UAVObjEvent *ev);
void SettingsCb(__attribute__((unused)) UAVObjEvent *ev);
void RateCb(__attribute__((unused)) UAVObjEvent *ev);
void RawGyrCb(__attribute__((unused)) UAVObjEvent *ev);
void RawAccCb(__attribute__((unused)) UAVObjEvent *ev);

static bool commandOverridePermitted = false;

// order here is important and must match order of rosbridgemessagetype_t
static rosbridgemessage_handler *const rosbridgemessagehandlers[ROSBRIDGEMESSAGE_END_ARRAY_SIZE] = {
    ping_handler,
    NULL,
    NULL,
    NULL,
    pong_handler,
    fullstate_estimate_handler,
    imu_average_handler,
    gyro_bias_handler,
    gimbal_estimate_handler,
    NULL,
    actuators_handler,
    autopilot_info_handler
};


/**
 * Process incoming bytes from an ROS query thing.
 * @param[in] b received byte
 * @return true if we should continue processing bytes
 */
static void ros_receive_byte(struct ros_bridge *m, uint8_t b)
{
    m->rx_buffer[m->rx_length] = b;
    m->rx_length++;
    rosbridgemessage_t *message = (rosbridgemessage_t *)m->rx_buffer;

    // very simple parser - but not a state machine, just a few checks
    if (m->rx_length <= offsetof(rosbridgemessage_t, length)) {
        // check (partial) magic number - partial is important since we need to restart at any time if garbage is received
        uint32_t canary = 0xff;
        for (uint32_t t = 1; t < m->rx_length; t++) {
            canary = (canary << 8) | 0xff;
        }
        if ((message->magic & canary) != (ROSBRIDGEMAGIC & canary)) {
            // parse error, not beginning of message
            goto rxfailure;
        }
    }
    if (m->rx_length == offsetof(rosbridgemessage_t, timestamp)) {
        if (message->length > (uint32_t)(ROSBRIDGEMESSAGE_BUFFERSIZE - offsetof(rosbridgemessage_t, data))) {
            // parse error, no messages are that long
            goto rxfailure;
        }
    }
    if (m->rx_length == offsetof(rosbridgemessage_t, crc32)) {
        if (message->type >= ROSBRIDGEMESSAGE_END_ARRAY_SIZE) {
            // parse error
            goto rxfailure;
        }
        if (message->length != ROSBRIDGEMESSAGE_SIZES[message->type]) {
            // parse error
            goto rxfailure;
        }
    }
    if (m->rx_length < offsetof(rosbridgemessage_t, data)) {
        // not a parse failure, just not there yet
        return;
    }
    if (m->rx_length == offsetof(rosbridgemessage_t, data) + ROSBRIDGEMESSAGE_SIZES[message->type]) {
        // complete message received and stored in pointer "message"
        // empty buffer for next message
        m->rx_length = 0;

        if (PIOS_CRC32_updateCRC(0xffffffff, message->data, message->length) != message->crc32) {
            // crc mismatch
            goto rxfailure;
        }
        uint32_t rxpackets;
        ROSBridgeStatusRxPacketsGet(&rxpackets);
        rxpackets++;
        ROSBridgeStatusRxPacketsSet(&rxpackets);
        switch (message->type) {
        case ROSBRIDGEMESSAGE_PING:
            ping_r_handler(m, message);
            break;
        case ROSBRIDGEMESSAGE_POS_ESTIMATE:
            pos_estimate_r_handler(m, message);
            break;
        case ROSBRIDGEMESSAGE_FLIGHTCONTROL:
            flightcontrol_r_handler(m, message);
            break;
        case ROSBRIDGEMESSAGE_GIMBALCONTROL:
            // TODO implement gimbal control somehow
            break;
        case ROSBRIDGEMESSAGE_PONG:
            pong_r_handler(m, message);
            break;
        case ROSBRIDGEMESSAGE_VEL_ESTIMATE:
            vel_estimate_r_handler(m, message);
            break;
        case ROSBRIDGEMESSAGE_ACTUATORS:
            actuators_r_handler(m, message);
            break;
        case ROSBRIDGEMESSAGE_FULLSTATE_ESTIMATE:
            fullstate_estimate_r_handler(m, message);
            break;
        default:
            // do nothing at all and discard the message
            break;
        }
    }
    return;

rxfailure:
    m->rx_length = 0;
    uint32_t rxfail;
    ROSBridgeStatusRxFailGet(&rxfail);
    rxfail++;
    ROSBridgeStatusRxFailSet(&rxfail);
}

static uint32_t hwsettings_rosspeed_enum_to_baud(uint8_t baud)
{
    switch (baud) {
    case HWSETTINGS_ROSSPEED_2400:
        return 2400;

    case HWSETTINGS_ROSSPEED_4800:
        return 4800;

    case HWSETTINGS_ROSSPEED_9600:
        return 9600;

    case HWSETTINGS_ROSSPEED_19200:
        return 19200;

    case HWSETTINGS_ROSSPEED_38400:
        return 38400;

    case HWSETTINGS_ROSSPEED_57600:
        return 57600;

    default:
    case HWSETTINGS_ROSSPEED_115200:
        return 115200;
    }
}


/**
 * Module start routine automatically called after initialization routine
 * @return 0 when was successful
 */
int32_t UAVOROSBridgeStart(void)
{
    if (!module_enabled) {
        // give port to telemetry if it doesn't have one
        // stops board getting stuck in condition where it can't be connected to gcs
        if (!PIOS_COM_TELEM_RF) {
            PIOS_COM_TELEM_RF = PIOS_COM_ROS;
        }

        return -1;
    }

    PIOS_DELTATIME_Init(&ros->roundtrip, 1e-3f, 1e-6f, 10.0f, 1e-1f);
    ros->pingTimer            = 0;
    ros->stateTimer           = 0;
    ros->biasTimer            = 0;
    ros->autopilotTimer       = 0;
    ros->rateTimer            = 0;
    ros->gyrTimer = 0;
    ros->accTimer = 0;
    ros->gyrRef[0]            = 0;
    ros->gyrRef[1]            = 0;
    ros->gyrRef[2]            = 0;
    ros->gyrBias[0]           = 0;
    ros->gyrBias[1]           = 0;
    ros->gyrBias[2]           = 0;
    ros->rateAccumulator[0]   = 0;
    ros->rateAccumulator[1]   = 0;
    ros->rateAccumulator[2]   = 0;
    ros->rawGyrAccumulator[0] = 0;
    ros->rawGyrAccumulator[1] = 0;
    ros->rawGyrAccumulator[2] = 0;
    ros->rawAccAccumulator[0] = 0;
    ros->rawAccAccumulator[1] = 0;
    ros->rawAccAccumulator[2] = 0;
    ros->rx_length            = 0;
    ros->myPingSequence       = 0x66;

    xTaskHandle taskHandle;

    xTaskCreate(UAVOROSBridgeRxTask, "UAVOROSBridge", STACK_SIZE_BYTES / 4, NULL, TASK_PRIORITY, &taskHandle);
    PIOS_TASK_MONITOR_RegisterTask(TASKINFO_RUNNING_UAVOROSBRIDGE, taskHandle);
    PIOS_CALLBACKSCHEDULER_Dispatch(callbackHandle);

    return 0;
}

/**
 * Module initialization routine
 * @return 0 when initialization was successful
 */
int32_t UAVOROSBridgeInitialize(void)
{
    if (PIOS_COM_ROS) {
        ros = pios_malloc(sizeof(*ros));
        if (ros != NULL) {
            memset(ros, 0x00, sizeof(*ros));

            ros->com = PIOS_COM_ROS;

            RevoSettingsInitialize();
            ROSBridgeSettingsInitialize();
            ROSBridgeSettingsConnectCallback(&SettingsCb);
            SettingsCb(NULL);
            ROSBridgeStatusInitialize();
            AUXPositionSensorInitialize();
            HwSettingsInitialize();
            HwSettingsROSSpeedOptions rosSpeed;
            HwSettingsROSSpeedGet(&rosSpeed);

            PIOS_COM_ChangeBaud(PIOS_COM_ROS, hwsettings_rosspeed_enum_to_baud(rosSpeed));
            callbackHandle = PIOS_CALLBACKSCHEDULER_Create(&UAVOROSBridgeTxTask, CALLBACK_PRIORITY, CBTASK_PRIORITY, CALLBACKINFO_RUNNING_UAVOROSBRIDGE, STACK_SIZE_BYTES);
            AirspeedStateInitialize();
            VelocityStateInitialize();
            PositionStateInitialize();
            AttitudeStateInitialize();
            AttitudeStateConnectCallback(&AttitudeCb);
            GyroStateInitialize();
            GyroStateConnectCallback(&RateCb);
            GyroSensorInitialize();
            GyroSensorConnectCallback(&RawGyrCb);
            AccelStateInitialize();
            AccelSensorInitialize();
            AccelSensorConnectCallback(&RawAccCb);
            FlightStatusInitialize();
            PathDesiredInitialize();
            PathStatusInitialize();
            VelocityDesiredInitialize();
            PoiLocationInitialize();
            ActuatorDesiredInitialize();
            ActuatorCommandInitialize();
            ActuatorOverrideInitialize();
            FlightModeSettingsInitialize();
            ManualControlCommandInitialize();
            AccessoryDesiredInitialize();

            module_enabled = true;
            return 0;
        }
    }

    return -1;
}
MODULE_INITCALL(UAVOROSBridgeInitialize, UAVOROSBridgeStart);

/** various handlers **/
static void ping_r_handler(struct ros_bridge *rb, rosbridgemessage_t *m)
{
    rosbridgemessage_pingpong_t *data = (rosbridgemessage_pingpong_t *)&(m->data);

    rb->remotePingSequence = data->sequence_number;
    rb->scheduled[ROSBRIDGEMESSAGE_PONG] = true;
    PIOS_CALLBACKSCHEDULER_Dispatch(callbackHandle);
}

static void ping_handler(struct ros_bridge *rb, rosbridgemessage_t *m)
{
    rosbridgemessage_pingpong_t *data = (rosbridgemessage_pingpong_t *)&(m->data);

    data->sequence_number = ++rb->myPingSequence;
    rb->roundtrip.last    = PIOS_DELAY_GetRaw();
}

static void pong_r_handler(struct ros_bridge *rb, rosbridgemessage_t *m)
{
    rosbridgemessage_pingpong_t *data = (rosbridgemessage_pingpong_t *)&(m->data);

    if (data->sequence_number != rb->myPingSequence) {
        return;
    }
    float roundtrip = PIOS_DELTATIME_GetAverageSeconds(&(rb->roundtrip));
    ROSBridgeStatusPingRoundTripTimeSet(&roundtrip);
}

static void flightcontrol_r_handler(__attribute__((unused)) struct ros_bridge *rb, rosbridgemessage_t *m)
{
    rosbridgemessage_flightcontrol_t *data = (rosbridgemessage_flightcontrol_t *)&(m->data);
    FlightStatusFlightModeOptions mode;

    commandOverridePermitted = false;

    FlightStatusFlightModeGet(&mode);
    if (mode != FLIGHTSTATUS_FLIGHTMODE_ROSCONTROLLED) {
        return;
    }

    // safety
    if (!IS_REAL(data->control[0])) {
        return;
    }
    if (!IS_REAL(data->control[1])) {
        return;
    }
    if (!IS_REAL(data->control[2])) {
        return;
    }
    if (!IS_REAL(data->control[3])) {
        return;
    }
    if (!IS_REAL(data->vel[0])) {
        return;
    }
    if (!IS_REAL(data->vel[1])) {
        return;
    }
    if (!IS_REAL(data->vel[2])) {
        return;
    }
    if (!IS_REAL(data->poi[0])) {
        return;
    }
    if (!IS_REAL(data->poi[1])) {
        return;
    }
    if (!IS_REAL(data->poi[2])) {
        return;
    }

    PathDesiredData pathDesired;
    PathDesiredGet(&pathDesired);
    switch (data->mode) {
    case ROSBRIDGEMESSAGE_FLIGHTCONTROL_MODE_ACTUATORS:
    {
        // this behaves like position hold - but with command overrides - the effect is - if command overrides stop happening, we end up in position hold
        PositionStateData curpos;
        PositionStateGet(&curpos);
        FlightModeSettingsPositionHoldOffsetData offset;
        FlightModeSettingsPositionHoldOffsetGet(&offset);
        pathDesired.End.North        = boundf(curpos.North, settings.GeoFenceBoxMin.North, settings.GeoFenceBoxMax.North);
        pathDesired.End.East         = boundf(curpos.East, settings.GeoFenceBoxMin.East, settings.GeoFenceBoxMax.East);
        pathDesired.End.Down         = boundf(curpos.Down, settings.GeoFenceBoxMin.Down, settings.GeoFenceBoxMax.Down);
        pathDesired.Start.North      = pathDesired.End.North + offset.Horizontal;
        pathDesired.Start.East       = pathDesired.End.East;
        pathDesired.Start.Down       = pathDesired.End.Down;
        pathDesired.StartingVelocity = 0.0f;
        pathDesired.EndingVelocity   = 0.0f;
        pathDesired.Mode = PATHDESIRED_MODE_GOTOENDPOINT;
        if (
            curpos.North < settings.GeoFenceBoxMin.North || curpos.North > settings.GeoFenceBoxMax.North ||
            curpos.East < settings.GeoFenceBoxMin.East || curpos.East > settings.GeoFenceBoxMax.East ||
            curpos.Down < settings.GeoFenceBoxMin.Down || curpos.Down > settings.GeoFenceBoxMax.Down) {
            // if outside of safebox, disallow command overriding - this is also set to false by canary.
            commandOverridePermitted = false;
        } else {
            commandOverridePermitted = true;
        }
    }
    break;
    case ROSBRIDGEMESSAGE_FLIGHTCONTROL_MODE_WAYPOINT:
    {
        FlightModeSettingsPositionHoldOffsetData offset;
        FlightModeSettingsPositionHoldOffsetGet(&offset);
        pathDesired.End.North        = boundf(data->control[0], settings.GeoFenceBoxMin.North, settings.GeoFenceBoxMax.North);
        pathDesired.End.East         = boundf(data->control[1], settings.GeoFenceBoxMin.East, settings.GeoFenceBoxMax.East);
        pathDesired.End.Down         = boundf(data->control[2], settings.GeoFenceBoxMin.Down, settings.GeoFenceBoxMax.Down);
        pathDesired.Start.North      = pathDesired.End.North + offset.Horizontal;
        pathDesired.Start.East       = pathDesired.End.East;
        pathDesired.Start.Down       = pathDesired.End.Down;
        pathDesired.StartingVelocity = 0.0f;
        pathDesired.EndingVelocity   = 0.0f;
        pathDesired.Mode = PATHDESIRED_MODE_GOTOENDPOINT;
    }
    break;
    case ROSBRIDGEMESSAGE_FLIGHTCONTROL_MODE_VECTOR:
    {
        FlightModeSettingsPositionHoldOffsetData offset;
        FlightModeSettingsPositionHoldOffsetGet(&offset);
        PositionStateData curpos;
        PositionStateGet(&curpos);
        if (
            curpos.North < settings.GeoFenceBoxMin.North || curpos.North > settings.GeoFenceBoxMax.North ||
            curpos.East < settings.GeoFenceBoxMin.East || curpos.East > settings.GeoFenceBoxMax.East ||
            curpos.Down < settings.GeoFenceBoxMin.Down || curpos.Down > settings.GeoFenceBoxMax.Down) {
            // outside of safebox, fly back to safebox
            pathDesired.End.North        = (settings.GeoFenceBoxMin.North + settings.GeoFenceBoxMax.North) / 2.0f;
            pathDesired.End.East         = (settings.GeoFenceBoxMin.East + settings.GeoFenceBoxMax.East) / 2.0f;
            pathDesired.End.Down         = (settings.GeoFenceBoxMin.Down + settings.GeoFenceBoxMax.Down) / 2.0f;
            pathDesired.Start.North      = curpos.North;
            pathDesired.Start.East       = curpos.East;
            pathDesired.Start.Down       = curpos.Down;
            pathDesired.StartingVelocity = 1.0f;
            pathDesired.EndingVelocity   = 0.0f;
            pathDesired.Mode = PATHDESIRED_MODE_GOTOENDPOINT;
        } else {
            pathDesired.Start.North      = data->control[0];
            pathDesired.Start.East       = data->control[1];
            pathDesired.Start.Down       = data->control[2];
            pathDesired.End.North        = data->control[0] + data->vel[0];
            pathDesired.End.East         = data->control[1] + data->vel[1];
            pathDesired.End.Down         = data->control[2] + data->vel[2];
            pathDesired.StartingVelocity = VectorMagnitude(data->vel);
            pathDesired.EndingVelocity   = pathDesired.StartingVelocity;
            pathDesired.Mode = PATHDESIRED_MODE_FOLLOWVECTOR;
        }
    }
    break;
    case ROSBRIDGEMESSAGE_FLIGHTCONTROL_MODE_ROSCONTROL:
    {
        FlightModeSettingsPositionHoldOffsetData offset;
        FlightModeSettingsPositionHoldOffsetGet(&offset);
        PositionStateData curpos;
        PositionStateGet(&curpos);
        if (
            curpos.North < settings.GeoFenceBoxMin.North || curpos.North > settings.GeoFenceBoxMax.North ||
            curpos.East < settings.GeoFenceBoxMin.East || curpos.East > settings.GeoFenceBoxMax.East ||
            curpos.Down < settings.GeoFenceBoxMin.Down || curpos.Down > settings.GeoFenceBoxMax.Down) {
            // outside of safebox, fly back to safebox
            pathDesired.End.North        = (settings.GeoFenceBoxMin.North + settings.GeoFenceBoxMax.North) / 2.0f;
            pathDesired.End.East         = (settings.GeoFenceBoxMin.East + settings.GeoFenceBoxMax.East) / 2.0f;
            pathDesired.End.Down         = (settings.GeoFenceBoxMin.Down + settings.GeoFenceBoxMax.Down) / 2.0f;
            pathDesired.Start.North      = curpos.North;
            pathDesired.Start.East       = curpos.East;
            pathDesired.Start.Down       = curpos.Down;
            pathDesired.StartingVelocity = 1.0f;
            pathDesired.EndingVelocity   = 0.0f;
            pathDesired.Mode = PATHDESIRED_MODE_GOTOENDPOINT;
        } else {
            pathDesired.ModeParameters[0] = data->control[0];
            pathDesired.ModeParameters[1] = data->control[1];
            pathDesired.ModeParameters[2] = data->control[2];
            pathDesired.ModeParameters[3] = data->control[3];
            pathDesired.Start.North      = 0.0f,
            pathDesired.Start.East       = 0.0f,
            pathDesired.Start.Down       = 0.0f,
            pathDesired.End.North        = data->vel[0],
            pathDesired.End.East         = data->vel[1],
            pathDesired.End.Down         = data->vel[2],
            pathDesired.StartingVelocity = 0.0f;
            pathDesired.EndingVelocity   = data->vel[3];
            pathDesired.Mode = PATHDESIRED_MODE_ROSCONTROL;
        }
    }
    break;
    case ROSBRIDGEMESSAGE_FLIGHTCONTROL_MODE_ATTITUDE:
    {
        PositionStateData curpos;
        PositionStateGet(&curpos);
        if (
            curpos.North < settings.GeoFenceBoxMin.North || curpos.North > settings.GeoFenceBoxMax.North ||
            curpos.East < settings.GeoFenceBoxMin.East || curpos.East > settings.GeoFenceBoxMax.East ||
            curpos.Down < settings.GeoFenceBoxMin.Down || curpos.Down > settings.GeoFenceBoxMax.Down) {
            // outside of safebox, fly back to safebox
            pathDesired.End.North        = (settings.GeoFenceBoxMin.North + settings.GeoFenceBoxMax.North) / 2.0f;
            pathDesired.End.East         = (settings.GeoFenceBoxMin.East + settings.GeoFenceBoxMax.East) / 2.0f;
            pathDesired.End.Down         = (settings.GeoFenceBoxMin.Down + settings.GeoFenceBoxMax.Down) / 2.0f;
            pathDesired.Start.North      = curpos.North;
            pathDesired.Start.East       = curpos.East;
            pathDesired.Start.Down       = curpos.Down;
            pathDesired.StartingVelocity = 1.0f;
            pathDesired.EndingVelocity   = 0.0f;
            pathDesired.Mode = PATHDESIRED_MODE_GOTOENDPOINT;
        } else {
            pathDesired.ModeParameters[0] = data->control[0];
            pathDesired.ModeParameters[1] = data->control[1];
            pathDesired.ModeParameters[2] = data->control[2];
            pathDesired.ModeParameters[3] = data->control[3];
            pathDesired.Mode = PATHDESIRED_MODE_FIXEDATTITUDE;
        }
    }
    break;
    }
    PathDesiredSet(&pathDesired);
    PoiLocationData poiLocation;
    PoiLocationGet(&poiLocation);
    poiLocation.North = data->poi[0];
    poiLocation.East  = data->poi[1];
    poiLocation.Down  = data->poi[2];
    PoiLocationSet(&poiLocation);
}

static void actuators_r_handler(__attribute__((unused)) struct ros_bridge *rb, rosbridgemessage_t *m)
{
    // only allow override of actuators if the apropriate flight mode command has been sent
    if (commandOverridePermitted != true) {
        return;
    }
    // only allow override of actutors if the craft is in ROS controlled mode
    FlightStatusFlightModeOptions mode;
    FlightStatusFlightModeGet(&mode);
    if (mode != FLIGHTSTATUS_FLIGHTMODE_ROSCONTROLLED) {
        return;
    }

    rosbridgemessage_actuators_t *data = (rosbridgemessage_actuators_t *)&(m->data);
    ActuatorOverrideChannelSet(&(data->pwm[0]));
    ActuatorDesiredUpdated(); // manually trigger faux update on actuatordesired - this will trigger Actuator module to process early and drive the outputs ASAP

    // WARNING, possible race condition. Actuators module currently has higher priority (device driver - IDLE+4) than the event dispatcher (flight control , IDLE+3)
    // which would trigger processing of the updated override object
    // however ROSBridge is running with an even lower task priority (AUXILLIARY, IDLE+1), which means the call to ActuatorOverrideChannelSet() should
    // preempt the code flow here and allow the UpdatedCb() on ActuatorOverride to finish executing before execution returns to this callback
    // to trigger the ActuatorDesired updated call
    // if this gets messed up, worst case, the update will be one cycle delayed, which shouldn't matter for anything but the most agile crafts
}

static void fullstate_estimate_r_handler(__attribute__((unused)) struct ros_bridge *rb, rosbridgemessage_t *m)
{
    // only allow override if no other sensor fusion is taking place
    RevoSettingsFusionAlgorithmOptions fusion;

    RevoSettingsFusionAlgorithmGet(&fusion);
    if (fusion != REVOSETTINGS_FUSIONALGORITHM_NONE) {
        return;
    }

    rosbridgemessage_fullstate_estimate_t *data = (rosbridgemessage_fullstate_estimate_t *)&(m->data);

    // safety
    if (!IS_REAL(data->quaternion[0])) {
        return;
    }
    if (!IS_REAL(data->quaternion[1])) {
        return;
    }
    if (!IS_REAL(data->quaternion[2])) {
        return;
    }
    if (!IS_REAL(data->quaternion[3])) {
        return;
    }
    if (!IS_REAL(data->rotation[0])) {
        return;
    }
    if (!IS_REAL(data->rotation[1])) {
        return;
    }
    if (!IS_REAL(data->rotation[2])) {
        return;
    }
    if (!IS_REAL(data->accessory[0])) {
        return;
    }
    if (!IS_REAL(data->accessory[1])) {
        return;
    }
    if (!IS_REAL(data->accessory[2])) {
        return;
    }
    if (!IS_REAL(data->accessory[3])) {
        return;
    }
    if (!IS_REAL(data->position[0])) {
        return;
    }
    if (!IS_REAL(data->position[1])) {
        return;
    }
    if (!IS_REAL(data->position[2])) {
        return;
    }
    if (!IS_REAL(data->velocity[0])) {
        return;
    }
    if (!IS_REAL(data->velocity[1])) {
        return;
    }
    if (!IS_REAL(data->velocity[2])) {
        return;
    }

    // attitude
    {
        AttitudeStateData s;
        AttitudeStateGet(&s);
        s.q1     = data->quaternion[0];
        s.q2     = data->quaternion[1];
        s.q3     = data->quaternion[2];
        s.q4     = data->quaternion[3];
        Quaternion2RPY(&s.q1, &s.Roll);
        s.NavYaw = s.Yaw;
        AttitudeStateSet(&s);
    }

    // gyroscopes
    {
        GyroStateData s;
        GyroStateGet(&s);
        s.x = data->rotation[0];
        s.y = data->rotation[1];
        s.z = data->rotation[2];
        GyroStateSet(&s);
    }

    // accelerometers
    {
        AccelStateData s;
        AccelStateGet(&s);
        s.x = data->accessory[0];
        s.y = data->accessory[1];
        s.z = data->accessory[2];
        AccelStateSet(&s);
    }

    float altitude;

    // position
    {
        PositionStateData s;
        PositionStateGet(&s);
        s.North  = data->position[0];
        s.East   = data->position[1];
        s.Down   = data->position[2];
        altitude = -s.Down;
        PositionStateSet(&s);
    }

    // airspeed
    {
        AirspeedStateData s;
        AirspeedStateGet(&s);
        s.CalibratedAirspeed = data->accessory[3];
    #define IAS2TAS(alt) (1.0f + (0.02f * (alt) / 304.8f))
        s.TrueAirspeed = s.CalibratedAirspeed * IAS2TAS(altitude);
        AirspeedStateSet(&s);
    }

    // velocity
    {
        VelocityStateData s;
        VelocityStateGet(&s);
        s.North  = data->velocity[0];
        s.East   = data->velocity[1];
        s.Down   = data->velocity[2];
        altitude = -s.Down;
        VelocityStateSet(&s);
    }
}

static void pos_estimate_r_handler(__attribute__((unused)) struct ros_bridge *rb, rosbridgemessage_t *m)
{
    rosbridgemessage_pos_estimate_t *data = (rosbridgemessage_pos_estimate_t *)&(m->data);

    // safety
    if (!IS_REAL(data->position[0])) {
        return;
    }
    if (!IS_REAL(data->position[1])) {
        return;
    }
    if (!IS_REAL(data->position[2])) {
        return;
    }

    AUXPositionSensorData pos;

    pos.North = data->position[0];
    pos.East  = data->position[1];
    pos.Down  = data->position[2];
    AUXPositionSensorSet(&pos);
}

static void vel_estimate_r_handler(__attribute__((unused)) struct ros_bridge *rb, rosbridgemessage_t *m)
{
    rosbridgemessage_vel_estimate_t *data = (rosbridgemessage_vel_estimate_t *)&(m->data);

    // safety
    if (!IS_REAL(data->velocity[0])) {
        return;
    }
    if (!IS_REAL(data->velocity[1])) {
        return;
    }
    if (!IS_REAL(data->velocity[2])) {
        return;
    }

    AUXVelocitySensorData vel;

    vel.North = data->velocity[0];
    vel.East  = data->velocity[1];
    vel.Down  = data->velocity[2];
    AUXVelocitySensorSet(&vel);
}

static void pong_handler(struct ros_bridge *rb, rosbridgemessage_t *m)
{
    rosbridgemessage_pingpong_t *data = (rosbridgemessage_pingpong_t *)&(m->data);


    data->sequence_number = rb->remotePingSequence;
}

static void fullstate_estimate_handler(__attribute__((unused)) struct ros_bridge *rb, rosbridgemessage_t *m)
{
    PositionStateData pos;
    VelocityStateData vel;
    AttitudeStateData att;
    FlightStatusFlightModeOptions mode;
    FlightStatusArmedOptions armed;
    float thrust;
    float airspeed;
    AccessoryDesiredData accessory;
    ManualControlCommandData manualcontrol;

    ManualControlCommandGet(&manualcontrol);
    FlightStatusArmedGet(&armed);
    FlightStatusFlightModeGet(&mode);
    ActuatorDesiredThrustGet(&thrust);
    PositionStateGet(&pos);
    VelocityStateGet(&vel);
    AttitudeStateGet(&att);
    AirspeedStateCalibratedAirspeedGet(&airspeed);
    rosbridgemessage_fullstate_estimate_t *data = (rosbridgemessage_fullstate_estimate_t *)&(m->data);
    data->quaternion[0] = att.q1;
    data->quaternion[1] = att.q2;
    data->quaternion[2] = att.q3;
    data->quaternion[3] = att.q4;
    data->position[0]   = pos.North;
    data->position[1]   = pos.East;
    data->position[2]   = pos.Down;
    data->velocity[0]   = vel.North;
    data->velocity[1]   = vel.East;
    data->velocity[2]   = vel.Down;
    data->rotation[0]   = ros->rateAccumulator[0];
    data->rotation[1]   = ros->rateAccumulator[1];
    data->rotation[2]   = ros->rateAccumulator[2];
    data->thrust = thrust;
    data->ROSControlled = (mode == FLIGHTSTATUS_FLIGHTMODE_ROSCONTROLLED) ? 1 : 0;
    data->FlightMode    = manualcontrol.FlightModeSwitchPosition;
    data->armed = (armed == FLIGHTSTATUS_ARMED_ARMED) ? 1 : 0;
    data->controls[0]   = manualcontrol.Roll;
    data->controls[1]   = manualcontrol.Pitch;
    data->controls[2]   = manualcontrol.Yaw;
    data->controls[3]   = manualcontrol.Thrust;
    data->controls[4]   = manualcontrol.Collective;
    data->controls[5]   = manualcontrol.Throttle;
    data->airspeed = airspeed;
    for (int t = 0; t < 4; t++) {
        if (AccessoryDesiredInstGet(t, &accessory) == 0) {
            data->accessory[t] = accessory.AccessoryVal;
        }
    }

    int x, y;
    float *P[13];
    INSGetPAddress(P);
    for (x = 0; x < 10; x++) {
        for (y = 0; y < 10; y++) {
            data->matrix[x * 10 + y] = P[x][y];
        }
    }
    if (ros->rateTimer >= 1) {
        float factor = 1.0f / (float)ros->rateTimer;
        data->rotation[0] *= factor;
        data->rotation[1] *= factor;
        data->rotation[2] *= factor;
        ros->rateAccumulator[0] = 0;
        ros->rateAccumulator[1] = 0;
        ros->rateAccumulator[2] = 0;
        ros->rateTimer = 0;
    }
}
static void imu_average_handler(__attribute__((unused)) struct ros_bridge *rb, __attribute__((unused)) rosbridgemessage_t *m)
{
    rosbridgemessage_imu_average_t *data = (rosbridgemessage_imu_average_t *)&(m->data);

    data->gyro_average[0]  = ros->rawGyrAccumulator[0];
    data->gyro_average[1]  = ros->rawGyrAccumulator[1];
    data->gyro_average[2]  = ros->rawGyrAccumulator[2];
    data->accel_average[0] = ros->rawAccAccumulator[0];
    data->accel_average[1] = ros->rawAccAccumulator[1];
    data->accel_average[2] = ros->rawAccAccumulator[2];
    data->gyrsamples = ros->gyrTimer;
    data->accsamples = ros->accTimer;
    if (ros->gyrTimer >= 1) {
        float factor = 1.0f / (float)ros->gyrTimer;
        data->gyro_average[0]    *= factor;
        data->gyro_average[1]    *= factor;
        data->gyro_average[2]    *= factor;
        ros->rawGyrAccumulator[0] = 0;
        ros->rawGyrAccumulator[1] = 0;
        ros->rawGyrAccumulator[2] = 0;
        ros->gyrTimer = 0;
    }
    if (ros->accTimer >= 1) {
        float factor = 1.0f / (float)ros->accTimer;
        data->accel_average[0]   *= factor;
        data->accel_average[1]   *= factor;
        data->accel_average[2]   *= factor;
        ros->rawAccAccumulator[0] = 0;
        ros->rawAccAccumulator[1] = 0;
        ros->rawAccAccumulator[2] = 0;
        ros->accTimer = 0;
    }
}
static void gyro_bias_handler(__attribute__((unused)) struct ros_bridge *rb, rosbridgemessage_t *m)
{
    rosbridgemessage_gyro_bias_t *data = (rosbridgemessage_gyro_bias_t *)&(m->data);

    data->gyro_bias[0] = ros->gyrBias[0];
    data->gyro_bias[1] = ros->gyrBias[1];
    data->gyro_bias[2] = ros->gyrBias[2];
}
static void gimbal_estimate_handler(__attribute__((unused)) struct ros_bridge *rb, __attribute__((unused)) rosbridgemessage_t *m)
{
    // TODO
}
static void actuators_handler(__attribute__((unused)) struct ros_bridge *rb, rosbridgemessage_t *m)
{
    rosbridgemessage_actuators_t *data = (rosbridgemessage_actuators_t *)&(m->data);

    ActuatorCommandChannelGet(&(data->pwm[0]));
}

static void autopilot_info_handler(__attribute__((unused)) struct ros_bridge *rb, rosbridgemessage_t *m)
{
    rosbridgemessage_autopilot_info_t *data = (rosbridgemessage_autopilot_info_t *)&(m->data);

    PathStatusData pathStatus;
    PathDesiredData pathDesired;
    VelocityDesiredData velocityDesired;

    PathStatusGet(&pathStatus);
    PathDesiredGet(&pathDesired);
    VelocityDesiredGet(&velocityDesired);
    data->status = pathStatus.Status;
    data->fractional_progress = pathStatus.fractional_progress;
    data->error  = pathStatus.error;
    data->pathDirection[0]  = pathStatus.path_direction_north;
    data->pathDirection[1]  = pathStatus.path_direction_east;
    data->pathDirection[2]  = pathStatus.path_direction_down;
    data->pathCorrection[0] = pathStatus.correction_direction_north;
    data->pathCorrection[1] = pathStatus.correction_direction_east;
    data->pathCorrection[2] = pathStatus.correction_direction_down;
    data->pathTime = pathStatus.path_time;
    data->Mode = pathDesired.Mode;
    data->ModeParameters[0] = pathDesired.ModeParameters[0];
    data->ModeParameters[1] = pathDesired.ModeParameters[1];
    data->ModeParameters[2] = pathDesired.ModeParameters[2];
    data->ModeParameters[3] = pathDesired.ModeParameters[3];
    data->Start[0] = pathDesired.Start.North;
    data->Start[1] = pathDesired.Start.East;
    data->Start[2] = pathDesired.Start.Down;
    data->End[0]   = pathDesired.End.North;
    data->End[1]   = pathDesired.End.East;
    data->End[2]   = pathDesired.End.Down;
    data->StartingVelocity   = pathDesired.StartingVelocity;
    data->EndingVelocity     = pathDesired.EndingVelocity;
    data->VelocityDesired[0] = velocityDesired.North;
    data->VelocityDesired[1] = velocityDesired.East;
    data->VelocityDesired[2] = velocityDesired.Down;
}

/**
 * Main task routine
 * @param[in] parameters parameter given by PIOS_Callback_Create()
 */
static void UAVOROSBridgeTxTask(void)
{
    uint8_t buffer[ROSBRIDGEMESSAGE_BUFFERSIZE]; // buffer on the stack? could also be in static RAM but not reuseale by other callbacks then
    rosbridgemessage_t *message = (rosbridgemessage_t *)buffer;

    for (rosbridgemessagetype_t type = ROSBRIDGEMESSAGE_PING; type < ROSBRIDGEMESSAGE_END_ARRAY_SIZE; type++) {
        if (ros->scheduled[type] && rosbridgemessagehandlers[type] != NULL) {
            message->magic       = ROSBRIDGEMAGIC;
            message->length      = ROSBRIDGEMESSAGE_SIZES[type];
            message->type        = type;
            message->timestamp   = PIOS_DELAY_GetuS();
            (*rosbridgemessagehandlers[type])(ros, message);
            message->crc32       = PIOS_CRC32_updateCRC(0xffffffff, message->data, message->length);
            int32_t ret = PIOS_COM_SendBufferNonBlocking(ros->com, buffer, offsetof(rosbridgemessage_t, data) + message->length);
            // int32_t ret = PIOS_COM_SendBuffer(ros->com, buffer, offsetof(rosbridgemessage_t, data) + message->length);
            ros->scheduled[type] = false;
            if (ret >= 0) {
                uint32_t txpackets;
                ROSBridgeStatusTxPacketsGet(&txpackets);
                txpackets++;
                ROSBridgeStatusTxPacketsSet(&txpackets);
            } else {
                uint32_t txfail;
                ROSBridgeStatusTxFailGet(&txfail);
                txfail++;
                ROSBridgeStatusTxFailSet(&txfail);
            }
            PIOS_CALLBACKSCHEDULER_Dispatch(callbackHandle);
            return;
        }
    }
    // canari: If in ROS mode and outside of skybox, maybe we haven't received any commands in too long
    // this prevents a flyaway as a last resort
    FlightStatusFlightModeOptions mode;
    FlightStatusFlightModeGet(&mode);
    if (mode == FLIGHTSTATUS_FLIGHTMODE_ROSCONTROLLED) {
        PositionStateData curpos;
        PositionStateGet(&curpos);
        if (
            curpos.North < settings.GeoFenceBoxMin.North || curpos.North > settings.GeoFenceBoxMax.North ||
            curpos.East < settings.GeoFenceBoxMin.East || curpos.East > settings.GeoFenceBoxMax.East ||
            curpos.Down < settings.GeoFenceBoxMin.Down || curpos.Down > settings.GeoFenceBoxMax.Down) {
            // outside of safebox, fly back to safebox
            PathDesiredData pathDesired;
            PathDesiredGet(&pathDesired);
            pathDesired.End.North        = (settings.GeoFenceBoxMin.North + settings.GeoFenceBoxMax.North) / 2.0f;
            pathDesired.End.East         = (settings.GeoFenceBoxMin.East + settings.GeoFenceBoxMax.East) / 2.0f;
            pathDesired.End.Down         = (settings.GeoFenceBoxMin.Down + settings.GeoFenceBoxMax.Down) / 2.0f;
            pathDesired.Start.North      = curpos.North;
            pathDesired.Start.East       = curpos.East;
            pathDesired.Start.Down       = curpos.Down;
            pathDesired.StartingVelocity = 1.0f;
            pathDesired.EndingVelocity   = 0.0f;
            pathDesired.Mode = PATHDESIRED_MODE_GOTOENDPOINT;
            commandOverridePermitted     = false;
            PathDesiredSet(&pathDesired);
        }
    }

    // nothing scheduled, rescheduling will be done by Event Callbacks
}

/**
 * Event Callback on Gyro updates (called with PIOS_SENSOR_RATE - roughly 500 Hz - from State Estimation)
 */
void RateCb(__attribute__((unused)) UAVObjEvent *ev)
{
    GyroStateData gyr;

    GyroStateGet(&gyr);
    ros->gyrBias[0] = gyr.x - ros->gyrRef[0];
    ros->gyrBias[1] = gyr.y - ros->gyrRef[1];
    ros->gyrBias[2] = gyr.z - ros->gyrRef[2];
    ros->rateAccumulator[0] += gyr.x;
    ros->rateAccumulator[1] += gyr.y;
    ros->rateAccumulator[2] += gyr.z;
    ros->rateTimer++;
}

/**
 * Event Callback on Gyro updates (called with PIOS_SENSOR_RATE - roughly 500 Hz - from State Estimation)
 */
void RawGyrCb(__attribute__((unused)) UAVObjEvent *ev)
{
    GyroSensorData gyr;

    GyroSensorGet(&gyr);
    ros->gyrRef[0] = gyr.x;
    ros->gyrRef[1] = gyr.y;
    ros->gyrRef[2] = gyr.z;
    ros->rawGyrAccumulator[0] += gyr.x;
    ros->rawGyrAccumulator[1] += gyr.y;
    ros->rawGyrAccumulator[2] += gyr.z;
    ros->gyrTimer++;
}

/**
 * Event Callback on Accel updates (called with PIOS_SENSOR_RATE - roughly 500 Hz - from State Estimation)
 */
void RawAccCb(__attribute__((unused)) UAVObjEvent *ev)
{
    AccelSensorData acc;

    AccelSensorGet(&acc);
    ros->rawAccAccumulator[0] += acc.x;
    ros->rawAccAccumulator[1] += acc.y;
    ros->rawAccAccumulator[2] += acc.z;
    ros->accTimer++;
    ros->scheduled[ROSBRIDGEMESSAGE_IMU_AVERAGE] = true;
    if (callbackHandle) {
        PIOS_CALLBACKSCHEDULER_Dispatch(callbackHandle);
    }
}

/**
 * Event Callback on Attitude updates (called with PIOS_SENSOR_RATE - roughly 500 Hz - from State Estimation)
 */
void AttitudeCb(__attribute__((unused)) UAVObjEvent *ev)
{
    bool dispatch = false;

    if (++ros->pingTimer >= settings.UpdateRate.Ping && settings.UpdateRate.Ping > 0) {
        ros->pingTimer = 0;
        dispatch = true;
        ros->scheduled[ROSBRIDGEMESSAGE_PING] = true;
    }
    if (++ros->stateTimer >= settings.UpdateRate.State && settings.UpdateRate.State > 0) {
        ros->stateTimer = 0;
        dispatch = true;
        ros->scheduled[ROSBRIDGEMESSAGE_FULLSTATE_ESTIMATE] = true;
        ros->scheduled[ROSBRIDGEMESSAGE_ACTUATORS] = true;
    }
    if (++ros->autopilotTimer >= settings.UpdateRate.Autopilot && settings.UpdateRate.Autopilot > 0) {
        ros->autopilotTimer = 0;
        dispatch = true;
        ros->scheduled[ROSBRIDGEMESSAGE_AUTOPILOT_INFO] = true;
    }
    if (++ros->biasTimer >= settings.UpdateRate.Bias && settings.UpdateRate.Bias > 0) {
        ros->biasTimer = 0;
        dispatch = true;
        ros->scheduled[ROSBRIDGEMESSAGE_GYRO_BIAS] = true;
    }
    if (dispatch && callbackHandle) {
        PIOS_CALLBACKSCHEDULER_Dispatch(callbackHandle);
    }
}

/**
 * Event Callback on Settings
 */
void SettingsCb(__attribute__((unused)) UAVObjEvent *ev)
{
    ROSBridgeSettingsGet(&settings);
}

/**
 * Main task routine
 * @param[in] parameters parameter given by PIOS_Thread_Create()
 */
static void UAVOROSBridgeRxTask(__attribute__((unused)) void *parameters)
{
    while (1) {
        uint8_t b = 0;
        uint16_t count = PIOS_COM_ReceiveBuffer(ros->com, &b, 1, ~0);
        if (count) {
            ros_receive_byte(ros, b);
        }
    }
}

#endif // PIOS_INCLUDE_ROS_BRIDGE
/**
 * @}
 * @}
 */
