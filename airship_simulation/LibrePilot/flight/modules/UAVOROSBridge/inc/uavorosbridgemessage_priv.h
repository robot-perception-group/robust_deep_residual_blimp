/**
 ******************************************************************************
 * @addtogroup LibrePilotModules LibrePilot Modules
 * @{
 * @addtogroup UAVOROSBridgeModule ROS Bridge Module
 * @{
 *
 * @file       uavorosbridgemessage_priv.h
 * @author     The LibrePilot Project, http://www.librepilot.org Copyright (C) 2016.
 *             Max Planck Institute for intelligent systems, http://www.is.mpg.de Copyright (C) 2016.
 * @brief      Message definition for UAVO ROS Bridge
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

#ifndef UAVOROSBRIDGEMESSAGE_H
#define UAVOROSBRIDGEMESSAGE_H

#define ROSBRIDGEMAGIC 0x76543210

typedef enum {
    ROSBRIDGEMESSAGE_PING,
    ROSBRIDGEMESSAGE_POS_ESTIMATE,
    ROSBRIDGEMESSAGE_FLIGHTCONTROL,
    ROSBRIDGEMESSAGE_GIMBALCONTROL,
    ROSBRIDGEMESSAGE_PONG,
    ROSBRIDGEMESSAGE_FULLSTATE_ESTIMATE,
    ROSBRIDGEMESSAGE_IMU_AVERAGE,
    ROSBRIDGEMESSAGE_GYRO_BIAS,
    ROSBRIDGEMESSAGE_GIMBAL_ESTIMATE,
    ROSBRIDGEMESSAGE_VEL_ESTIMATE,
    ROSBRIDGEMESSAGE_ACTUATORS,
    ROSBRIDGEMESSAGE_AUTOPILOT_INFO,
    ROSBRIDGEMESSAGE_END_ARRAY_SIZE,
} rosbridgemessagetype_t;

struct rosbridgemessage_s {
    uint32_t magic;
    uint32_t length;
    uint32_t timestamp;
    uint32_t type;
    uint32_t crc32;
    uint8_t  data[];
};

typedef struct rosbridgemessage_s rosbridgemessage_t;

typedef struct {
    uint8_t sequence_number;
} rosbridgemessage_pingpong_t;

typedef struct {
    float position[3];
} rosbridgemessage_pos_estimate_t;

typedef struct {
    float velocity[3];
} rosbridgemessage_vel_estimate_t;

typedef enum {
    ROSBRIDGEMESSAGE_FLIGHTCONTROL_MODE_ATTITUDE,
    ROSBRIDGEMESSAGE_FLIGHTCONTROL_MODE_WAYPOINT,
    ROSBRIDGEMESSAGE_FLIGHTCONTROL_MODE_VECTOR,
    ROSBRIDGEMESSAGE_FLIGHTCONTROL_MODE_ACTUATORS,
    ROSBRIDGEMESSAGE_FLIGHTCONTROL_MODE_ROSCONTROL,
    ROSBRIDGEMESSAGE_FLIGHTCONTROL_MODE_NUMELEM,
} rosbridgemessage_flightcontrol_mode_t;

typedef struct {
    float    control[4];
    float    vel[4];
    float    poi[3];
    uint32_t mode;
} rosbridgemessage_flightcontrol_t;

typedef struct {
    float control[3];
} rosbridgemessage_gimbalcontrol_t;

typedef struct {
    float   quaternion[4];
    float   position[3];
    float   velocity[3];
    float   rotation[3];
    float   thrust;
    int32_t ROSControlled;
    int32_t FlightMode;
    int32_t armed;
    float   controls[6];
    float   accessory[4];
    float   airspeed;
    float   matrix[100];
} rosbridgemessage_fullstate_estimate_t;

typedef struct {
    uint32_t gyrsamples;
    uint32_t accsamples;
    float    gyro_average[3];
    float    accel_average[3];
} rosbridgemessage_imu_average_t;

typedef struct {
    float gyro_bias[3];
} rosbridgemessage_gyro_bias_t;

typedef struct {
    float quaternion[4];
    float qvar[4];
} rosbridgemessage_gimbal_estimate_t;

typedef struct {
    int16_t pwm[12];
} rosbridgemessage_actuators_t;

typedef struct {
    int32_t status;
    float   fractional_progress;
    float   error;
    float   pathDirection[3];
    float   pathCorrection[3];
    float   pathTime;
    int32_t Mode;
    float   ModeParameters[4];
    float   Start[3];
    float   End[3];
    float   StartingVelocity;
    float   EndingVelocity;
    float   VelocityDesired[3];
} rosbridgemessage_autopilot_info_t;

static const size_t ROSBRIDGEMESSAGE_SIZES[ROSBRIDGEMESSAGE_END_ARRAY_SIZE] = {
    sizeof(rosbridgemessage_pingpong_t),
    sizeof(rosbridgemessage_pos_estimate_t),
    sizeof(rosbridgemessage_flightcontrol_t),
    sizeof(rosbridgemessage_gimbalcontrol_t),
    sizeof(rosbridgemessage_pingpong_t),
    sizeof(rosbridgemessage_fullstate_estimate_t),
    sizeof(rosbridgemessage_imu_average_t),
    sizeof(rosbridgemessage_gyro_bias_t),
    sizeof(rosbridgemessage_gimbal_estimate_t),
    sizeof(rosbridgemessage_vel_estimate_t),
    sizeof(rosbridgemessage_actuators_t),
    sizeof(rosbridgemessage_autopilot_info_t),
};

#define ROSBRIDGEMESSAGE_BUFFERSIZE (offsetof(rosbridgemessage_t, data) + sizeof(rosbridgemessage_fullstate_estimate_t) + 2)

struct ros_bridge;

typedef void (rosbridgemessage_handler (struct ros_bridge *rb, rosbridgemessage_t *message));

#endif // UAVOROSBRIDGEMESSAGE_H

/**
 * @}
 * @}
 */
