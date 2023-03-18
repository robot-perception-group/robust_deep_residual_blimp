/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup PIOS_SDP3x SDP3x Differential pressure sensors
 * @brief Hardware functions to deal with the Eagle Tree Airspeed MicroSensor V3
 * @{
 *
 * @file       pios_sdp3x.h
 * @author     The LibrePilot Team, http://www.librepilot.org Copyright (C) 2021.
 * @brief      SDP3x Airspeed Sensor Driver
 * @see        The GNU Public License (GPL) Version 3
 *
 ******************************************************************************/
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

#ifndef PIOS_SDP3X_H
#define PIOS_SDP3X_H

#define SDP3X_I2C_ADDR1            0x21
#define SDP3X_I2C_ADDR2            0x22
#define SDP3X_I2C_ADDR3            0x23

#define SDP3X_SCALE_TEMPERATURE    200.0f
#define SDP3X_RESET_ADDR           0x00
#define SDP3X_RESET_CMD            0x06
#define SDP3X_CONT_MEAS_AVG_MODE   0x3615
#define SDP3X_CONT_MODE_STOP       0x3FF9

#define SDP3X_SCALE_PRESSURE_SDP31 60
#define SDP3X_SCALE_PRESSURE_SDP32 240
#define SDP3X_SCALE_PRESSURE_SDP33 20
#define SDP3X_CRC_INIT             0xff
#define SDP3X_CRC_POLY             0x31

#define SDP3X_CONVERSION_INTERVAL  (1000000 / SDP3X_MEAS_RATE)     /* microseconds */

#define PIOS_SDP3X_RETRY_LIMIT     3
struct PIOS_SDP3X_STATE {
    enum { SDP3X_STATUS_NOTFOUND, SDP3X_STATUS_CONFIGURED, SDP3X_STATUS_READY } status;
    int16_t pressure;
    int16_t temperature;
    int16_t scale;
};

void PIOS_SDP3X_ReadAirspeed(volatile struct PIOS_SDP3X_STATE *state);

#endif /* PIOS_SDP3X_H */
