/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup PIOS_SDP3x SDP3x Differential pressure sensors
 * @brief Hardware functions to deal with the Eagle Tree Airspeed MicroSensor V3
 * @{
 *
 * @file       pios_stp3x.c
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

#include "pios.h"

#ifdef PIOS_INCLUDE_SDP3X

static bool PIOS_SDP3X_WriteCommand(uint16_t command)
{
    uint8_t commandbuffer[2];

    commandbuffer[0] = (uint8_t)((uint16_t)(command >> 8));
    commandbuffer[1] = (uint8_t)((uint16_t)(command & 0xff));
    const struct pios_i2c_txn txn_list[] = {
        {
            .info = __func__,
            .addr = SDP3X_I2C_ADDR1,
            .rw   = PIOS_I2C_TXN_WRITE,
            .len  = 2,
            .buf  = commandbuffer,
        }
    };

    return PIOS_I2C_Transfer(PIOS_I2C_SDP3X_ADAPTER, txn_list, NELEMENTS(txn_list));
}

static bool PIOS_SDP3X_Read(uint8_t *buffer, uint8_t len)
{
    const struct pios_i2c_txn txn_list[] = {
        {
            .info = __func__,
            .addr = SDP3X_I2C_ADDR1,
            .rw   = PIOS_I2C_TXN_READ,
            .len  = len,
            .buf  = buffer,
        }
    };

    for (uint8_t retry = PIOS_SDP3X_RETRY_LIMIT; retry > 0; --retry) {
        if (PIOS_I2C_Transfer(PIOS_I2C_SDP3X_ADAPTER, txn_list, NELEMENTS(txn_list)) == 0) {
            return 0;
        }
    }
    return -1;
}

uint8_t PIOS_SDP3X_crc(const uint8_t buffer[], unsigned len, uint8_t crc)
{
    uint8_t c = SDP3X_CRC_INIT;

    for (uint8_t i = 0; i < len; i++) {
        c ^= buffer[i];
        for (uint8_t b = 8; b > 0; --b) {
            if (c & 0x80) {
                c = (c << 1) ^ SDP3X_CRC_POLY;
            } else {
                c = c << 1;
            }
        }
    }
    return c == crc;
}

void PIOS_SDP3X_ReadAirspeed(volatile struct PIOS_SDP3X_STATE *state)
{
    PIOS_Assert(state);

    if (state->status == SDP3X_STATUS_NOTFOUND) {
        if (PIOS_SDP3X_WriteCommand(SDP3X_CONT_MODE_STOP) == 0) {
            PIOS_DELAY_WaituS(500);
            if (PIOS_SDP3X_WriteCommand(SDP3X_CONT_MEAS_AVG_MODE) == 0) {
                state->status = SDP3X_STATUS_CONFIGURED;
            }
        }
        return;
    }
    if (state->status == SDP3X_STATUS_CONFIGURED || state->status == SDP3X_STATUS_READY) {
        state->status = SDP3X_STATUS_READY;
        uint8_t airspeed_raw[9];
        if (PIOS_SDP3X_Read(airspeed_raw, sizeof(airspeed_raw)) == 0) {
            // check checksum
            if (!PIOS_SDP3X_crc(&airspeed_raw[0], 2, airspeed_raw[2]) ||
                !PIOS_SDP3X_crc(&airspeed_raw[0], 2, airspeed_raw[2]) ||
                !PIOS_SDP3X_crc(&airspeed_raw[0], 2, airspeed_raw[2])) {
// CRC error
                state->status = SDP3X_STATUS_CONFIGURED;
                return;
            }

            state->pressure    = (int16_t)((((uint16_t)airspeed_raw[0]) << 8) + (uint16_t)airspeed_raw[1]);
            state->temperature = (int16_t)((((uint16_t)airspeed_raw[3]) << 8) + (uint16_t)airspeed_raw[4]);
            state->scale = (int16_t)((((uint16_t)airspeed_raw[6]) << 8) + (uint16_t)airspeed_raw[7]);
        } else {
            state->status = SDP3X_STATUS_NOTFOUND;
        }
    }
}

#endif /* PIOS_INCLUDE_SDP3X */
