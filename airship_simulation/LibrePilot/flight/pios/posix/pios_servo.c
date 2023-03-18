/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup   PIOS_SERVO RC Servo Functions
 * @brief Code to do set RC servo output
 * @{
 *
 * @file       pios_servo.c
 * @author     The LibrePilot Project, http://www.librepilot.org Copyright (C) 2016.
 *             The OpenPilot Team, http://www.openpilot.org Copyright (C) 2012.
 * @brief      RC Servo routines (STM32 dependent)
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
/*
 * DShot: Tribute belongs to dRonin, http://dRonin.org/ for sparking the idea of
 * using gpio bitbang as more general solution over using timer dma.
 */

#include "pios.h"

#ifdef PIOS_INCLUDE_SERVO

static const struct pios_servo_cfg *servo_cfg;

extern void PIOS_Servo_SetActive(__attribute__((unused)) uint32_t active)
{}

extern void PIOS_Servo_Disable()
{}

static void PIOS_Servo_SetupBank(__attribute__((unused)) uint8_t bank_nr)
{}

extern void PIOS_Servo_Enable()
{}

void PIOS_Servo_DSHot_Rate(__attribute__((unused)) uint32_t rate_in_khz)
{}

/**
 * Initialise Servos
 */
int32_t PIOS_Servo_Init(const struct pios_servo_cfg *cfg)
{
    servo_cfg = cfg;
    return 0;
}

void PIOS_Servo_SetBankMode(__attribute__((unused)) uint8_t bank, __attribute__((unused)) uint8_t mode)
{}

static void PIOS_Servo_DShot_Update()
{}


void PIOS_Servo_Update()
{}
/**
 * Set the servo update rate (Max 500Hz)
 * \param[in] array of rates in Hz
 * \param[in] array of timer clocks in Hz
 * \param[in] maximum number of banks
 */
void PIOS_Servo_SetHz(__attribute__((unused)) const uint16_t *speeds, __attribute__((unused))  const uint32_t *clock, __attribute__((unused))  uint8_t banks)
{}

/**
 * Set servo position
 * \param[in] Servo Servo number (0-7)
 * \param[in] Position Servo position in microseconds
 */
void PIOS_Servo_Set(__attribute__((unused)) uint8_t servo, __attribute__((unused))  uint16_t position)
{}

uint8_t PIOS_Servo_GetPinBank(__attribute__((unused)) uint8_t pin)
{
    return 0;
}

const struct pios_servo_cfg *PIOS_Servo_GetConfig()
{
    return servo_cfg;
}

#endif /* PIOS_INCLUDE_SERVO */
