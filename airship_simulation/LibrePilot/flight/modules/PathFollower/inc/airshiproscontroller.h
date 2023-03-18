/**
 ******************************************************************************
 * @addtogroup LibrePilotModules LibrePilot Modules
 * @{
 * @addtogroup Airship CONTROL interface class
 * @brief CONTROL interface class for pathfollower fixed wing fly controller
 * @{
 *
 * @file       fixedwingflycontroller.h
 * @author     The LibrePilot Project, http://www.librepilot.org Copyright (C) 2016.
 *             The OpenPilot Team, http://www.openpilot.org Copyright (C) 2015.
 * @brief      Executes CONTROL for fixed wing fly objectives
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
#ifndef AIRSHIPROSCONTROLLER_H
#define AIRSHIPROSCONTROLLER_H
#include "pathfollowercontrol.h"

class AirshipROSController : public PathFollowerControl {
protected:
    static AirshipROSController *p_inst;
    AirshipROSController();


public:
    static AirshipROSController *instance()
    {
        if (!p_inst) {
            p_inst = new AirshipROSController();
        }
        return p_inst;
    }

    int32_t Initialize(AirshipPathFollowerSettingsData *airshipSettings);
    void Activate(void);
    void Deactivate(void);
    void SettingsUpdated(void);
    void UpdateAutoPilot(void);
    void ObjectiveUpdated(void);
    uint8_t IsActive(void);
    uint8_t Mode(void);
    void AirspeedStateUpdatedCb(__attribute__((unused)) UAVObjEvent * ev);

protected:
    AirshipPathFollowerSettingsData *airshipSettings;

    uint8_t mActive;
    uint8_t mMode;
    // correct speed by measured airspeed
    float indicatedAirspeedStateBias;
    // to prevent oscillations on turn-around
    float previousCourseError;
private:
    void resetGlobals();
    uint8_t updateAutoPilotAirship();
    int32_t lastAirspeedUpdate;

    struct pid PIDalt;
    struct pid PIDpower;
};

#endif // AIRSHIPROSCONTROLLER_H
