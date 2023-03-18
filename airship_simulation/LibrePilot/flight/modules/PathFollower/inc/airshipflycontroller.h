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
#ifndef AIRSHIPFLYCONTROLLER_H
#define AIRSHIPFLYCONTROLLER_H
#include "pathfollowercontrol.h"

class AirshipFlyController : public PathFollowerControl {
protected:
    static AirshipFlyController *p_inst;
    AirshipFlyController();


public:
    static AirshipFlyController *instance()
    {
        if (!p_inst) {
            p_inst = new AirshipFlyController();
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
    void updatePathVelocity(float kFF, bool limited);
    uint8_t updateFixedDesiredAttitude();
    bool correctCourse(float *C, float *V, float *F, float s);
    int32_t lastAirspeedUpdate;

    struct pid PIDposH[2];
    struct pid PIDposV;
    struct pid PIDalt;
    struct pid PIDpower;
};

#endif // AIRSHIPFLYCONTROLLER_H
