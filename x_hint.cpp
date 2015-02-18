/* Copyright (C) 2015  Matteo Hausner
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include "XPLMDataAccess.h"
#include "XPLMDisplay.h"
#include "XPLMGraphics.h"
#include "XPLMProcessing.h"

#include <stdio.h>
#include <string.h>

// define name
#define NAME "X-hint"
#define NAME_LOWERCASE "x_hint"

// define version
#define VERSION "0.1"

// define hint duration
#define HINT_DURARTION 2.0f

// global dataref variables
static XPLMDataRef barometerSettingInHgPilotDataRef = NULL, barometerSettingInHgCopilotDataRef = NULL, nav1ObsDegMagPilotDataRef = NULL, nav2ObsDegMagPilotDataRef = NULL, nav1ObsDegMagCopilotDataRef = NULL, nav2ObsDegMagCopilotDataRef = NULL;

// global internal variables
static float lastHintTime = 0.0f, lastBarometerSettingInHgPilot = 0.0f, lastBarometerSettingInHgCopilot = 0.0f, lastNav1ObsDegMagPilot = 0.0f, lastNav2ObsDegMagPilot = 0.0f, lastNav1ObsDegMagCopilot = 0.0f, lastNav2ObsDegMagCopilot = 0.0f;
static char hintText[32];

static void DisplayBarometerHint(float barometerSettingInHg)
{
    sprintf(hintText, "%.2f INGH / %.0f MB", barometerSettingInHg, barometerSettingInHg * 33.8638866667f);
    lastHintTime = XPLMGetElapsedTime();
}

static void DisplayObsHint(float obs)
{
    sprintf(hintText, "%.0f DEG", obs);
    lastHintTime = XPLMGetElapsedTime();
}

// flightloop-callback that handles which hint is when displayed
static float FlightLoopCallback(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, int inCounter, void *inRefcon)
{
    float barometerSettingInHgPilot = XPLMGetDataf(barometerSettingInHgPilotDataRef);
    float barometerSettingInHgCopilot = XPLMGetDataf(barometerSettingInHgCopilotDataRef);
    float nav1ObsDegMagPilot = XPLMGetDataf(nav1ObsDegMagPilotDataRef);
    float nav2ObsDegMagPilot = XPLMGetDataf(nav2ObsDegMagPilotDataRef);
    float nav1ObsDegMagCopilot = XPLMGetDataf(nav1ObsDegMagCopilotDataRef);
    float nav2ObsDegMagCopilot = XPLMGetDataf(nav2ObsDegMagCopilotDataRef);

    if (lastBarometerSettingInHgPilot != 0 && barometerSettingInHgPilot != lastBarometerSettingInHgPilot)
        DisplayBarometerHint(barometerSettingInHgPilot);
    else if (lastBarometerSettingInHgCopilot != 0 && barometerSettingInHgCopilot != lastBarometerSettingInHgCopilot)
        DisplayBarometerHint(barometerSettingInHgCopilot);
    else if (lastBarometerSettingInHgCopilot != 0 && barometerSettingInHgCopilot != lastBarometerSettingInHgCopilot)
        DisplayBarometerHint(barometerSettingInHgCopilot);

    lastBarometerSettingInHgPilot = barometerSettingInHgPilot;
    lastBarometerSettingInHgCopilot = barometerSettingInHgCopilot;
    lastNav1ObsDegMagPilot = nav1ObsDegMagPilot;
    lastNav2ObsDegMagPilot = nav2ObsDegMagPilot;
    lastNav1ObsDegMagCopilot = nav1ObsDegMagCopilot;
    lastNav2ObsDegMagCopilot = nav2ObsDegMagCopilot;

    return -1.0f;
}

// draw-callback that performs the actual drawing of the hint
static int DrawCallback(XPLMDrawingPhase inPhase, int inIsBefore, void *inRefcon)
{
    if (XPLMGetElapsedTime() - lastHintTime <= HINT_DURARTION)
    {
        float color[] = { 1.0f, 1.0f, 1.0f };
        int x = 0, y = 0;
        XPLMGetMouseLocation(&x, &y);
        XPLMDrawString(color, x - 25, y - 25, hintText, NULL, xplmFont_Basic);
    }
}

PLUGIN_API int XPluginStart(char *outName, char *outSig, char *outDesc)
{
    // set plugin info
    strcpy(outName, NAME);
    strcpy(outSig, "de.bwravencl."NAME_LOWERCASE);
    strcpy(outDesc, NAME" simpliefies handling X-Plane by adding tooltips!");

    // obtain datarefs
    barometerSettingInHgPilotDataRef = XPLMFindDataRef("sim/cockpit2/gauges/actuators/barometer_setting_in_hg_pilot");
    barometerSettingInHgCopilotDataRef = XPLMFindDataRef("sim/cockpit2/gauges/actuators/barometer_setting_in_hg_copilot");
    nav1ObsDegMagPilotDataRef = XPLMFindDataRef("sim/cockpit2/radios/actuators/nav1_obs_deg_mag_pilot");
    nav2ObsDegMagPilotDataRef = XPLMFindDataRef("sim/cockpit2/radios/actuators/nav2_obs_deg_mag_pilot");
    nav1ObsDegMagCopilotDataRef = XPLMFindDataRef("sim/cockpit2/radios/actuators/nav1_obs_deg_mag_copilot");
    nav2ObsDegMagCopilotDataRef = XPLMFindDataRef("sim/cockpit2/radios/actuators/nav2_obs_deg_mag_copilot");

    // register flight loop callback
    XPLMRegisterFlightLoopCallback(FlightLoopCallback, -1, NULL);

    // register draw callback
    XPLMRegisterDrawCallback(DrawCallback, xplm_Phase_Window, 1, NULL);

    return 1;
}

PLUGIN_API void	XPluginStop(void)
{
}

PLUGIN_API void XPluginDisable(void)
{
}

PLUGIN_API int XPluginEnable(void)
{
    return 1;
}

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFromWho, long inMessage, void *inParam)
{
}
