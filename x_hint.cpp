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
#include "XPLMPlugin.h"
#include "XPLMProcessing.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

// define name
#define NAME "X-hint"
#define NAME_LOWERCASE "x_hint"

// define version
#define VERSION "0.3"

// define QPAC A320 plugin signature
#define QPAC_A320_PLUGIN_SIGNATURE "QPAC.airbus.fbw"

// define hint duration
#define HINT_DURATION 4.0f

// global dataref variables
static XPLMDataRef dgDriftVacDegDataRef = NULL, dgDriftEleDegDataRef = NULL, dgDriftVac2DegDataRef = NULL, dgDriftEle2DegDataRef = NULL, headingDialDegMagPilotDataRef = NULL, headingDialDegMagCopilotDataRef = NULL, barometerSettingInHgPilotDataRef = NULL, barometerSettingInHgCopilotDataRef = NULL, adf1CardHeadingDegMagPilotDataRef = NULL, adf2CardHeadingDegMagPilotDataRef = NULL, adf1CardHeadingDegMagCopilotDataRef = NULL, adf2CardHeadingDegMagCopilotDataRef = NULL, hsiObsDegMagPilotDataRef = NULL, hsiObsDegMagCopilotDataRef = NULL, nav1ObsDegMagPilotDataRef = NULL, nav2ObsDegMagPilotDataRef = NULL, nav1ObsDegMagCopilotDataRef = NULL, nav2ObsDegMagCopilotDataRef = NULL;

// global internal variables
static char hintText[32] = "";
static int bringFakeWindowToFront = 0, lastChangeDetected = 0, forceDisplay = 0;
static float lastMouseUsageTime = 0.0f, lastHintTime = 0.0f, lastDgDriftVacDeg = FLT_MAX, lastDgDriftEleDeg = FLT_MAX, lastDgDriftVac2Deg = FLT_MAX, lastDgDriftEle2Deg = FLT_MAX, lastHeadingDialDegMagPilot = FLT_MAX, lastHeadingDialDegMagCopilot = FLT_MAX, lastBarometerSettingInHgPilot = FLT_MAX, lastBarometerSettingInHgCopilot = FLT_MAX, lastAdf1CardHeadingDegMagPilot = FLT_MAX, lastAdf2CardHeadingDegMagPilot = FLT_MAX, lastAdf1CardHeadingDegMagCopilot = FLT_MAX, lastAdf2CardHeadingDegMagCopilot = FLT_MAX, lastHsiObsDegMagPilot = FLT_MAX, lastHsiObsDegMagCopilot = FLT_MAX, lastNav1ObsDegMagPilot = FLT_MAX, lastNav2ObsDegMagPilot = FLT_MAX, lastNav1ObsDegMagCopilot = FLT_MAX, lastNav2ObsDegMagCopilot = FLT_MAX;
static XPLMWindowID fakeWindow = NULL;

// flightloop-callback that resizes and brings the fake window back to the front if needed
static float UpdateFakeWindowCallback(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, int inCounter, void *inRefcon)
{
    if (fakeWindow != NULL)
    {
        int x = 0, y = 0;
        XPLMGetScreenSize(&x, &y);
        XPLMSetWindowGeometry(fakeWindow, 0, y, x, 0);

        if (bringFakeWindowToFront == 0)
        {
            XPLMBringWindowToFront(fakeWindow);
            bringFakeWindowToFront = 1;
        }
    }

    return -1.0f;
}

// check if a plugin with a given signature is enabled
static int IsPluginEnabled(const char* pluginSignature)
{
    XPLMPluginID pluginId = XPLMFindPluginBySignature(pluginSignature);

    return XPLMIsPluginEnabled(pluginId);
}

// if the given value is beyond the range a value that is inside the given range is returned - the behavior resembles integer underflows / overflows occured - values inside the given range are simply returned
static float HandleOverflow(float value, float min, float max)
{
    float a = max - min;
    float b = fabs(value - max);
    float r = b - a * (float) ((int) b / (int) a);

    if (value < min)
        return max - r;
    else if (value > max)
        return min + r;
    else
        return value;
}

// display a hint showing a drift between -180 and 180 degrees
static void DisplayDriftHint(float degrees)
{
    if (IsPluginEnabled(QPAC_A320_PLUGIN_SIGNATURE) == 0)
    {
        sprintf(hintText, "%.1f deg", HandleOverflow(degrees, -180.0f, 180.0f));
        lastHintTime = XPLMGetElapsedTime();
    }
}

// display a hint showing a barometer setting
static void DisplayBarometerHint(float barometerSettingInHg)
{
    sprintf(hintText, "%.2f inHg / %.0f mb", barometerSettingInHg, barometerSettingInHg * 33.8638866667f);
    lastHintTime = XPLMGetElapsedTime();
}

// display a hint showing a heading between 0 and 360 degrees
static void DisplayHeadingHint(float degrees)
{
    if (IsPluginEnabled(QPAC_A320_PLUGIN_SIGNATURE) == 0)
    {
        sprintf(hintText, "%.0f deg", HandleOverflow(degrees, 0.0f, 360.0f));
        lastHintTime = XPLMGetElapsedTime();
    }
}

// flightloop-callback that handles which hint is displayed when
static float FlightLoopCallback(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, int inCounter, void *inRefcon)
{
    float dgDriftVacDeg = XPLMGetDataf(dgDriftVacDegDataRef);
    float dgDriftEleDeg = XPLMGetDataf(dgDriftEleDegDataRef);
    float dgDriftVac2Deg = XPLMGetDataf(dgDriftVac2DegDataRef);
    float dgDriftEle2Deg = XPLMGetDataf(dgDriftEle2DegDataRef);
    float headingDialDegMagPilot = XPLMGetDataf(headingDialDegMagPilotDataRef);
    float headingDialDegMagCopilot = XPLMGetDataf(headingDialDegMagCopilotDataRef);
    float barometerSettingInHgPilot = XPLMGetDataf(barometerSettingInHgPilotDataRef);
    float barometerSettingInHgCopilot = XPLMGetDataf(barometerSettingInHgCopilotDataRef);
    float adf1CardHeadingDegMagPilot = XPLMGetDataf(adf1CardHeadingDegMagPilotDataRef);
    float adf2CardHeadingDegMagPilot= XPLMGetDataf(adf2CardHeadingDegMagPilotDataRef);
    float adf1CardHeadingDegMagCopilot= XPLMGetDataf(adf1CardHeadingDegMagCopilotDataRef);
    float adf2CardHeadingDegMagCopilot= XPLMGetDataf(adf2CardHeadingDegMagCopilotDataRef);
    float hsiObsDegMagPilot = XPLMGetDataf(hsiObsDegMagPilotDataRef);
    float hsiObsDegMagCopilot = XPLMGetDataf(hsiObsDegMagCopilotDataRef);
    float nav1ObsDegMagPilot = XPLMGetDataf(nav1ObsDegMagPilotDataRef);
    float nav2ObsDegMagPilot = XPLMGetDataf(nav2ObsDegMagPilotDataRef);
    float nav1ObsDegMagCopilot = XPLMGetDataf(nav1ObsDegMagCopilotDataRef);
    float nav2ObsDegMagCopilot = XPLMGetDataf(nav2ObsDegMagCopilotDataRef);

    int changeDetected = 1;

    if (lastDgDriftVacDeg != FLT_MAX && fabs(dgDriftVacDeg - lastDgDriftVacDeg) > 0.01f)
        DisplayDriftHint(dgDriftVacDeg);
    else if (lastDgDriftEleDeg != FLT_MAX && fabs(dgDriftEleDeg - lastDgDriftEleDeg) > 0.01f)
        DisplayDriftHint(dgDriftEleDeg);
    else if (lastDgDriftVac2Deg != FLT_MAX && fabs(dgDriftVac2Deg - lastDgDriftVac2Deg) > 0.01f)
        DisplayDriftHint(dgDriftVac2Deg);
    else if (lastDgDriftEle2Deg != FLT_MAX && fabs(dgDriftEle2Deg - lastDgDriftEle2Deg) > 0.01f)
        DisplayDriftHint(dgDriftEle2Deg);
    else if (lastHeadingDialDegMagPilot != FLT_MAX && headingDialDegMagPilot != lastHeadingDialDegMagPilot)
        DisplayHeadingHint(headingDialDegMagPilot);
    else if (lastHeadingDialDegMagCopilot != FLT_MAX && headingDialDegMagCopilot != lastHeadingDialDegMagCopilot)
        DisplayHeadingHint(headingDialDegMagCopilot);
    else if (lastBarometerSettingInHgPilot != FLT_MAX && barometerSettingInHgPilot != lastBarometerSettingInHgPilot)
        DisplayBarometerHint(barometerSettingInHgPilot);
    else if (lastBarometerSettingInHgCopilot != FLT_MAX && barometerSettingInHgCopilot != lastBarometerSettingInHgCopilot)
        DisplayBarometerHint(barometerSettingInHgCopilot);
    else if (lastBarometerSettingInHgCopilot != FLT_MAX && barometerSettingInHgCopilot != lastBarometerSettingInHgCopilot)
        DisplayBarometerHint(barometerSettingInHgCopilot);
    else if (lastAdf1CardHeadingDegMagPilot != FLT_MAX && adf1CardHeadingDegMagPilot != lastAdf1CardHeadingDegMagPilot)
        DisplayHeadingHint(adf1CardHeadingDegMagPilot);
    else if (lastAdf2CardHeadingDegMagPilot != FLT_MAX && adf2CardHeadingDegMagPilot != lastAdf2CardHeadingDegMagPilot)
        DisplayHeadingHint(adf2CardHeadingDegMagPilot);
    else if (lastAdf1CardHeadingDegMagCopilot != FLT_MAX && adf1CardHeadingDegMagCopilot != lastAdf1CardHeadingDegMagCopilot)
        DisplayHeadingHint(adf1CardHeadingDegMagCopilot);
    else if (lastAdf2CardHeadingDegMagCopilot != FLT_MAX && adf2CardHeadingDegMagCopilot != lastAdf2CardHeadingDegMagCopilot)
        DisplayHeadingHint(adf2CardHeadingDegMagCopilot);
    else if (lastHsiObsDegMagPilot != FLT_MAX && hsiObsDegMagPilot != lastHsiObsDegMagPilot)
        DisplayHeadingHint(hsiObsDegMagPilot);
    else if (lastHsiObsDegMagCopilot != FLT_MAX && hsiObsDegMagCopilot != lastHsiObsDegMagCopilot)
        DisplayHeadingHint(hsiObsDegMagCopilot);
    else if (lastNav1ObsDegMagPilot != FLT_MAX && nav1ObsDegMagPilot != lastNav1ObsDegMagPilot)
        DisplayHeadingHint(nav1ObsDegMagPilot);
    else if (lastNav2ObsDegMagPilot != FLT_MAX && nav2ObsDegMagPilot != lastNav2ObsDegMagPilot)
        DisplayHeadingHint(nav2ObsDegMagPilot);
    else if (lastNav1ObsDegMagCopilot != FLT_MAX && nav1ObsDegMagCopilot != lastNav1ObsDegMagCopilot)
        DisplayHeadingHint(nav1ObsDegMagCopilot);
    else if (lastNav2ObsDegMagCopilot != FLT_MAX && nav2ObsDegMagCopilot != lastNav2ObsDegMagCopilot)
        DisplayHeadingHint(nav2ObsDegMagCopilot);
    else
        changeDetected = 0;

    if (changeDetected != 0 && ((XPLMGetElapsedTime() - lastMouseUsageTime < 1.0f) || (lastChangeDetected != 0 && forceDisplay != 0)))
        forceDisplay = 1;
    else
        forceDisplay = 0;

    lastDgDriftVacDeg = dgDriftVacDeg;
    lastDgDriftEleDeg = dgDriftEleDeg;
    lastDgDriftVac2Deg = dgDriftVac2Deg;
    lastDgDriftEle2Deg = dgDriftEle2Deg;
    lastHeadingDialDegMagPilot = headingDialDegMagPilot;
    lastHeadingDialDegMagCopilot = headingDialDegMagCopilot;
    lastBarometerSettingInHgPilot = barometerSettingInHgPilot;
    lastBarometerSettingInHgCopilot = barometerSettingInHgCopilot;
    lastAdf1CardHeadingDegMagPilot = adf1CardHeadingDegMagPilot;
    lastAdf2CardHeadingDegMagPilot = adf2CardHeadingDegMagPilot;
    lastAdf1CardHeadingDegMagCopilot = adf1CardHeadingDegMagCopilot;
    lastAdf2CardHeadingDegMagCopilot = adf2CardHeadingDegMagCopilot;
    lastHsiObsDegMagPilot = hsiObsDegMagPilot;
    lastHsiObsDegMagCopilot = hsiObsDegMagCopilot;
    lastNav1ObsDegMagPilot = nav1ObsDegMagPilot;
    lastNav2ObsDegMagPilot = nav2ObsDegMagPilot;
    lastNav1ObsDegMagCopilot = nav1ObsDegMagCopilot;
    lastNav2ObsDegMagCopilot = nav2ObsDegMagCopilot;

    lastChangeDetected = changeDetected;

    return 0.1f;
}

// draw-callback that performs the actual drawing of the hint
static int DrawCallback(XPLMDrawingPhase inPhase, int inIsBefore, void *inRefcon)
{
    float currentTime = XPLMGetElapsedTime();

    if ((currentTime - lastMouseUsageTime <= HINT_DURATION || forceDisplay != 0) && currentTime - lastHintTime <= HINT_DURATION)
    {
        float color[] = {1.0f, 1.0f, 1.0f};
        int x = 0, y = 0;
        XPLMGetMouseLocation(&x, &y);
        XPLMDrawString(color, x + 40, y - 40, hintText, NULL, xplmFont_Basic);
    }

    return 1;
}

static void DrawWindow(XPLMWindowID inWindowID, void *inRefcon)
{
}

static void HandleKey(XPLMWindowID inWindowID, char inKey, XPLMKeyFlags inFlags, char inVirtualKey, void *inRefcon, int losingFocus)
{
}

static int HandleMouseClick(XPLMWindowID inWindowID, int x, int y, XPLMMouseStatus inMouse, void *inRefcon)
{
    lastMouseUsageTime = XPLMGetElapsedTime();

    return 0;
}

static XPLMCursorStatus HandleCursor(XPLMWindowID inWindowID, int x, int y, void *inRefcon)
{
    return xplm_CursorDefault;
}

static int HandleMouseWheel(XPLMWindowID inWindowID, int x, int y, int wheel, int clicks, void *inRefcon)
{
    lastMouseUsageTime = XPLMGetElapsedTime();

    return 0;
}

PLUGIN_API int XPluginStart(char *outName, char *outSig, char *outDesc)
{
    // set plugin info
    strcpy(outName, NAME);
    strcpy(outSig, "de.bwravencl." NAME_LOWERCASE);
    strcpy(outDesc, NAME " simpliefies handling X-Plane by adding tooltips!");

    // obtain datarefs
    dgDriftVacDegDataRef = XPLMFindDataRef("sim/cockpit/gyros/dg_drift_vac_deg");
    dgDriftEleDegDataRef = XPLMFindDataRef("sim/cockpit/gyros/dg_drift_ele_deg");
    dgDriftVac2DegDataRef = XPLMFindDataRef("sim/cockpit/gyros/dg_drift_vac2_deg");
    dgDriftEle2DegDataRef = XPLMFindDataRef("sim/cockpit/gyros/dg_drift_ele2_deg");
    headingDialDegMagPilotDataRef = XPLMFindDataRef("sim/cockpit2/autopilot/heading_dial_deg_mag_pilot");
    headingDialDegMagCopilotDataRef = XPLMFindDataRef("sim/cockpit2/autopilot/heading_dial_deg_mag_copilot");
    barometerSettingInHgPilotDataRef = XPLMFindDataRef("sim/cockpit2/gauges/actuators/barometer_setting_in_hg_pilot");
    barometerSettingInHgCopilotDataRef = XPLMFindDataRef("sim/cockpit2/gauges/actuators/barometer_setting_in_hg_copilot");
    adf1CardHeadingDegMagPilotDataRef = XPLMFindDataRef("sim/cockpit2/radios/actuators/adf1_card_heading_deg_mag_pilot");
    adf2CardHeadingDegMagPilotDataRef = XPLMFindDataRef("sim/cockpit2/radios/actuators/adf2_card_heading_deg_mag_pilot");
    adf1CardHeadingDegMagCopilotDataRef = XPLMFindDataRef("sim/cockpit2/radios/actuators/adf1_card_heading_deg_mag_copilot");
    adf2CardHeadingDegMagCopilotDataRef = XPLMFindDataRef("sim/cockpit2/radios/actuators/adf2_card_heading_deg_mag_copilot");
    hsiObsDegMagPilotDataRef = XPLMFindDataRef("sim/cockpit2/radios/actuators/hsi_obs_deg_mag_pilot");
    hsiObsDegMagCopilotDataRef = XPLMFindDataRef("sim/cockpit2/radios/actuators/hsi_obs_deg_mag_copilot");
    nav1ObsDegMagPilotDataRef = XPLMFindDataRef("sim/cockpit2/radios/actuators/nav1_obs_deg_mag_pilot");
    nav2ObsDegMagPilotDataRef = XPLMFindDataRef("sim/cockpit2/radios/actuators/nav2_obs_deg_mag_pilot");
    nav1ObsDegMagCopilotDataRef = XPLMFindDataRef("sim/cockpit2/radios/actuators/nav1_obs_deg_mag_copilot");
    nav2ObsDegMagCopilotDataRef = XPLMFindDataRef("sim/cockpit2/radios/actuators/nav2_obs_deg_mag_copilot");

    // create fake window
    XPLMCreateWindow_t fakeWindowParameters;
    memset(&fakeWindowParameters, 0, sizeof(fakeWindowParameters));
    fakeWindowParameters.structSize = sizeof(fakeWindowParameters);
    fakeWindowParameters.left = 0;
    int x = 0, y = 0;
    XPLMGetScreenSize(&x, &y);
    fakeWindowParameters.top = y;
    fakeWindowParameters.right = x;
    fakeWindowParameters.bottom = 0;
    fakeWindowParameters.visible = 1;
    fakeWindowParameters.drawWindowFunc = DrawWindow;
    fakeWindowParameters.handleKeyFunc = HandleKey;
    fakeWindowParameters.handleMouseClickFunc = HandleMouseClick;
    fakeWindowParameters.handleCursorFunc = HandleCursor;
    fakeWindowParameters.handleMouseWheelFunc = HandleMouseWheel;
    fakeWindow = XPLMCreateWindowEx(&fakeWindowParameters);

    // register flight loop callbacks
    XPLMRegisterFlightLoopCallback(UpdateFakeWindowCallback, -1, NULL);
    XPLMRegisterFlightLoopCallback(FlightLoopCallback, -1, NULL);

    // register draw callback
    XPLMRegisterDrawCallback(DrawCallback, xplm_Phase_LastCockpit, 0, NULL);

    return 1;
}

PLUGIN_API void	XPluginStop(void)
{
    // unregister flight loop callbacks
    XPLMUnregisterFlightLoopCallback(UpdateFakeWindowCallback, NULL);
    XPLMUnregisterFlightLoopCallback(FlightLoopCallback, NULL);

    // unregister draw callback
    XPLMUnregisterDrawCallback(DrawCallback, xplm_Phase_LastCockpit, 0, NULL);
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
    if (inMessage == XPLM_MSG_PLANE_LOADED)
        bringFakeWindowToFront = 0;
}
