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

#if APL
#include "ApplicationServices/ApplicationServices.h"
#else
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#endif
#if IBM
#include <windows.h>
#elif LIN
#include <X11/Xlib.h>
#endif

// define name
#define NAME "X-hint"
#define NAME_LOWERCASE "x_hint"

// define version
#define VERSION "0.1"

// define hint duration
#define HINT_DURATION 1.0f

// define XPScrollWheel plugin signature
#define XP_SCROLL_WHEEL_PLUGIN_SIGNATURE "thranda.window.scrollwheel"

// global dataref variables
static XPLMDataRef dgDriftVacDegDataRef = NULL, dgDriftEleDegDataRef = NULL, dgDriftVac2DegDataRef = NULL, dgDriftEle2DegDataRef = NULL, headingDialDegMagPilotDataRef = NULL, headingDialDegMagCopilotDataRef = NULL, barometerSettingInHgPilotDataRef = NULL, barometerSettingInHgCopilotDataRef = NULL, adf1CardHeadingDegMagPilotDataRef = NULL, adf2CardHeadingDegMagPilotDataRef = NULL, adf1CardHeadingDegMagCopilotDataRef = NULL, adf2CardHeadingDegMagCopilotDataRef = NULL, hsiObsDegMagPilotDataRef = NULL, hsiObsDegMagCopilotDataRef = NULL, nav1ObsDegMagPilotDataRef = NULL, nav2ObsDegMagPilotDataRef = NULL, nav1ObsDegMagCopilotDataRef = NULL, nav2ObsDegMagCopilotDataRef = NULL;

// global internal variables
static float lastMouseClickTime = 0.0f, lastHintTime = 0.0f, lastDgDriftVacDeg = INT_MAX, lastDgDriftEleDeg = INT_MAX, lastDgDriftVac2Deg = INT_MAX, lastDgDriftEle2Deg = INT_MAX, lastHeadingDialDegMagPilot = INT_MAX, lastHeadingDialDegMagCopilot = INT_MAX, lastBarometerSettingInHgPilot = INT_MAX, lastBarometerSettingInHgCopilot = INT_MAX, lastAdf1CardHeadingDegMagPilot = INT_MAX, lastAdf2CardHeadingDegMagPilot = INT_MAX, lastAdf1CardHeadingDegMagCopilot = INT_MAX, lastAdf2CardHeadingDegMagCopilot = INT_MAX, lastHsiObsDegMagPilot = INT_MAX, lastHsiObsDegMagCopilot = INT_MAX, lastNav1ObsDegMagPilot = INT_MAX, lastNav2ObsDegMagPilot = INT_MAX, lastNav1ObsDegMagCopilot = INT_MAX, lastNav2ObsDegMagCopilot = INT_MAX;
static char hintText[32];
#if LIN
static Display *display = NULL;
#endif

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
static void DisplayDrifHint(float degrees)
{
    sprintf(hintText, "%.1f deg", HandleOverflow(degrees, -180.0f, 180.0f));
    lastHintTime = XPLMGetElapsedTime();
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
    sprintf(hintText, "%.0f deg", HandleOverflow(degrees, 0.0f, 360.0f));
    lastHintTime = XPLMGetElapsedTime();
}

// flightloop-callback that handles which hint is when displayed
static float FlightLoopCallback(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, int inCounter, void *inRefcon)
{
#if APL
    int mouseButtonDown = CGEventSourceButtonState(kCGEventSourceStateCombinedSessionState, kCGMouseButtonLeft);
#elif IBM
    // if the most significant bit is set, the key is down
    SHORT state = GetAsyncKeyState(VK_LBUTTON);
    SHORT msb = state >> 15;
    int mouseButtonDown = (int) msb;
#elif LIN
    if (display == NULL)
        display = XOpenDisplay(NULL);
    Window root, child;
    int rootX, rootY, winX, winY;
    unsigned int mask;
    XQueryPointer(display, DefaultRootWindow(display), &root, &child, &rootX, &rootY, &winX, &winY, &mask);
    int mouseButtonDown = (mask & Button1Mask) >> 8;
#endif

    if (mouseButtonDown != 0)
        lastMouseClickTime = XPLMGetElapsedTime();

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

    if (lastDgDriftVacDeg != INT_MAX && fabs(dgDriftVacDeg - lastDgDriftVacDeg) > 0.01f)
        DisplayDrifHint(dgDriftVacDeg);
    else if (lastDgDriftEleDeg != INT_MAX && fabs(dgDriftEleDeg - lastDgDriftEleDeg) > 0.01f)
        DisplayDrifHint(dgDriftEleDeg);
    else if (lastDgDriftVac2Deg != INT_MAX && fabs(dgDriftVac2Deg - lastDgDriftVac2Deg) > 0.01f)
        DisplayDrifHint(dgDriftVac2Deg);
    else if (lastDgDriftEle2Deg != INT_MAX && fabs(dgDriftEle2Deg - lastDgDriftEle2Deg) > 0.01f)
        DisplayDrifHint(dgDriftEle2Deg);
    else if (lastHeadingDialDegMagPilot != INT_MAX && headingDialDegMagPilot != lastHeadingDialDegMagPilot)
        DisplayHeadingHint(headingDialDegMagPilot);
    else if (lastHeadingDialDegMagCopilot != INT_MAX && headingDialDegMagCopilot != lastHeadingDialDegMagCopilot)
        DisplayHeadingHint(headingDialDegMagCopilot);
    else if (lastBarometerSettingInHgPilot != INT_MAX && barometerSettingInHgPilot != lastBarometerSettingInHgPilot)
        DisplayBarometerHint(barometerSettingInHgPilot);
    else if (lastBarometerSettingInHgCopilot != INT_MAX && barometerSettingInHgCopilot != lastBarometerSettingInHgCopilot)
        DisplayBarometerHint(barometerSettingInHgCopilot);
    else if (lastBarometerSettingInHgCopilot != INT_MAX && barometerSettingInHgCopilot != lastBarometerSettingInHgCopilot)
        DisplayBarometerHint(barometerSettingInHgCopilot);
    else if (lastAdf1CardHeadingDegMagPilot != INT_MAX && adf1CardHeadingDegMagPilot != lastAdf1CardHeadingDegMagPilot)
        DisplayHeadingHint(adf1CardHeadingDegMagPilot);
    else if (lastAdf2CardHeadingDegMagPilot != INT_MAX && adf2CardHeadingDegMagPilot != lastAdf2CardHeadingDegMagPilot)
        DisplayHeadingHint(adf2CardHeadingDegMagPilot);
    else if (lastAdf1CardHeadingDegMagCopilot != INT_MAX && adf1CardHeadingDegMagCopilot != lastAdf1CardHeadingDegMagCopilot)
        DisplayHeadingHint(adf1CardHeadingDegMagCopilot);
    else if (lastAdf2CardHeadingDegMagCopilot != INT_MAX && adf2CardHeadingDegMagCopilot != lastAdf2CardHeadingDegMagCopilot)
        DisplayHeadingHint(adf2CardHeadingDegMagCopilot);
    else if (lastHsiObsDegMagPilot != INT_MAX && hsiObsDegMagPilot != lastHsiObsDegMagPilot)
        DisplayHeadingHint(hsiObsDegMagPilot);
    else if (lastHsiObsDegMagCopilot != INT_MAX && hsiObsDegMagCopilot != lastHsiObsDegMagCopilot)
        DisplayHeadingHint(hsiObsDegMagCopilot);
    else if (lastNav1ObsDegMagPilot != INT_MAX && nav1ObsDegMagPilot != lastNav1ObsDegMagPilot)
        DisplayHeadingHint(nav1ObsDegMagPilot);
    else if (lastNav2ObsDegMagPilot != INT_MAX && nav2ObsDegMagPilot != lastNav2ObsDegMagPilot)
        DisplayHeadingHint(nav2ObsDegMagPilot);
    else if (lastNav1ObsDegMagCopilot != INT_MAX && nav1ObsDegMagCopilot != lastNav1ObsDegMagCopilot)
        DisplayHeadingHint(nav1ObsDegMagCopilot);
    else if (lastNav2ObsDegMagCopilot != INT_MAX && nav2ObsDegMagCopilot != lastNav2ObsDegMagCopilot)
        DisplayHeadingHint(nav2ObsDegMagCopilot);

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

    return -1.0f;
}

// check if the XPScrollWheel plugin is active
static int isXpScrollWheelPluginActive()
{
    XPLMPluginID pluginId = XPLMFindPluginBySignature(XP_SCROLL_WHEEL_PLUGIN_SIGNATURE);

    if (XPLMIsPluginEnabled(pluginId) != 0)
    {
        XPLMDataRef scrollWhDataRef = XPLMFindDataRef("thranda/panels/scrollWh");
        return XPLMGetDatai(scrollWhDataRef);
    }
    else
        return 0;
}

// draw-callback that performs the actual drawing of the hint
static int DrawCallback(XPLMDrawingPhase inPhase, int inIsBefore, void *inRefcon)
{
    float currentTime = XPLMGetElapsedTime();

    if ((currentTime - lastMouseClickTime <= HINT_DURATION || isXpScrollWheelPluginActive() != 0) && currentTime - lastHintTime <= HINT_DURATION)
    {
        float color[] = {1.0f, 1.0f, 1.0f};
        int x = 0, y = 0;
        XPLMGetMouseLocation(&x, &y);
        XPLMDrawString(color, x + 40, y - 40, hintText, NULL, xplmFont_Basic);
    }

    return 1;
}

PLUGIN_API int XPluginStart(char *outName, char *outSig, char *outDesc)
{
    // set plugin info
    strcpy(outName, NAME);
    strcpy(outSig, "de.bwravencl."NAME_LOWERCASE);
    strcpy(outDesc, NAME" simpliefies handling X-Plane by adding tooltips!");

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

    // register flight loop callback
    XPLMRegisterFlightLoopCallback(FlightLoopCallback, -1, NULL);

    // register draw callback
    XPLMRegisterDrawCallback(DrawCallback, xplm_Phase_LastCockpit, 0, NULL);

    return 1;
}

PLUGIN_API void	XPluginStop(void)
{
#if LIN
    if(display != NULL)
        XCloseDisplay(display);
#endif
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
