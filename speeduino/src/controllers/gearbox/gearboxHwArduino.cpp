/**
 * @file gearboxHwArduino.cpp
 * @brief Arduino/Speeduino side of the paddle shift controller: real pins, real clock, Speeduino's
 *        live values, and the call from the main loop.
 *
 * All decisions are made in gearboxController.cpp / gearboxStateMachine.cpp (tested on a PC).
 * This file only supplies the real hardware functions and copies Speeduino's values.
 *
 * HOW TO SWITCH THE GEARBOX ON: open TunerStudio, menu Accessories -> "Gearbox controller" (page 17),
 * tick "Enable", enter the paddle pins and burn. Until then the module does nothing at all.
 * The settings are re-read every loop (but never in the middle of a shift).
 * The servo/actuator driver is NOT written yet: register one in gearboxBoardSetup() below.
 *
 * NOT YET TESTED ON REAL HARDWARE.
 */
#include "../../../globals.h"
#include "gearboxController.h"
#include "gearboxSettings.h"
#include "gearboxHwArduino.h"

// ------------------------------------------------------------------------------------------------
//  Real hardware functions
// ------------------------------------------------------------------------------------------------
static void hwSetupInput(uint8_t pin, bool pullup) { pinMode(pin, pullup ? INPUT_PULLUP : INPUT); }
static bool hwReadPin(uint8_t pin)                 { return digitalRead(pin) != LOW; }
static uint32_t hwNowMs(void)                      { return millis(); }

/** A pin may be used only if Speeduino does not already use it (sensors, outputs, reserved pins). */
static bool hwPinAvailable(uint8_t pin) { return !pinIsUsed(pin); }

static const gearboxHal_t arduinoHal = { hwSetupInput, hwReadPin, hwNowMs, hwPinAvailable };

static gearboxController_t controller;
static bool controllerReady = false;
static gearboxSettingsPage_t cachedSettings;   ///< settings page as last applied (to detect changes)
static bool cachedSettingsValid = false;

// ------------------------------------------------------------------------------------------------
//  >>> THE PLACE TO CONFIGURE THE GEARBOX <<<
// ------------------------------------------------------------------------------------------------
/**
 * Called once at start. The settings (enable, pins, timings) now come from TunerStudio page 17, so
 * the only thing left to do here is registering an actuator driver (servo), e.g.:
 *
 *     ctrl.actuator = myServoDriver;   // void myServoDriver(bool enable, int16_t permille)
 *
 * No driver exists yet, so the shifts run without moving anything (cut only).
 */
static void gearboxBoardSetup(gearboxController_t& ctrl)
{
  (void)ctrl; // nothing configured: the gearbox stays disabled
}

// ------------------------------------------------------------------------------------------------
//  Main loop entry points
// ------------------------------------------------------------------------------------------------

/** Copies the values the gearbox logic needs out of Speeduino's global status and settings. */
static gearbox_live_t readLiveValues(void)
{
  gearbox_live_t l;
  l.rpm = currentStatus.RPM;
  l.gear = currentStatus.gear;
  l.vssEnabled = (configPage2.vssMode != 0U);
  l.ratio[0] = configPage2.vssRatio1;
  l.ratio[1] = configPage2.vssRatio2;
  l.ratio[2] = configPage2.vssRatio3;
  l.ratio[3] = configPage2.vssRatio4;
  l.ratio[4] = configPage2.vssRatio5;
  l.ratio[5] = configPage2.vssRatio6;
  return l;
}

void gearboxTask(void)
{
  if (!controllerReady)
  {
    gearboxBoardSetup(controller);
    controllerReady = true;
  }
  // Pick up changes made in TunerStudio (does nothing if unchanged or while a shift runs).
  gearboxRefreshSettings(configPage17, cachedSettings, cachedSettingsValid, controller, arduinoHal);
  if (!controller.config.enabled)
  {
    return; // disabled: no pin reads, no cut, nothing
  }
  gearboxUpdate(controller, readLiveValues(), arduinoHal);
}

void gearboxApplyShiftCut(uint8_t& ignitionChannelsOn, uint8_t& fuelChannelsOn, uint8_t& ignitionChannelsPending)
{
  if (controllerReady && controller.config.enabled)
  {
    gearboxApplyCut(controller.status, ignitionChannelsOn, fuelChannelsOn, ignitionChannelsPending);
  }
}
