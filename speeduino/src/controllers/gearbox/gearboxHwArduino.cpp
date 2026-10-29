/**
 * @file gearboxHwArduino.cpp
 * @brief Arduino/Speeduino side of the paddle shift controller: real pins, real clock, Speeduino's
 *        live values, and the call from the main loop.
 *
 * All decisions are made in gearboxController.cpp / gearboxStateMachine.cpp (tested on a PC).
 * This file only supplies the real hardware functions and copies Speeduino's values.
 *
 * HOW TO SWITCH THE GEARBOX ON (there is no TunerStudio page yet): edit gearboxBoardSetup() below:
 * give it the paddle pins, set config.enabled = true, and optionally register an actuator driver.
 * Until then the module does nothing at all.
 *
 * NOT YET TESTED ON REAL HARDWARE.
 */
#include "../../../globals.h"
#include "gearboxController.h"
#include "gearboxHwArduino.h"

// ------------------------------------------------------------------------------------------------
//  Real hardware functions
// ------------------------------------------------------------------------------------------------
static void hwSetupInput(uint8_t pin, bool pullup) { pinMode(pin, pullup ? INPUT_PULLUP : INPUT); }
static bool hwReadPin(uint8_t pin)                 { return digitalRead(pin) != LOW; }
static uint32_t hwNowMs(void)                      { return millis(); }

static const gearboxHal_t arduinoHal = { hwSetupInput, hwReadPin, hwNowMs };

static gearboxController_t controller;
static bool controllerReady = false;

// ------------------------------------------------------------------------------------------------
//  >>> THE PLACE TO CONFIGURE THE GEARBOX <<<
// ------------------------------------------------------------------------------------------------
/**
 * Called once, before the pins are set up. Default: everything off.
 *
 * Example (pins are only an example, pick free ones for your board!):
 *
 *     controller.pinPaddleUp   = 30;
 *     controller.pinPaddleDown = 31;
 *     controller.pinNeutral    = 32;                 // optional, GEARBOX_NO_PIN if not fitted
 *     controller.config.enabled = true;
 *     controller.actuator = myServoDriver;           // void myServoDriver(bool enable, int16_t permille)
 *
 * The timing values (cut time, hold time, debounce, ...) are in gearbox_config_t, see
 * gearboxStateMachine.h. Make sure the chosen pins are not used by anything else in Speeduino.
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
    gearboxInit(controller, arduinoHal);
    controllerReady = true;
  }
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
