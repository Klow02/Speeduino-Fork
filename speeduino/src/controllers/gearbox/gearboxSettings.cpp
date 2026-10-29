/**
 * @file gearboxSettings.cpp
 * @brief Implementation of gearboxSettings.h. No hardware access.
 */
#include "gearboxSettings.h"
#include <string.h>

/** Limits a value to 0 ... maxValue. */
static uint16_t clampUp(uint16_t value, uint16_t maxValue)
{
  return (value > maxValue) ? maxValue : value;
}

void gearboxApplySettings(const gearboxSettingsPage_t& page, gearboxController_t& ctrl)
{
  gearbox_config_t& c = ctrl.config;
  c.enabled            = (page.enabled != 0U);
  c.allowDownToNeutral = (page.allowDownToNeutral != 0U);
  c.cutOnUpshift       = (page.cutOnUpshift != 0U);
  c.cutOnDownshift     = (page.cutOnDownshift != 0U);
  c.cutIgnition        = (page.cutIgnition != 0U);
  c.cutFuel            = (page.cutFuel != 0U);
  c.useFeedback        = (page.useFeedback != 0U);

  // Highest gear: at least 1, at most what the sequencer can handle.
  uint8_t maxGear = page.maxGear;
  if (maxGear < 1U) { maxGear = 1U; }
  if (maxGear > GEARBOX_MAX_GEARS) { maxGear = GEARBOX_MAX_GEARS; }
  c.maxGear = maxGear;

  c.cutMinRpm         = page.cutMinRpm;
  c.maxCutMs          = page.maxCutMs;
  c.downshiftMaxRpm   = page.downshiftMaxRpm;
  c.upTravel          = clampUp(page.upTravel, 1000U);
  c.downTravel        = clampUp(page.downTravel, 1000U);
  c.positionTolerance = clampUp(page.positionTolerance, 1000U);
  c.debounceMs        = page.debounceMs;
  c.cutLeadMs         = page.cutLeadMs;
  c.moveMs            = page.moveMs;
  c.holdMs            = page.holdMs;
  c.returnMs          = page.returnMs;
  c.verifyMs          = page.verifyMs;
  c.minIntervalMs     = page.minIntervalMs;
  c.maxShiftMs        = page.maxShiftMs;
  c.faultLockoutMs    = page.faultLockoutMs;

  ctrl.io.paddlesActiveLow = (page.paddlesActiveLow != 0U);
  ctrl.io.neutralActiveLow = (page.neutralActiveLow != 0U);
  ctrl.pinPaddleUp   = page.pinPaddleUp;
  ctrl.pinPaddleDown = page.pinPaddleDown;
  ctrl.pinNeutral    = page.pinNeutral;
}

bool gearboxRefreshSettings(const gearboxSettingsPage_t& page, gearboxSettingsPage_t& cached, bool& cachedValid,
                            gearboxController_t& ctrl, const gearboxHal_t& hal)
{
  if (cachedValid && memcmp(&page, &cached, sizeof(page)) == 0)
  {
    return false; // nothing changed
  }
  // Never change the settings in the middle of a shift: wait until the sequencer is idle again.
  if (ctrl.state.phase != ShiftPhase::Idle)
  {
    return false;
  }

  gearboxApplySettings(page, ctrl);
  // New pins / polarity / enable state: set the pins up again and restart the sequencer.
  // (gearboxInit switches the module off if a paddle pin is missing.)
  gearboxInit(ctrl, hal);

  memcpy(&cached, &page, sizeof(page));
  cachedValid = true;
  return true;
}
