/**
 * @file gearboxController.cpp
 * @brief Implementation of gearboxController.h. No direct hardware access.
 */
#include "gearboxController.h"

void gearboxInit(gearboxController_t& ctrl, const gearboxHal_t& hal)
{
  ctrl.state = gearbox_state_t();
  ctrl.status = gearboxStatus_t();
  ctrl.primed = false;

  if (!ctrl.config.enabled)
  {
    return; // switched off: do not touch any pin
  }

  // A pin counts only if one is given AND Speeduino is not using it for something else.
  const bool upOk   = (ctrl.pinPaddleUp   != GEARBOX_NO_PIN) && hal.pinAvailable(ctrl.pinPaddleUp);
  const bool downOk = (ctrl.pinPaddleDown != GEARBOX_NO_PIN) && hal.pinAvailable(ctrl.pinPaddleDown);

  // Both paddles are needed. Without them the module switches itself off and touches nothing.
  if (!(upOk && downOk))
  {
    ctrl.config.enabled = false;
    return;
  }

  // The neutral switch is optional. An unusable pin is forgotten, so it is never read.
  if ((ctrl.pinNeutral != GEARBOX_NO_PIN) && !hal.pinAvailable(ctrl.pinNeutral))
  {
    ctrl.pinNeutral = GEARBOX_NO_PIN;
  }

  const bool pullup = ctrl.io.paddlesActiveLow; // active-low switches need the internal pull-up
  hal.setupInput(ctrl.pinPaddleUp, pullup);
  hal.setupInput(ctrl.pinPaddleDown, pullup);
  if (ctrl.pinNeutral != GEARBOX_NO_PIN)
  {
    hal.setupInput(ctrl.pinNeutral, ctrl.io.neutralActiveLow);
  }
}

/** Is this switch pressed? A pin that is not connected is never pressed. */
static bool isPressed(const gearboxHal_t& hal, uint8_t pin, bool activeLow)
{
  return (pin != GEARBOX_NO_PIN) && (hal.readPin(pin) != activeLow);
}

gearbox_inputs_t gearboxBuildInputs(const gearboxController_t& ctrl, const gearboxHal_t& hal, const gearbox_live_t& live,
                                    uint32_t nowMs, bool upRequest, bool downRequest)
{
  gearbox_inputs_t in;
  in.nowMs = nowMs;
  in.upRequest = upRequest;
  in.downRequest = downRequest;
  in.rpm = live.rpm;

  if (isPressed(hal, ctrl.pinNeutral, ctrl.io.neutralActiveLow))
  {
    in.gear = 0U;
    in.gearValid = true;
  }
  else
  {
    in.gear = live.gear;
    // Speeduino reports gear 0 when it could not work the gear out. That is "unknown", not neutral.
    in.gearValid = live.vssEnabled && (live.gear > 0U);
  }

  for (uint8_t i = 0U; i < GEARBOX_MAX_GEARS; ++i)
  {
    in.ratio[i] = live.ratio[i];
  }

  // No position feedback is wired up yet.
  in.positionValid = false;
  return in;
}

void gearboxUpdate(gearboxController_t& ctrl, const gearbox_live_t& live, const gearboxHal_t& hal)
{
  const uint32_t nowMs = hal.nowMs();

  const bool upPressed   = isPressed(hal, ctrl.pinPaddleUp,   ctrl.io.paddlesActiveLow);
  const bool downPressed = isPressed(hal, ctrl.pinPaddleDown, ctrl.io.paddlesActiveLow);

  // A paddle that is already held when the module starts must not count as a press.
  if (!ctrl.primed)
  {
    debouncePrime(ctrl.debounceUp, upPressed, nowMs);
    debouncePrime(ctrl.debounceDown, downPressed, nowMs);
    ctrl.primed = true;
  }

  const bool upRequest   = debounceRisingEdge(ctrl.debounceUp,   upPressed,   nowMs, ctrl.config.debounceMs);
  const bool downRequest = debounceRisingEdge(ctrl.debounceDown, downPressed, nowMs, ctrl.config.debounceMs);

  const gearbox_inputs_t in = gearboxBuildInputs(ctrl, hal, live, nowMs, upRequest, downRequest);
  const gearbox_outputs_t out = gearboxStep(ctrl.state, ctrl.config, in);

  // Publish the result.
  gearboxStatus_t& s = ctrl.status;
  s.phase = ctrl.state.phase;
  s.direction = ctrl.state.direction;
  s.lastFault = ctrl.state.lastFault;
  s.lastReject = ctrl.state.lastReject;
  s.cutIgnition = out.cutIgnition;
  s.cutFuel = out.cutFuel;
  s.actuatorEnable = out.actuatorEnable;
  s.actuatorPermille = out.actuatorPermille;
  s.shiftCount = ctrl.state.shiftCount;

  if (ctrl.actuator != nullptr)
  {
    ctrl.actuator(out.actuatorEnable, out.actuatorPermille);
  }
}

void gearboxApplyCut(const gearboxStatus_t& status, uint8_t& ignitionOn, uint8_t& fuelOn, uint8_t& ignitionPending)
{
  if (status.cutIgnition)
  {
    ignitionOn = 0x00U;
    ignitionPending = 0x00U;
  }
  if (status.cutFuel)
  {
    fuelOn = 0x00U;
  }
}
