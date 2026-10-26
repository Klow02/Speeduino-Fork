#include "gearboxController.h"
#include "src/pins/inputPin.h"
#include "unit_testing.h"
#include "globals.h"

static gearbox_config_t _config;
static gearbox_io_config_t _ioConfig;
static gearbox_state_t _state;
static gearboxActuator_t _actuator = nullptr;

TESTABLE_STATIC inputPin_t paddleUpPin;
TESTABLE_STATIC inputPin_t paddleDownPin;
TESTABLE_STATIC inputPin_t neutralPin;
static debounce_t _debounceUp;
static debounce_t _debounceDown;
static bool _primed = false;

gearbox_config_t& gearboxConfig(void) { return _config; }
gearbox_io_config_t& gearboxIoConfig(void) { return _ioConfig; }
void gearboxSetActuator(gearboxActuator_t actuator) { _actuator = actuator; }

void initialiseGearbox(statuses &current, const pinNumbers_t &pins)
{
  paddleUpPin.setPin(pins.pinPaddleUp, _ioConfig.paddlesActiveLow ? INPUT_PULLUP : INPUT);
  paddleDownPin.setPin(pins.pinPaddleDown, _ioConfig.paddlesActiveLow ? INPUT_PULLUP : INPUT);
  neutralPin.setPin(pins.pinNeutral, _ioConfig.neutralActiveLow ? INPUT_PULLUP : INPUT);

  // Both paddles are needed. Without them the module stays off.
  _config.enabled = _config.enabled && paddleUpPin.isValid() && paddleDownPin.isValid();

  _state = gearbox_state_t();
  _primed = false;
  current.gearbox = gearboxStatus_t();
}

static bool isPressed(const inputPin_t &pin, bool activeLow)
{
  return pin.isValid() && (pin.isPinHigh() != activeLow);
}

TESTABLE_STATIC gearbox_inputs_t buildInputs(const statuses &current, const config2 &page2, uint32_t nowMs, bool upRequest, bool downRequest)
{
  gearbox_inputs_t in;
  in.nowMs = nowMs;
  in.upRequest = upRequest;
  in.downRequest = downRequest;
  in.rpm = current.RPM;

  // The neutral switch, when fitted, is more trustworthy than the VSS/RPM ratio
  if (isPressed(neutralPin, _ioConfig.neutralActiveLow))
  {
    in.gear = 0U;
    in.gearValid = true;
  }
  else
  {
    in.gear = current.gear;
    // currentStatus.gear is 0 when it could not be worked out. That is "unknown", not neutral.
    in.gearValid = (page2.vssMode != VSS_MODE_OFF) && (current.gear > 0U);
  }

  for (uint8_t i = 0U; i < GEARBOX_MAX_GEARS; ++i)
  {
    in.ratio[i] = page2.vssRatios[i];
  }

  // No position feedback is wired up yet
  in.positionValid = false;
  return in;
}

static void publish(statuses &current, const gearbox_outputs_t &out)
{
  gearboxStatus_t &s = current.gearbox;
  s.phase = _state.phase;
  s.direction = _state.direction;
  s.lastFault = _state.lastFault;
  s.lastReject = _state.lastReject;
  s.cutIgnition = out.cutIgnition;
  s.cutFuel = out.cutFuel;
  s.actuatorEnable = out.actuatorEnable;
  s.actuatorPermille = out.actuatorPermille;
  s.shiftCount = _state.shiftCount;
}

void gearboxControl(statuses &current, const config2 &page2)
{
  const uint32_t nowMs = millis();

  const bool upPressed = isPressed(paddleUpPin, _ioConfig.paddlesActiveLow);
  const bool downPressed = isPressed(paddleDownPin, _ioConfig.paddlesActiveLow);

  // A paddle that is already held when the module starts must not count as a press
  if (!_primed)
  {
    debouncePrime(_debounceUp, upPressed, nowMs);
    debouncePrime(_debounceDown, downPressed, nowMs);
    _primed = true;
  }

  const bool upRequest = debounceRisingEdge(_debounceUp, upPressed, nowMs, _config.debounceMs);
  const bool downRequest = debounceRisingEdge(_debounceDown, downPressed, nowMs, _config.debounceMs);

  const gearbox_inputs_t in = buildInputs(current, page2, nowMs, upRequest, downRequest);
  const gearbox_outputs_t out = gearboxStep(_state, _config, in);

  publish(current, out);

  if (_actuator != nullptr)
  {
    _actuator(out.actuatorEnable, out.actuatorPermille);
  }
}
