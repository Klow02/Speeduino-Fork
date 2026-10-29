#include "gearboxStateMachine.h"

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Elapsed check that survives the 32-bit millisecond counter wrapping
static inline bool hasElapsed(uint32_t nowMs, uint32_t sinceMs, uint16_t durationMs)
{
  return static_cast<uint32_t>(nowMs - sinceMs) >= static_cast<uint32_t>(durationMs);
}

static inline int16_t clampTravel(uint16_t travel)
{
  return (travel > static_cast<uint16_t>(GEARBOX_PERMILLE_FULL)) ? GEARBOX_PERMILLE_FULL : static_cast<int16_t>(travel);
}

static int16_t targetPermille(const gearbox_config_t &cfg, ShiftDirection direction)
{
  if (direction == ShiftDirection::Up)
  {
    return clampTravel(cfg.upTravel);
  }
  if (direction == ShiftDirection::Down)
  {
    return static_cast<int16_t>(-clampTravel(cfg.downTravel));
  }
  return 0;
}

static bool isNear(int16_t position, int16_t target, uint16_t tolerance)
{
  int32_t diff = static_cast<int32_t>(position) - static_cast<int32_t>(target);
  if (diff < 0)
  {
    diff = -diff;
  }
  return diff <= static_cast<int32_t>(tolerance);
}

static void setPhase(gearbox_state_t &state, ShiftPhase phase, uint32_t nowMs)
{
  state.phase = phase;
  state.phaseStartMs = nowMs;
}

static void enterFault(gearbox_state_t &state, ShiftFault fault, uint32_t nowMs)
{
  state.lastFault = fault;
  state.cutActive = false;
  state.direction = ShiftDirection::None;
  setPhase(state, ShiftPhase::Fault, nowMs);
}

static bool isSequenceRunning(ShiftPhase phase)
{
  return (phase == ShiftPhase::PreCut)
      || (phase == ShiftPhase::Move)
      || (phase == ShiftPhase::Hold)
      || (phase == ShiftPhase::Return)
      || (phase == ShiftPhase::Verify);
}

/**
 * Predict the engine RPM after a downshift from the current gear.
 *
 * The ratio table holds speed per rpm for each gear, so a taller gear has a larger value:
 * rpm_after = rpm * ratio[current] / ratio[current - 1]
 *
 * If the gear or ratios are unknown, the current RPM is used. That is optimistic about the
 * ratio but still stops a downshift while the engine is already over the limit.
 */
static bool wouldOverRev(const gearbox_config_t &cfg, const gearbox_inputs_t &in)
{
  if (cfg.downshiftMaxRpm == 0U)
  {
    return false;
  }

  uint32_t predicted = in.rpm;
  if (in.gearValid && (in.gear >= 2U) && (in.gear <= GEARBOX_MAX_GEARS))
  {
    const uint16_t from = in.ratio[in.gear - 1U];
    const uint16_t to = in.ratio[in.gear - 2U];
    if ((from != 0U) && (to != 0U))
    {
      predicted = (static_cast<uint32_t>(in.rpm) * from) / to;
    }
  }
  return predicted > cfg.downshiftMaxRpm;
}

static ShiftReject checkRequest(const gearbox_config_t &cfg, const gearbox_inputs_t &in, ShiftDirection direction)
{
  if (direction == ShiftDirection::Up)
  {
    if (in.gearValid && (in.gear >= cfg.maxGear))
    {
      return ShiftReject::TopGear;
    }
  }
  else
  {
    if (in.gearValid)
    {
      // Neutral, or 1st gear when moving to neutral is not wanted
      if ((in.gear == 0U) || ((in.gear == 1U) && !cfg.allowDownToNeutral))
      {
        return ShiftReject::BottomGear;
      }
    }
    if (wouldOverRev(cfg, in))
    {
      return ShiftReject::OverRev;
    }
  }
  return ShiftReject::None;
}

static void startShift(gearbox_state_t &state, const gearbox_config_t &cfg, const gearbox_inputs_t &in, ShiftDirection direction)
{
  state.direction = direction;
  state.shiftStartMs = in.nowMs;
  state.lastReject = ShiftReject::None;
  state.lastFault = ShiftFault::None;

  // checkRequest() has already refused a downshift from neutral, so gear-1 can't underflow
  state.expectGearKnown = in.gearValid;
  if (in.gearValid)
  {
    state.expectGear = (direction == ShiftDirection::Up) ? static_cast<uint8_t>(in.gear + 1U) : static_cast<uint8_t>(in.gear - 1U);
  }

  const bool cutForDirection = (direction == ShiftDirection::Up) ? cfg.cutOnUpshift : cfg.cutOnDownshift;
  const bool cutWanted = cutForDirection
                      && (cfg.cutIgnition || cfg.cutFuel)
                      && (cfg.maxCutMs > 0U)
                      && (in.rpm >= cfg.cutMinRpm);
  if (cutWanted)
  {
    state.cutActive = true;
    state.cutStartMs = in.nowMs;
  }

  setPhase(state, (cutWanted && (cfg.cutLeadMs > 0U)) ? ShiftPhase::PreCut : ShiftPhase::Move, in.nowMs);
}

static void finishShift(gearbox_state_t &state, uint32_t nowMs)
{
  state.shiftCount++;
  state.direction = ShiftDirection::None;
  setPhase(state, ShiftPhase::Lockout, nowMs);
}

// ---------------------------------------------------------------------------
// State machine
// ---------------------------------------------------------------------------

gearbox_outputs_t gearboxStep(gearbox_state_t &state, const gearbox_config_t &cfg, const gearbox_inputs_t &in)
{
  gearbox_outputs_t out;
  const uint32_t now = in.nowMs;
  const bool anyRequest = in.upRequest || in.downRequest;

  if (!cfg.enabled)
  {
    // Drop everything. Outputs stay at their all-off defaults.
    const uint16_t count = state.shiftCount;
    state = gearbox_state_t();
    state.shiftCount = count;
    if (anyRequest)
    {
      state.lastReject = ShiftReject::Disabled;
    }
    return out;
  }

  // Limits that apply whatever the sequence is doing -------------------------
  if (state.cutActive && hasElapsed(now, state.cutStartMs, cfg.maxCutMs))
  {
    state.cutActive = false;
  }
  if (isSequenceRunning(state.phase) && hasElapsed(now, state.shiftStartMs, cfg.maxShiftMs))
  {
    enterFault(state, ShiftFault::SequenceTimeout, now);
  }

  // Requests while not idle are refused, not queued --------------------------
  if (anyRequest && (state.phase != ShiftPhase::Idle))
  {
    if (state.phase == ShiftPhase::Lockout)
    {
      state.lastReject = ShiftReject::Lockout;
    }
    else if (state.phase == ShiftPhase::Fault)
    {
      state.lastReject = ShiftReject::Fault;
    }
    else
    {
      state.lastReject = ShiftReject::Busy;
    }
  }

  switch (state.phase)
  {
    case ShiftPhase::Idle:
      if (in.upRequest && in.downRequest)
      {
        state.lastReject = ShiftReject::BothPaddles;
      }
      else if (anyRequest)
      {
        const ShiftDirection direction = in.upRequest ? ShiftDirection::Up : ShiftDirection::Down;
        const ShiftReject reject = checkRequest(cfg, in, direction);
        if (reject == ShiftReject::None)
        {
          startShift(state, cfg, in, direction);
        }
        else
        {
          state.lastReject = reject;
        }
      }
      break;

    case ShiftPhase::PreCut:
      if (hasElapsed(now, state.phaseStartMs, cfg.cutLeadMs))
      {
        setPhase(state, ShiftPhase::Move, now);
      }
      break;

    case ShiftPhase::Move:
      if (cfg.useFeedback)
      {
        if (in.positionValid && isNear(in.positionPermille, targetPermille(cfg, state.direction), cfg.positionTolerance))
        {
          setPhase(state, ShiftPhase::Hold, now);
        }
        else if (hasElapsed(now, state.phaseStartMs, cfg.moveMs))
        {
          enterFault(state, ShiftFault::PositionTimeout, now);
        }
        else
        {
          // Keep waiting
        }
      }
      else if (hasElapsed(now, state.phaseStartMs, cfg.moveMs))
      {
        setPhase(state, ShiftPhase::Hold, now);
      }
      else
      {
        // Keep waiting
      }
      break;

    case ShiftPhase::Hold:
      if (hasElapsed(now, state.phaseStartMs, cfg.holdMs))
      {
        // The gear is engaged: release the cut and let the actuator go back
        state.cutActive = false;
        setPhase(state, ShiftPhase::Return, now);
      }
      break;

    case ShiftPhase::Return:
    {
      bool returned = false;
      if (cfg.useFeedback)
      {
        returned = in.positionValid && isNear(in.positionPermille, 0, cfg.positionTolerance);
        if (!returned && hasElapsed(now, state.phaseStartMs, cfg.returnMs))
        {
          enterFault(state, ShiftFault::PositionTimeout, now);
        }
      }
      else
      {
        returned = hasElapsed(now, state.phaseStartMs, cfg.returnMs);
      }

      if (returned)
      {
        if ((cfg.verifyMs > 0U) && state.expectGearKnown)
        {
          setPhase(state, ShiftPhase::Verify, now);
        }
        else
        {
          finishShift(state, now);
        }
      }
      break;
    }

    case ShiftPhase::Verify:
      if (in.gearValid && (in.gear == state.expectGear))
      {
        finishShift(state, now);
      }
      else if (hasElapsed(now, state.phaseStartMs, cfg.verifyMs))
      {
        enterFault(state, ShiftFault::GearNotReached, now);
      }
      else
      {
        // Keep waiting
      }
      break;

    case ShiftPhase::Lockout:
      if (hasElapsed(now, state.shiftStartMs, cfg.minIntervalMs))
      {
        setPhase(state, ShiftPhase::Idle, now);
      }
      break;

    case ShiftPhase::Fault:
      if (hasElapsed(now, state.phaseStartMs, cfg.faultLockoutMs))
      {
        setPhase(state, ShiftPhase::Idle, now);
      }
      break;

    default:
      // Unknown phase: treat it as a fault rather than carrying on
      enterFault(state, ShiftFault::SequenceTimeout, now);
      break;
  }

  // Outputs derive from the state after this step -----------------------------
  out.cutIgnition = state.cutActive && cfg.cutIgnition;
  out.cutFuel = state.cutActive && cfg.cutFuel;
  out.actuatorEnable = (state.phase != ShiftPhase::Fault);
  out.actuatorPermille = ((state.phase == ShiftPhase::Move) || (state.phase == ShiftPhase::Hold))
                       ? targetPermille(cfg, state.direction)
                       : static_cast<int16_t>(0);
  return out;
}

// ---------------------------------------------------------------------------
// Debounce
// ---------------------------------------------------------------------------

void debouncePrime(debounce_t &d, bool level, uint32_t nowMs)
{
  d.raw = level;
  d.stable = level;
  d.changedAtMs = nowMs;
}

bool debounceRisingEdge(debounce_t &d, bool pressed, uint32_t nowMs, uint16_t debounceMs)
{
  if (pressed != d.raw)
  {
    d.raw = pressed;
    d.changedAtMs = nowMs;
  }

  if ((d.raw != d.stable) && hasElapsed(nowMs, d.changedAtMs, debounceMs))
  {
    d.stable = d.raw;
    return d.stable;
  }
  return false;
}
