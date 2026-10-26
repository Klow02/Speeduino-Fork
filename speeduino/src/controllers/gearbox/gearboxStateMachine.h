#pragma once

/**
 * @file
 * @brief Hardware independent shift sequencer for a paddle shifted gearbox
 *        moved by an electric motor or servo.
 *
 * Nothing in here touches pins, timers or Arduino APIs: the current time and all
 * inputs are passed in, and the commanded outputs are returned. That keeps the
 * safety relevant logic (interlocks, timing, failsafes) unit testable on a host PC.
 *
 * Sequence for one shift:
 *
 *   Idle -> [PreCut] -> Move -> Hold -> Return -> [Verify] -> Lockout -> Idle
 *
 * - PreCut:  engine cut is asserted and the actuator waits, unloading the dogs (upshift only by default).
 * - Move:    actuator is driven to the shift target.
 * - Hold:    actuator stays at the target while the gear engages. Cut is still active.
 * - Return:  cut is released and the actuator goes back to centre.
 * - Verify:  optional, waits for the gear input to report the expected gear.
 * - Lockout: enforces a minimum time between shifts.
 *
 * Any failure drops into Fault: cut off, actuator disabled, for a configurable time.
 *
 * The actuator is described as a signed position in permille of full travel:
 * +1000 = full "up" travel, 0 = centre (rest), -1000 = full "down" travel.
 * Mapping that to a servo pulse or an H-bridge/position loop is the job of an actuator backend.
 */

#include <stdint.h>

constexpr uint8_t GEARBOX_MAX_GEARS = 6U;
constexpr int16_t GEARBOX_PERMILLE_FULL = 1000;

enum class ShiftDirection : uint8_t
{
  None = 0U,
  Up,
  Down
};

enum class ShiftPhase : uint8_t
{
  Idle = 0U,
  PreCut,
  Move,
  Hold,
  Return,
  Verify,
  Lockout,
  Fault
};

enum class ShiftFault : uint8_t
{
  None = 0U,
  SequenceTimeout,  ///< The whole shift took longer than gearbox_config_t::maxShiftMs
  PositionTimeout,  ///< Position feedback never reached the target (or centre)
  GearNotReached    ///< The gear input did not report the expected gear in time
};

enum class ShiftReject : uint8_t
{
  None = 0U,
  Disabled,
  Busy,         ///< A shift is in progress
  Lockout,      ///< Minimum time between shifts has not elapsed
  Fault,        ///< Recovering from a fault
  BothPaddles,  ///< Up and down requested in the same step
  TopGear,
  BottomGear,
  OverRev       ///< Downshift would push RPM above the configured limit
};

/**
 * @brief Tunable parameters.
 *
 * Plain integers on purpose so this can become a TunerStudio page without
 * changing the logic. All times are milliseconds.
 *
 * The defaults are conservative and the gearbox is disabled.
 */
struct gearbox_config_t
{
  bool enabled = false;

  /// Highest gear the box has (1 to GEARBOX_MAX_GEARS)
  uint8_t maxGear = 5U;
  /// Allow a downshift paddle press in 1st gear (moves to neutral)
  bool allowDownToNeutral = false;

  // ---- Engine cut ----
  bool cutOnUpshift = true;
  bool cutOnDownshift = false;
  bool cutIgnition = true;
  bool cutFuel = false;
  /// No cut below this RPM
  uint16_t cutMinRpm = 3000U;
  /// Absolute limit on how long the cut can be held, whatever the sequence is doing. 0 = never cut.
  uint16_t maxCutMs = 150U;

  // ---- Downshift protection ----
  /// A downshift is refused if the predicted RPM after the shift is above this. 0 = no check.
  uint16_t downshiftMaxRpm = 0U;

  // ---- Actuator ----
  /// Travel for an upshift, permille of full travel (0 to 1000)
  uint16_t upTravel = 800U;
  /// Travel for a downshift, permille of full travel (0 to 1000)
  uint16_t downTravel = 800U;
  /// If true, the sequence waits on position feedback instead of fixed times
  bool useFeedback = false;
  /// How close (permille) the feedback has to be to the target
  uint16_t positionTolerance = 50U;

  // ---- Timing ----
  uint16_t debounceMs = 20U;
  /// Cut is asserted this long before the actuator moves
  uint16_t cutLeadMs = 15U;
  /// Time allowed for the move. Fixed travel time without feedback, timeout with feedback.
  uint16_t moveMs = 80U;
  /// Time the actuator stays at the target
  uint16_t holdMs = 30U;
  /// Time allowed for the return to centre. Fixed time without feedback, timeout with feedback.
  uint16_t returnMs = 80U;
  /// How long to wait for the gear input to report the new gear. 0 = no verification.
  uint16_t verifyMs = 0U;
  /// Minimum time from the start of one shift to the start of the next
  uint16_t minIntervalMs = 250U;
  /// Watchdog for the whole sequence
  uint16_t maxShiftMs = 600U;
  /// How long to stay in Fault before accepting paddle requests again
  uint16_t faultLockoutMs = 1000U;
};

/** @brief Inputs for one step of the state machine */
struct gearbox_inputs_t
{
  uint32_t nowMs = 0U;
  /// Debounced one-shot: true for exactly one step when the up paddle was pressed
  bool upRequest = false;
  /// Debounced one-shot: true for exactly one step when the down paddle was pressed
  bool downRequest = false;
  uint16_t rpm = 0U;
  /// Current gear. Only meaningful if gearValid. 0 = neutral.
  uint8_t gear = 0U;
  /// False if the gear is unknown (e.g. no usable speed signal)
  bool gearValid = false;
  /// Speed per rpm for each gear, same units as the existing VSS ratio table (0 = unknown)
  uint16_t ratio[GEARBOX_MAX_GEARS] = {};
  bool positionValid = false;
  /// Measured actuator position, permille, same sign convention as the command
  int16_t positionPermille = 0;
};

/** @brief State kept between steps */
struct gearbox_state_t
{
  ShiftPhase phase = ShiftPhase::Idle;
  ShiftDirection direction = ShiftDirection::None;
  ShiftFault lastFault = ShiftFault::None;
  ShiftReject lastReject = ShiftReject::None;
  uint32_t phaseStartMs = 0U;
  uint32_t shiftStartMs = 0U;
  uint32_t cutStartMs = 0U;
  bool cutActive = false;
  bool expectGearKnown = false;
  uint8_t expectGear = 0U;
  uint16_t shiftCount = 0U;
};

/** @brief Commands produced by one step */
struct gearbox_outputs_t
{
  bool cutIgnition = false;
  bool cutFuel = false;
  bool actuatorEnable = false;
  /// Commanded position, permille. +up, -down, 0 centre.
  int16_t actuatorPermille = 0;
};

/** @brief Published status (lives in the statuses struct, available for logging) */
struct gearboxStatus_t
{
  ShiftPhase phase = ShiftPhase::Idle;
  ShiftDirection direction = ShiftDirection::None;
  ShiftFault lastFault = ShiftFault::None;
  ShiftReject lastReject = ShiftReject::None;
  bool cutIgnition = false;
  bool cutFuel = false;
  bool actuatorEnable = false;
  int16_t actuatorPermille = 0;
  uint16_t shiftCount = 0U;
};

/**
 * @brief Run one step of the shift sequencer.
 *
 * Call this often (every main loop pass is fine). Time resolution of the sequence is
 * the call interval, so keep it well below the shortest configured time.
 *
 * @param state     Sequencer state, updated in place
 * @param cfg       Configuration
 * @param in        Inputs for this step
 * @return          Commands to apply
 */
gearbox_outputs_t gearboxStep(gearbox_state_t &state, const gearbox_config_t &cfg, const gearbox_inputs_t &in);

/** @brief Debounce state for one switch */
struct debounce_t
{
  bool raw = false;
  bool stable = false;
  uint32_t changedAtMs = 0U;
};

/** @brief Set the debouncer to the current level, so a switch held at start-up does not count as a press */
void debouncePrime(debounce_t &d, bool level, uint32_t nowMs);

/**
 * @brief Update a debouncer.
 * @return true for exactly one call when the debounced level goes from released to pressed.
 */
bool debounceRisingEdge(debounce_t &d, bool pressed, uint32_t nowMs, uint16_t debounceMs);
