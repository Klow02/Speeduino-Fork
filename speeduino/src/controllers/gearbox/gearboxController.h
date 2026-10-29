#pragma once

/**
 * @file gearboxController.h
 * @brief Paddle shift controller: the glue between Speeduino (pins, clock, live values) and the
 *        hardware independent shift sequencer in gearboxStateMachine.h.
 *
 * ================================================================================================
 *  BACKGROUND FOR READERS WITHOUT PREVIOUS KNOWLEDGE
 * ================================================================================================
 *
 *  The car has a sequential gearbox that is shifted by an actuator (servo / motor) when the driver
 *  pulls a paddle behind the steering wheel. A good shift needs more than moving the lever:
 *
 *     1. cut the engine's ignition (and/or fuel) for a short time so the gear dogs unload,
 *     2. move the shift lever,
 *     3. hold, return, check that the new gear is really engaged,
 *     4. restore the engine and lock out the next shift for a moment.
 *
 *  gearboxStateMachine.h contains this sequence as plain logic with its safety interlocks (no
 *  shifting above the top gear, no over-revving, a hard cap on how long the engine may be cut, ...).
 *
 *  THIS file is the part that connects that logic to the real world:
 *     - reading the two paddle switches (with debouncing) and the optional neutral switch,
 *     - taking engine speed, gear and the gear ratios from Speeduino,
 *     - passing the actuator command to a driver function that the board code provides,
 *     - publishing the cut request so the main loop can switch ignition / fuel off.
 *
 *  Pins, clock and driver are reached through a small table of function pointers (gearboxHal_t),
 *  so everything here is tested on a PC with fake hardware. The Arduino version of those functions
 *  lives in gearboxHwArduino.cpp.
 *
 *  STATUS: the module is DISABLED by default. Its settings can only be changed in code
 *  (gearboxBoardSetup() in gearboxHwArduino.cpp), there is no TunerStudio page yet, and no
 *  actuator driver or position feedback is connected yet.
 */

#include <stdint.h>
#include <stdbool.h>
#include "gearboxStateMachine.h"

/** Value meaning "this pin is not connected". */
constexpr uint8_t GEARBOX_NO_PIN = 0xFFU;

/** @brief Electrical configuration of the gearbox inputs. */
struct gearbox_io_config_t
{
  bool paddlesActiveLow = true;  ///< Paddle switches pull the pin to ground when pressed (internal pull-up used).
  bool neutralActiveLow = true;  ///< Neutral switch pulls the pin to ground when the gearbox is in neutral.
};

/** @brief The Speeduino values the gearbox logic needs, copied into a plain struct. */
struct gearbox_live_t
{
  uint16_t rpm = 0U;                        ///< Engine speed.
  uint8_t  gear = 0U;                       ///< Gear as calculated by Speeduino's speed sensor code (0 = could not be worked out).
  bool     vssEnabled = false;              ///< Is a vehicle speed source configured at all?
  uint16_t ratio[GEARBOX_MAX_GEARS] = {};   ///< Speed ratio per gear (Speeduino's "vssRatio1..6" settings).
};

/**
 * @brief Actuator backend, called every step with the commanded state.
 *
 * Implement this for the servo / motor driver: map @p permille (+1000 full up, -1000 full down,
 * 0 centre) to a pulse width or position setpoint. When @p enable is false the driver must be
 * de-energised.
 */
using gearboxActuator_t = void (*)(bool enable, int16_t permille);

/** @brief The hardware functions the controller needs (Arduino version: gearboxHwArduino.cpp, tests: fakes). */
struct gearboxHal_t
{
  /** Configure a pin as input; with internal pull-up if @p pullup. */
  void (*setupInput)(uint8_t pin, bool pullup);
  /** Read a pin: true = HIGH. */
  bool (*readPin)(uint8_t pin);
  /** Milliseconds since start (wraps after ~49 days, handled). */
  uint32_t (*nowMs)(void);
};

/** @brief Everything the controller remembers. One instance exists on the real board. */
struct gearboxController_t
{
  gearbox_config_t    config;               ///< Shift timing, limits, enable flag (see gearboxStateMachine.h). Default: disabled.
  gearbox_io_config_t io;                   ///< Switch polarity.
  uint8_t             pinPaddleUp   = GEARBOX_NO_PIN;
  uint8_t             pinPaddleDown = GEARBOX_NO_PIN;
  uint8_t             pinNeutral    = GEARBOX_NO_PIN; ///< Optional.
  gearboxActuator_t   actuator = nullptr;   ///< nullptr = no driver (cut and status only).
  gearbox_state_t     state;                ///< Sequencer state.
  debounce_t          debounceUp;
  debounce_t          debounceDown;
  bool                primed = false;       ///< Debouncers have been set to the start-up switch levels.
  gearboxStatus_t     status;               ///< Latest published result (for the main loop, logging, TunerStudio later).
};

/**
 * @brief Claim the pins and check the setup.
 *
 * Both paddle pins are required: without them the module switches itself off (config.enabled = false).
 * The neutral pin is optional.
 */
void gearboxInit(gearboxController_t& ctrl, const gearboxHal_t& hal);

/**
 * @brief Builds the inputs of one sequencer step from the pins and the live values.
 *
 * The neutral switch, when fitted, wins over the speed/rpm based gear detection, because it is more
 * trustworthy. A gear of 0 from Speeduino means "unknown", not "neutral".
 */
gearbox_inputs_t gearboxBuildInputs(const gearboxController_t& ctrl, const gearboxHal_t& hal, const gearbox_live_t& live,
                                    uint32_t nowMs, bool upRequest, bool downRequest);

/** @brief One pass of the control: read paddles, run the sequencer, drive the actuator, publish the status. Call every main-loop pass. */
void gearboxUpdate(gearboxController_t& ctrl, const gearbox_live_t& live, const gearboxHal_t& hal);

/**
 * @brief Applies the shift cut to the ignition / fuel channel masks of Speeduino's main loop.
 *
 * It can only REMOVE channels. If the rev limiter has already cut something it stays cut; a
 * shift cut never switches anything back on.
 *
 * @param status        the controller's published status
 * @param ignitionOn    bit mask of ignition channels that are on (updated)
 * @param fuelOn        bit mask of fuel channels that are on (updated)
 * @param ignitionPending bit mask of ignition channels waiting to be re-enabled (cleared when ignition is cut)
 */
void gearboxApplyCut(const gearboxStatus_t& status, uint8_t& ignitionOn, uint8_t& fuelOn, uint8_t& ignitionPending);
