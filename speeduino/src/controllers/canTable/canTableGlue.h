/**
 * @file canTableGlue.h
 * @brief Connects the CAN message table (canTable.h) with the rest of Speeduino and the CAN hardware.
 *
 * ================================================================================================
 *  BACKGROUND FOR READERS WITHOUT PREVIOUS KNOWLEDGE
 * ================================================================================================
 *
 *  canTable.h only does calculations: "which value goes into which bits of which CAN frame, and
 *  when is a frame due". It never touches a pin or a CAN controller, so it can be tested on a PC.
 *
 *  THIS module is the "glue" around it. It does four things:
 *
 *   1. SOURCES   Takes the live engine values (rpm, temperatures, ...) from Speeduino's global
 *                status and converts them into the units the table expects (canTableSources_t).
 *   2. CHECKING  Notices when the table was changed (for example by TunerStudio writing page 16),
 *                checks it with canTableValidate() and only sends if it is free of mistakes.
 *   3. CHANNELS  Starts the CAN channels that are switched on, at their configured bitrate, and
 *                switches the optional external 120 ohm termination resistor.
 *   4. SENDING   Calls canTableProcess(), which sends every frame that is due.
 *
 *  All hardware access goes through a small table of function pointers (canTableHal_t). On the
 *  Teensy 4.1 the real functions are in canTableHwTeensy41.cpp; the tests on the PC hand in fake
 *  ones. Because of this the logic below is fully tested without any hardware.
 *
 *  Called from the Speeduino main loop through canTableTask() (see the bottom of this file).
 */
#ifndef CAN_TABLE_GLUE_H
#define CAN_TABLE_GLUE_H

#include <stdint.h>
#include <stdbool.h>
#include "canTable.h"

/**
 * @brief The live engine values the table can send, in Speeduino's own units.
 *
 * A plain copy of the few numbers needed. It is filled from Speeduino's global status object by
 * the hardware side (canTableHwTeensy41.cpp). Keeping it separate means this module (and its tests)
 * does not depend on the rest of Speeduino at all.
 */
typedef struct canTableLive_s
{
  uint16_t rpm;             ///< Engine speed, rpm.
  uint16_t mapKpa;          ///< Manifold pressure, kPa.
  uint8_t  tpsPercent;      ///< Throttle, 0 ... 100 %.
  int16_t  cltC;            ///< Coolant temperature, degrees C (negative allowed).
  int16_t  iatC;            ///< Intake air temperature, degrees C.
  uint8_t  battery100mV;    ///< Battery voltage in 0.1 V (12.5 V = 125).
  uint16_t speedKph;        ///< Vehicle speed, km/h.
  uint8_t  gear;            ///< Current gear.
  uint8_t  oilPressurePsi;  ///< Oil pressure in PSI (Speeduino keeps PSI).
  uint8_t  fuelPressurePsi; ///< Fuel pressure in PSI.
  int16_t  fuelTempC;       ///< Fuel temperature, degrees C.
  uint8_t  afrX10;          ///< Measured air/fuel ratio times 10 (Speeduino "O2").
  uint8_t  stoichX10;       ///< Stoichiometric air/fuel ratio times 10 (e.g. 147 for gasoline). 0 = unknown.
} canTableLive_t;

// ------------------------------------------------------------------------------------------------
//  1. Sources
// ------------------------------------------------------------------------------------------------

/**
 * @brief Converts Speeduino's live values to the units of canTableSource_t.
 *
 * Only the real engine values are written. The USER1...USER8 slots are NOT touched, so whatever
 * somebody else put there (later: incoming CAN messages) survives.
 *
 * Conversions:
 *   - oil and fuel pressure: Speeduino keeps PSI, the table wants kPa  (1 psi = 6.895 kPa, rounded).
 *   - lambda: Speeduino keeps the air/fuel ratio times 10 ("O2") and the stoichiometric ratio
 *     times 10 ("stoich"); lambda x 1000 = afrX10 * 1000 / stoichX10.  (14.7 AFR on gasoline = 1000.)
 *   - everything else is passed on unchanged (temperatures are already in degrees Celsius).
 *
 * @param live  Speeduino's current values (including the stoichiometric ratio for lambda).
 * @param out   Receives the values.
 */
void canTableFillSources(const canTableLive_t& live, canTableSources_t& out);

// ------------------------------------------------------------------------------------------------
//  2./3./4. Hardware abstraction and the controller
// ------------------------------------------------------------------------------------------------

/**
 * @brief The hardware functions the controller needs. The Teensy 4.1 version lives in
 *        canTableHwTeensy41.cpp, the tests use fakes. Every pointer must be set.
 */
typedef struct canTableHal_s
{
  /** Start (or re-configure) a CAN channel. channel 0 = CAN1, 1 = CAN3. @return true if it worked. */
  bool (*beginChannel)(uint8_t channel, uint16_t bitrateKbit);
  /** Send one standard-ID frame with 8 data bytes. @return true if the frame was accepted. */
  bool (*sendFrame)(uint8_t channel, uint16_t id, const uint8_t data[8]);
  /** Is this pin free to be used as termination switch? (false = Speeduino uses it for something else) */
  bool (*pinIsFree)(uint8_t pin);
  /** Drive the termination pin: on = true switches the external 120 ohm resistor IN. */
  void (*setTerminationPin)(uint8_t pin, bool on);
} canTableHal_t;

/** How often (ms) the controller looks whether the table was changed. Cheap, but not needed every loop. */
#define CAN_TABLE_CHANGE_CHECK_MS  250U

/** Runtime memory of the controller. Create one (zero it with canTableControllerReset) and keep it. */
typedef struct canTableController_s
{
  canTableState_t   state;        ///< When each message was last sent (see canTable.h).
  canTableSources_t sources;      ///< Latest source values. USER slots are kept between calls.
  uint32_t          cfgHash;      ///< Fingerprint of the table when it was last checked.
  uint32_t          lastCheckMs;  ///< Time of the last change check.
  canTableError_t   lastError;    ///< Result of the last validation (CAN_TABLE_OK = table is usable).
  uint8_t           errorIndex;   ///< Which message/signal the last error is about.
  bool              configKnown;  ///< false until the table was checked for the first time.
  bool              sending;      ///< true while the table is valid and frames are being sent.
  bool              channelUp[CAN_TABLE_CHANNELS];     ///< Channel is started and used.
  uint8_t           terminationPin[CAN_TABLE_CHANNELS];///< Pin currently driven for this channel (0xFF = none).
} canTableController_t;

/** Puts the controller in its start state: nothing known, nothing sending, all pins released. */
void canTableControllerReset(canTableController_t& ctrl);

/**
 * @brief The work of one main-loop pass. Call it as often as you like (every loop is fine).
 *
 *   - every CAN_TABLE_CHANGE_CHECK_MS: if the table changed, validate it and (re)start the channels
 *     and termination pins accordingly;
 *   - if the table is valid: refresh the source values and send all frames that are due.
 *
 * An invalid table (see ctrl.lastError) sends nothing at all and keeps all channels and
 * termination pins off. Messages already sent stay sent; there is nothing to undo on the bus.
 *
 * @param ctrl    controller memory
 * @param cfg     the table (Speeduino: configPage16.can)
 * @param live    live engine values
 * @param hal     hardware functions
 * @param nowMs   current time in milliseconds (wraps after ~49 days; handled)
 */
void canTableControllerUpdate(canTableController_t& ctrl, const canTableConfig_t& cfg, const canTableLive_t& live,
                              const canTableHal_t& hal, uint32_t nowMs);

// ------------------------------------------------------------------------------------------------
//  Entry point for the Speeduino main loop
// ------------------------------------------------------------------------------------------------

#if defined(CORE_TEENSY41) && !defined(UNIT_TEST)
/** Runs canTableControllerUpdate() with the real Teensy 4.1 hardware. Call once per main-loop pass. */
void canTableTask(void);
#else
/** On every other board (and in tests) the CAN table is not available: nothing to do. */
static inline void canTableTask(void) {}
#endif

#endif // CAN_TABLE_GLUE_H
