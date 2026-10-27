/**
 * @file canTable.h
 * @brief Configurable CAN message table: decides WHICH values are sent in WHICH CAN frame.
 *
 * ================================================================================================
 *  BACKGROUND FOR READERS WITHOUT PREVIOUS KNOWLEDGE
 * ================================================================================================
 *
 *  CAN is a two-wire bus on which devices (ECU, dashboard display, ...) exchange short messages
 *  called FRAMES. A frame consists of
 *
 *    - an ID    : a number (here 0 ... 0x7FF) that tells the receivers WHAT the frame is about.
 *    - 8 bytes  : the payload, which is 64 bits of data.
 *
 *  One frame normally carries several measured values side by side, e.g. engine speed in bits 0-15,
 *  gear in bits 16-23 and so on. Such a value inside a frame is called a SIGNAL.
 *
 *  This module holds a TABLE that describes
 *     - the MESSAGES (frames): their ID, how often they are sent, on which CAN channel, in or out,
 *     - the SIGNALS: which Speeduino value goes into which bits of which message and how it is
 *       scaled.
 *
 *  The table is plain data, so it can later be edited in TunerStudio instead of changing code.
 *  By default the table is EMPTY: nothing is sent until the user adds messages and signals.
 *
 *  THIS FILE CONTAINS NO HARDWARE CODE. It only calculates payload bytes and decides WHEN a frame
 *  is due. The actual sending is done by a small function that is handed in by the caller
 *  (see canTableSendFn_t), so the logic can be tested on a normal PC.
 *
 * ================================================================================================
 *  HOW A VALUE BECOMES BITS  (the "scaling formula")
 * ================================================================================================
 *
 *  A CAN signal can only carry a whole number. For every signal we calculate
 *
 *       wire number = ( source value * multiplier / divisor ) + offset        (rounded)
 *
 *  Example: oil pressure source is in kPa, display wants 0.1 bar steps. 1 bar = 100 kPa, so one
 *  0.1 bar step = 10 kPa  ->  multiplier 1, divisor 10, offset 0.   (250 kPa -> 25 -> "2.5 bar")
 *
 *  Example: a temperature that the display wants as unsigned number with -40 offset:
 *  multiplier 1, divisor 1, offset +40.   (-10 C -> 30)
 *
 *  The wire number is then clamped to what fits into the signal's bit length (so a wrong scaling
 *  gives a "stuck at maximum" display instead of random garbage) and written to the payload bits.
 *
 * ================================================================================================
 *  CHANNELS
 * ================================================================================================
 *  The Teensy 4.1 has three CAN controllers. This table uses two of them:
 *     channel 0 = CAN1 (pins 22/23)   - also used by the existing Speeduino CAN code
 *     channel 1 = CAN3 (pins 30/31)
 *  Every channel can be switched on or off and has its own bitrate. Each message selects a channel.
 *  A CAN transceiver chip must be wired between the Teensy pins and the bus.
 */

#ifndef CAN_TABLE_H
#define CAN_TABLE_H

#include <stdint.h>
#include <stdbool.h>

// ------------------------------------------------------------------------------------------------
//  Table sizes. These limit how much memory the table needs (see canTableConfig_t).
// ------------------------------------------------------------------------------------------------

/** Maximum number of messages (frames) in the table. */
#define CAN_TABLE_MAX_MESSAGES   16U
/** Maximum number of signals (values) in the table. They can be spread freely over the messages. */
#define CAN_TABLE_MAX_SIGNALS    64U
/** Number of CAN channels this table can use (see "CHANNELS" above). */
#define CAN_TABLE_CHANNELS       2U

// ------------------------------------------------------------------------------------------------
//  Sources: the Speeduino values that can be put into a signal.
// ------------------------------------------------------------------------------------------------

/**
 * @brief Which Speeduino value a signal sends.
 *
 * The unit of each source is fixed and written behind it, so the multiplier/divisor of a signal
 * can be calculated from it. The values are filled into a canTableSources_t by the caller.
 */
typedef enum canTableSource_e
{
  CAN_SRC_NONE = 0,     ///< Nothing selected: the signal is sent as 0.
  CAN_SRC_RPM,          ///< Engine speed, in rpm.
  CAN_SRC_MAP_KPA,      ///< Intake manifold pressure, in kPa.
  CAN_SRC_TPS_PERCENT,  ///< Throttle position, in percent (0 ... 100).
  CAN_SRC_CLT_C,        ///< Coolant (engine) temperature, in degrees C.
  CAN_SRC_IAT_C,        ///< Intake air temperature, in degrees C.
  CAN_SRC_BATTERY_100MV,///< Battery voltage in units of 0.1 V (12.5 V = 125).
  CAN_SRC_SPEED_KPH,    ///< Vehicle speed, in km/h.
  CAN_SRC_GEAR,         ///< Current gear (0 = unknown/neutral, 1 ... n).
  CAN_SRC_OIL_PRESSURE_KPA,   ///< Oil pressure, in kPa.
  CAN_SRC_FUEL_PRESSURE_KPA,  ///< Fuel pressure, in kPa.
  CAN_SRC_FUEL_TEMP_C,  ///< Fuel temperature, in degrees C.
  CAN_SRC_LAMBDA_MILLI, ///< Lambda in thousandths (1.000 = 1000).
  // "User" values are free slots. They are meant to be filled by something else, for example by
  // incoming CAN messages (later) or by sensors that Speeduino does not know. They can also be
  // sent out again.
  CAN_SRC_USER1,
  CAN_SRC_USER2,
  CAN_SRC_USER3,
  CAN_SRC_USER4,
  CAN_SRC_USER5,
  CAN_SRC_USER6,
  CAN_SRC_USER7,
  CAN_SRC_USER8,
  CAN_SRC_COUNT         ///< Not a real source: the number of sources (used for range checks).
} canTableSource_t;

/** A snapshot of all source values at one moment. Index = canTableSource_t. */
typedef struct canTableSources_s
{
  int32_t value[CAN_SRC_COUNT];
} canTableSources_t;

// ------------------------------------------------------------------------------------------------
//  The table itself
// ------------------------------------------------------------------------------------------------

/** Bits of canTableMessage_t::flags. */
#define CAN_MSG_FLAG_ENABLED    0x01U  ///< Message is active. A cleared bit = message does not exist.
#define CAN_MSG_FLAG_INPUT      0x02U  ///< 0 = message is SENT by us (out), 1 = message is RECEIVED (in).
#define CAN_MSG_FLAG_CHANNEL1   0x04U  ///< 0 = channel 0 (CAN1), 1 = channel 1 (CAN3).

/** Bits of canTableSignal_t::flags. */
#define CAN_SIG_FLAG_ENABLED    0x01U  ///< Signal is active. A cleared bit = signal does not exist.
#define CAN_SIG_FLAG_SIGNED     0x02U  ///< The wire number may be negative (two's complement).
#define CAN_SIG_FLAG_BIGENDIAN  0x04U  ///< High byte first ("Motorola"). Default: low byte first ("Intel").

/** One message (frame). 6 bytes. */
typedef struct canTableMessage_s
{
  uint16_t id;          ///< CAN ID, 0 ... 0x7FF (11-bit standard ID).
  uint16_t intervalMs;  ///< Send this message again every this many milliseconds (outgoing messages only).
  uint8_t  flags;       ///< CAN_MSG_FLAG_* bits.
  uint8_t  reserved;    ///< Unused, keeps the structure 6 bytes long.
} canTableMessage_t;

/** One signal (value inside a message). 12 bytes. */
typedef struct canTableSignal_s
{
  uint8_t  message;     ///< Index into the message table (0 ... CAN_TABLE_MAX_MESSAGES-1).
  uint8_t  source;      ///< canTableSource_t: which Speeduino value is sent.
  uint8_t  startBit;    ///< First bit inside the 8-byte payload (0 ... 63). Bit 0 = least significant bit of byte 0.
  uint8_t  lengthBits;  ///< Number of bits (1 ... 32).
  uint8_t  flags;       ///< CAN_SIG_FLAG_* bits.
  uint8_t  reserved;    ///< Unused, keeps the structure 12 bytes long.
  int16_t  multiplier;  ///< Scaling: wire = value * multiplier / divisor + offset.
  int16_t  divisor;     ///< Must not be 0.
  int16_t  offset;
} canTableSignal_t;

/** Settings of one CAN channel. 6 bytes. */
typedef struct canTableChannel_s
{
  uint8_t  enabled;      ///< 0 = channel is switched off completely (nothing sent, hardware not started).
  uint8_t  terminationPin;///< Pin that switches an external 120 ohm termination (0xFF = none). See README of the CAN table.
  uint16_t bitrateKbit;  ///< Bus speed in kbit/s, e.g. 500 or 1000.
  uint16_t reserved;
} canTableChannel_t;

/**
 * @brief The whole table. It is empty (all flags zero) until messages and signals are added.
 *
 * Size: 16*6 + 64*12 + 2*6 = 876 bytes. This is what would be stored in a TunerStudio page.
 */
typedef struct canTableConfig_s
{
  canTableChannel_t channel[CAN_TABLE_CHANNELS];
  canTableMessage_t message[CAN_TABLE_MAX_MESSAGES];
  canTableSignal_t  signal[CAN_TABLE_MAX_SIGNALS];
} canTableConfig_t;

/** Values for "no termination pin". */
#define CAN_TABLE_NO_PIN 0xFFU

// ------------------------------------------------------------------------------------------------
//  Functions
// ------------------------------------------------------------------------------------------------

/** Result of canTableValidate(). */
typedef enum canTableError_e
{
  CAN_TABLE_OK = 0,
  CAN_TABLE_ERR_ID,            ///< Message ID larger than 0x7FF.
  CAN_TABLE_ERR_INTERVAL,      ///< Outgoing message with interval 0.
  CAN_TABLE_ERR_DUPLICATE_ID,  ///< Two enabled messages with same ID, direction and channel.
  CAN_TABLE_ERR_BITRATE,       ///< Message uses a channel whose bitrate is 0.
  CAN_TABLE_ERR_SIGNAL_MESSAGE,///< Signal points to a message index that does not exist or is not enabled.
  CAN_TABLE_ERR_SIGNAL_SOURCE, ///< Signal uses an unknown source.
  CAN_TABLE_ERR_SIGNAL_LENGTH, ///< Length is 0 or above 32.
  CAN_TABLE_ERR_SIGNAL_RANGE,  ///< Signal reaches beyond bit 63.
  CAN_TABLE_ERR_SIGNAL_DIVISOR,///< Divisor is 0.
  CAN_TABLE_ERR_SIGNAL_ALIGN,  ///< Big endian signal does not start/end on a byte border.
  CAN_TABLE_ERR_SIGNAL_OVERLAP ///< Two signals of the same message use the same bits.
} canTableError_t;

/**
 * @brief Function that really sends a frame. Provided by the caller (the hardware side).
 * @param channel 0 = CAN1, 1 = CAN3
 * @param id      CAN ID
 * @param data    8 payload bytes
 * @return true if the frame was accepted for sending.
 */
typedef bool (*canTableSendFn_t)(uint8_t channel, uint16_t id, const uint8_t data[8]);

/** Runtime memory of the table (when each message was last sent). */
typedef struct canTableState_s
{
  uint32_t lastSentMs[CAN_TABLE_MAX_MESSAGES];
  bool     sentOnce[CAN_TABLE_MAX_MESSAGES]; ///< false until a message has been sent at least once.
  uint32_t sendFailures;                     ///< Counts frames the send function refused. For diagnosis.
} canTableState_t;

/** Sets the whole table to "empty": nothing enabled, no termination pin, bitrates 0. */
void canTableClear(canTableConfig_t& cfg);

/** Resets the runtime memory. */
void canTableResetState(canTableState_t& state);

/**
 * @brief Checks the table for mistakes.
 * @param cfg   the table
 * @param index set to the index of the faulty message/signal (if an error is returned)
 * @return CAN_TABLE_OK or the first problem found.
 * A table that does not pass must not be used for sending.
 */
canTableError_t canTableValidate(const canTableConfig_t& cfg, uint8_t& index);

/**
 * @brief Converts one value to its wire number (formula above), including clamping to the bit length.
 */
int32_t canTableToWire(const canTableSignal_t& sig, int32_t sourceValue);

/** Writes a wire number into the right bits of the payload (other bits stay unchanged). */
void canTablePackSignal(const canTableSignal_t& sig, int32_t wire, uint8_t payload[8]);

/** Reads a wire number back out of a payload (the reverse of canTablePackSignal). Used by the tests and later for incoming messages. */
int32_t canTableUnpackSignal(const canTableSignal_t& sig, const uint8_t payload[8]);

/** Builds the 8 payload bytes of message number @p messageIndex from the current source values. */
void canTableBuildPayload(const canTableConfig_t& cfg, uint8_t messageIndex, const canTableSources_t& src, uint8_t payload[8]);

/**
 * @brief Call this often (e.g. every loop). Sends every outgoing message whose time has come.
 *
 * Messages of a disabled channel, input messages and disabled messages are skipped.
 * This function does NOT check the table for mistakes (that would be wasted time on every call):
 * call canTableValidate() after every change of the table and only call this if it returned CAN_TABLE_OK.
 * @return number of frames handed to the send function in this call.
 */
uint8_t canTableProcess(const canTableConfig_t& cfg, canTableState_t& state, const canTableSources_t& src,
                        uint32_t nowMs, canTableSendFn_t send);

/**
 * @brief Fills @p cfg with the example table for the Bosch DDU9 display (the table that was tested on the display).
 *
 * DDU9 messages 0x011, 0x033, 0x055 and 0x066 on channel 1 (CAN3) at 1000 kbit/s.
 * Signals that Speeduino has no value for (steering angle, oil temperature, exhaust temperature, ...)
 * use the USER sources, so they can be filled from elsewhere.
 */
void canTableLoadDdu9Example(canTableConfig_t& cfg);

#endif // CAN_TABLE_H
