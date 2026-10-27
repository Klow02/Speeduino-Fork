/**
 * @file canTable.cpp
 * @brief Implementation of the configurable CAN message table. See canTable.h for the big picture.
 */

#include "canTable.h"
#include <string.h>

// ------------------------------------------------------------------------------------------------
//  Small helpers
// ------------------------------------------------------------------------------------------------

/**
 * Divides @p numerator by @p denominator and rounds to the NEAREST whole number
 * (halves are rounded away from zero). Plain "/" in C would cut off the decimals instead, which
 * would make 2.9 become 2. The denominator must not be 0 (checked by canTableValidate).
 */
static int64_t divideRounded(int64_t numerator, int64_t denominator)
{
  if (denominator < 0) { numerator = -numerator; denominator = -denominator; }   // make the denominator positive
  const int64_t half = denominator / 2;
  if (numerator >= 0) { return (numerator + half) / denominator; }
  return -((-numerator + half) / denominator);
}

/** Bit mask with the lowest @p bits bits set (bits = 1 ... 32). */
static uint32_t lowBitsMask(uint8_t bits)
{
  return (bits >= 32U) ? 0xFFFFFFFFUL : ((1UL << bits) - 1UL);
}

/** Bit mask of the payload bits a signal covers (bit n of the result = payload bit n). */
static uint64_t signalBitMask(const canTableSignal_t& sig)
{
  uint64_t mask = 0;
  for (uint8_t n = 0; n < sig.lengthBits; n++)
  {
    const uint8_t position = (uint8_t)(sig.startBit + n);
    if (position < 64U) { mask |= ((uint64_t)1 << position); }
  }
  return mask;
}

static bool messageIsOutput(const canTableMessage_t& msg)
{
  return ((msg.flags & CAN_MSG_FLAG_ENABLED) != 0U) && ((msg.flags & CAN_MSG_FLAG_INPUT) == 0U);
}

static uint8_t messageChannel(const canTableMessage_t& msg)
{
  return ((msg.flags & CAN_MSG_FLAG_CHANNEL1) != 0U) ? 1U : 0U;
}

// ------------------------------------------------------------------------------------------------
//  Clearing
// ------------------------------------------------------------------------------------------------

void canTableClear(canTableConfig_t& cfg)
{
  memset(&cfg, 0, sizeof(cfg));                               // everything 0 = disabled / empty
  for (uint8_t c = 0; c < CAN_TABLE_CHANNELS; c++)
  {
    cfg.channel[c].terminationPin = CAN_TABLE_NO_PIN;         // 0 would be a real pin, so mark "none" explicitly
  }
}

void canTableResetState(canTableState_t& state)
{
  memset(&state, 0, sizeof(state));
}

// ------------------------------------------------------------------------------------------------
//  Validation
// ------------------------------------------------------------------------------------------------

canTableError_t canTableValidate(const canTableConfig_t& cfg, uint8_t& index)
{
  index = 0;

  // ---- Messages ---------------------------------------------------------------------------
  for (uint8_t m = 0; m < CAN_TABLE_MAX_MESSAGES; m++)
  {
    const canTableMessage_t& msg = cfg.message[m];
    if ((msg.flags & CAN_MSG_FLAG_ENABLED) == 0U) { continue; }   // not in use: ignore
    index = m;

    if (msg.id > 0x7FFU) { return CAN_TABLE_ERR_ID; }
    if (messageIsOutput(msg) && (msg.intervalMs == 0U)) { return CAN_TABLE_ERR_INTERVAL; }
    if (cfg.channel[messageChannel(msg)].bitrateKbit == 0U) { return CAN_TABLE_ERR_BITRATE; }

    // The same ID must not be used twice for the same direction on the same channel.
    for (uint8_t other = 0; other < m; other++)
    {
      const canTableMessage_t& o = cfg.message[other];
      if (((o.flags & CAN_MSG_FLAG_ENABLED) != 0U)
          && (o.id == msg.id)
          && ((o.flags & CAN_MSG_FLAG_INPUT) == (msg.flags & CAN_MSG_FLAG_INPUT))
          && (messageChannel(o) == messageChannel(msg)))
      {
        return CAN_TABLE_ERR_DUPLICATE_ID;
      }
    }
  }

  // ---- Signals ----------------------------------------------------------------------------
  for (uint8_t s = 0; s < CAN_TABLE_MAX_SIGNALS; s++)
  {
    const canTableSignal_t& sig = cfg.signal[s];
    if ((sig.flags & CAN_SIG_FLAG_ENABLED) == 0U) { continue; }
    index = s;

    if ((sig.message >= CAN_TABLE_MAX_MESSAGES)
        || ((cfg.message[sig.message].flags & CAN_MSG_FLAG_ENABLED) == 0U))
    { return CAN_TABLE_ERR_SIGNAL_MESSAGE; }
    if (sig.source >= (uint8_t)CAN_SRC_COUNT) { return CAN_TABLE_ERR_SIGNAL_SOURCE; }
    if ((sig.lengthBits == 0U) || (sig.lengthBits > 32U)) { return CAN_TABLE_ERR_SIGNAL_LENGTH; }
    if (((uint16_t)sig.startBit + sig.lengthBits) > 64U) { return CAN_TABLE_ERR_SIGNAL_RANGE; }
    if (sig.divisor == 0) { return CAN_TABLE_ERR_SIGNAL_DIVISOR; }
    if (((sig.flags & CAN_SIG_FLAG_BIGENDIAN) != 0U)
        && (((sig.startBit % 8U) != 0U) || ((sig.lengthBits % 8U) != 0U)))
    { return CAN_TABLE_ERR_SIGNAL_ALIGN; }

    // No two signals of the same message may use the same bits.
    const uint64_t mask = signalBitMask(sig);
    for (uint8_t other = 0; other < s; other++)
    {
      const canTableSignal_t& o = cfg.signal[other];
      if (((o.flags & CAN_SIG_FLAG_ENABLED) != 0U) && (o.message == sig.message)
          && ((signalBitMask(o) & mask) != 0U))
      {
        return CAN_TABLE_ERR_SIGNAL_OVERLAP;
      }
    }
  }

  index = 0;
  return CAN_TABLE_OK;
}

// ------------------------------------------------------------------------------------------------
//  Value -> bits
// ------------------------------------------------------------------------------------------------

int32_t canTableToWire(const canTableSignal_t& sig, int32_t sourceValue)
{
  // 64-bit arithmetic so that value * multiplier can never overflow.
  const int64_t scaled = divideRounded((int64_t)sourceValue * (int64_t)sig.multiplier, (int64_t)sig.divisor)
                       + (int64_t)sig.offset;

  // Smallest and largest number that fits into the signal's bits.
  int64_t lowest, highest;
  if ((sig.flags & CAN_SIG_FLAG_SIGNED) != 0U)
  {
    lowest  = -((int64_t)1 << (sig.lengthBits - 1U));       // e.g. 8 bits: -128
    highest =  ((int64_t)1 << (sig.lengthBits - 1U)) - 1;   //              +127
  }
  else
  {
    lowest  = 0;                                            // e.g. 8 bits: 0
    highest = ((int64_t)1 << sig.lengthBits) - 1;           //              255
  }

  if (scaled < lowest)  { return (int32_t)lowest; }
  if (scaled > highest) { return (int32_t)highest; }
  return (int32_t)scaled;
}

void canTablePackSignal(const canTableSignal_t& sig, int32_t wire, uint8_t payload[8])
{
  // Negative numbers are stored in "two's complement": cutting the number down to its lowest
  // 'lengthBits' bits does exactly that. Example: -1 in 8 bits becomes 0xFF.
  const uint32_t bits = ((uint32_t)wire) & lowBitsMask(sig.lengthBits);

  if ((sig.flags & CAN_SIG_FLAG_BIGENDIAN) == 0U)
  {
    // Little endian ("Intel"): bit n of the number goes to payload bit (startBit + n).
    for (uint8_t n = 0; n < sig.lengthBits; n++)
    {
      const uint8_t position = (uint8_t)(sig.startBit + n);
      if (position > 63U) { return; }                          // outside the frame: ignore
      const uint8_t byteIndex = position / 8U;
      const uint8_t bitIndex  = position % 8U;
      if (((bits >> n) & 1U) != 0U) { payload[byteIndex] |= (uint8_t)(1U << bitIndex); }
      else                          { payload[byteIndex] &= (uint8_t)~(1U << bitIndex); }
    }
  }
  else
  {
    // Big endian ("Motorola"): whole bytes, most significant byte first (at the lower position).
    // Only allowed for byte-aligned signals (checked by canTableValidate).
    const uint8_t bytes = sig.lengthBits / 8U;
    for (uint8_t b = 0; b < bytes; b++)
    {
      const uint8_t byteIndex = (uint8_t)((sig.startBit / 8U) + b);
      if (byteIndex > 7U) { return; }
      payload[byteIndex] = (uint8_t)((bits >> (8U * (bytes - 1U - b))) & 0xFFU);
    }
  }
}

int32_t canTableUnpackSignal(const canTableSignal_t& sig, const uint8_t payload[8])
{
  uint32_t bits = 0;
  if ((sig.flags & CAN_SIG_FLAG_BIGENDIAN) == 0U)
  {
    for (uint8_t n = 0; n < sig.lengthBits; n++)
    {
      const uint8_t position = (uint8_t)(sig.startBit + n);
      if (position > 63U) { break; }
      if (((payload[position / 8U] >> (position % 8U)) & 1U) != 0U) { bits |= (1UL << n); }
    }
  }
  else
  {
    const uint8_t bytes = sig.lengthBits / 8U;
    for (uint8_t b = 0; b < bytes; b++)
    {
      const uint8_t byteIndex = (uint8_t)((sig.startBit / 8U) + b);
      if (byteIndex > 7U) { break; }
      bits = (bits << 8U) | payload[byteIndex];
    }
  }

  // Signed signals: if the top bit is set, the number is negative -> subtract 2^length.
  if (((sig.flags & CAN_SIG_FLAG_SIGNED) != 0U) && ((bits & (1UL << (sig.lengthBits - 1U))) != 0U))
  {
    return (int32_t)((int64_t)bits - ((int64_t)1 << sig.lengthBits));
  }
  return (int32_t)bits;
}

void canTableBuildPayload(const canTableConfig_t& cfg, uint8_t messageIndex, const canTableSources_t& src, uint8_t payload[8])
{
  memset(payload, 0, 8);                                       // unused bits stay 0

  for (uint8_t s = 0; s < CAN_TABLE_MAX_SIGNALS; s++)
  {
    const canTableSignal_t& sig = cfg.signal[s];
    if (((sig.flags & CAN_SIG_FLAG_ENABLED) == 0U) || (sig.message != messageIndex)) { continue; }

    // "No source" and unknown sources deliver 0 (the offset can still turn this into a fixed value).
    int32_t value = 0;
    if ((sig.source != (uint8_t)CAN_SRC_NONE) && (sig.source < (uint8_t)CAN_SRC_COUNT)) { value = src.value[sig.source]; }

    canTablePackSignal(sig, canTableToWire(sig, value), payload);
  }
}

// ------------------------------------------------------------------------------------------------
//  Sending
// ------------------------------------------------------------------------------------------------

uint8_t canTableProcess(const canTableConfig_t& cfg, canTableState_t& state, const canTableSources_t& src,
                        uint32_t nowMs, canTableSendFn_t send)
{
  if (send == nullptr) { return 0; }

  uint8_t sent = 0;
  for (uint8_t m = 0; m < CAN_TABLE_MAX_MESSAGES; m++)
  {
    const canTableMessage_t& msg = cfg.message[m];
    if (!messageIsOutput(msg) || (msg.intervalMs == 0U)) { continue; }          // only outgoing messages
    const uint8_t channel = messageChannel(msg);
    if (cfg.channel[channel].enabled == 0U) { continue; }                       // channel switched off

    // Is it time? "nowMs - last" stays correct when the millisecond counter wraps around (every ~49 days).
    const bool due = !state.sentOnce[m] || ((uint32_t)(nowMs - state.lastSentMs[m]) >= msg.intervalMs);
    if (!due) { continue; }

    uint8_t payload[8];
    canTableBuildPayload(cfg, m, src, payload);

    // The time is updated even if sending fails. That keeps the schedule steady and avoids
    // hammering a bus that is not connected.
    state.sentOnce[m] = true;
    state.lastSentMs[m] = nowMs;
    if (send(channel, msg.id, payload)) { sent++; }
    else { state.sendFailures++; }
  }
  return sent;
}

// ------------------------------------------------------------------------------------------------
//  Example table for the Bosch DDU9 display
// ------------------------------------------------------------------------------------------------

/** Helper: sets message number @p index. */
static void setMessage(canTableConfig_t& cfg, uint8_t index, uint16_t id, uint16_t intervalMs)
{
  cfg.message[index].id = id;
  cfg.message[index].intervalMs = intervalMs;
  cfg.message[index].flags = (uint8_t)(CAN_MSG_FLAG_ENABLED | CAN_MSG_FLAG_CHANNEL1);   // output, channel 1 (CAN3)
}

/** Helper: sets signal number @p index. flags: CAN_SIG_FLAG_SIGNED or 0. */
static void setSignal(canTableConfig_t& cfg, uint8_t index, uint8_t message, canTableSource_t source,
                      uint8_t startBit, uint8_t lengthBits, bool isSigned,
                      int16_t multiplier, int16_t divisor, int16_t offset)
{
  canTableSignal_t& sig = cfg.signal[index];
  sig.message = message;
  sig.source = (uint8_t)source;
  sig.startBit = startBit;
  sig.lengthBits = lengthBits;
  sig.flags = (uint8_t)(CAN_SIG_FLAG_ENABLED | (isSigned ? CAN_SIG_FLAG_SIGNED : 0U));
  sig.multiplier = multiplier;
  sig.divisor = divisor;
  sig.offset = offset;
}

void canTableLoadDdu9Example(canTableConfig_t& cfg)
{
  canTableClear(cfg);

  // CAN3 is the channel used for the display, 1 Mbit/s (as tested on the real DDU9).
  cfg.channel[1].enabled = 1;
  cfg.channel[1].bitrateKbit = 1000;

  // The four messages the display listens to: message index 0...3.
  setMessage(cfg, 0, 0x011,   50);
  setMessage(cfg, 1, 0x033,  100);
  setMessage(cfg, 2, 0x055, 1000);
  setMessage(cfg, 3, 0x066, 1000);

  // ---- Message 0x011 (every 50 ms) -------------------------------------------------------------
  //                      msg  source                    start len signed  mul div off
  setSignal(cfg,  0, 0, CAN_SRC_TPS_PERCENT,           40, 16, false,  10,  1,  0);  // throttle: 1.0 = 1000, so percent * 10
  setSignal(cfg,  1, 0, CAN_SRC_GEAR,                  16,  8, true,    1,  1,  0);  // gear
  setSignal(cfg,  2, 0, CAN_SRC_RPM,                    0, 16, false,   1,  1,  0);  // engine speed in rpm
  setSignal(cfg,  3, 0, CAN_SRC_MAP_KPA,               24, 16, false,  10,  1,  0);  // manifold pressure: kPa -> mbar (x10)

  // ---- Message 0x033 (every 100 ms) ------------------------------------------------------------
  setSignal(cfg,  4, 1, CAN_SRC_LAMBDA_MILLI,          24, 16, false,   1,  1,  0);  // lambda, 0.001 per step
  setSignal(cfg,  5, 1, CAN_SRC_OIL_PRESSURE_KPA,      16,  8, false,   1, 10,  0);  // oil pressure: kPa -> 0.1 bar steps
  setSignal(cfg,  6, 1, CAN_SRC_SPEED_KPH,              0, 16, false,   1,  1,  0);  // speed in km/h
  setSignal(cfg,  7, 1, CAN_SRC_USER1,                 40, 16, true,    1,  1,  0);  // steering angle in degrees (user value 1)

  // ---- Message 0x055 (every 1000 ms) -----------------------------------------------------------
  setSignal(cfg,  8, 2, CAN_SRC_USER2,                 24,  8, false,   1,  1,  0);  // selected engine map 0...3 (user value 2)
  setSignal(cfg,  9, 2, CAN_SRC_FUEL_TEMP_C,           32,  8, true,    1,  1,  0);  // fuel temperature, signed 8 bit
  setSignal(cfg, 10, 2, CAN_SRC_IAT_C,                  8,  8, true,    1,  1,  0);  // intake temperature
  setSignal(cfg, 11, 2, CAN_SRC_CLT_C,                 16,  8, true,    1,  1,  0);  // engine (coolant) temperature
  setSignal(cfg, 12, 2, CAN_SRC_USER3,                  0,  8, true,    1,  2,  0);  // oil temperature, 2 degrees per step (user value 3)

  // ---- Message 0x066 (every 1000 ms) -----------------------------------------------------------
  setSignal(cfg, 13, 3, CAN_SRC_FUEL_PRESSURE_KPA,     16,  8, false,   1, 10,  0);  // fuel pressure: kPa -> 0.1 bar steps
  setSignal(cfg, 14, 3, CAN_SRC_USER4,                  0, 16, true,    1,  1,  0);  // exhaust temperature in degrees C (user value 4)
  setSignal(cfg, 15, 3, CAN_SRC_NONE,                  24,  4, false,   1,  1,  2);  // ignition flag: no source, fixed value 2 via the offset (as in the tested table)
}
