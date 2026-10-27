/**
 * Tests of the CAN message table. The test setup is described in each test's name.
 * All tests run on a PC: the table logic has no hardware access.
 */
#include <unity.h>
#include <string.h>
#include "src/controllers/canTable/canTable.h"

// ------------------------------------------------------------------------------------------------
//  Helpers
// ------------------------------------------------------------------------------------------------

/** A table with one enabled output message (ID 0x123, 100 ms, channel 0 enabled at 500 kbit/s) and no signals. */
static canTableConfig_t baseTable(void)
{
  canTableConfig_t cfg;
  canTableClear(cfg);
  cfg.channel[0].enabled = 1;
  cfg.channel[0].bitrateKbit = 500;
  cfg.message[0].id = 0x123;
  cfg.message[0].intervalMs = 100;
  cfg.message[0].flags = CAN_MSG_FLAG_ENABLED;
  return cfg;
}

/** Adds an enabled signal to message 0 at table position @p index. */
static void addSignal(canTableConfig_t& cfg, uint8_t index, canTableSource_t source, uint8_t startBit,
                      uint8_t length, bool isSigned, int16_t mul = 1, int16_t div = 1, int16_t off = 0)
{
  canTableSignal_t& s = cfg.signal[index];
  s.message = 0;
  s.source = (uint8_t)source;
  s.startBit = startBit;
  s.lengthBits = length;
  s.flags = (uint8_t)(CAN_SIG_FLAG_ENABLED | (isSigned ? CAN_SIG_FLAG_SIGNED : 0U));
  s.multiplier = mul;
  s.divisor = div;
  s.offset = off;
}

static canTableError_t validate(const canTableConfig_t& cfg)
{
  uint8_t index;
  return canTableValidate(cfg, index);
}

// ------------------------------------------------------------------------------------------------
//  Validation
// ------------------------------------------------------------------------------------------------

static void test_emptyTable_isValid_andHasNoTerminationPins(void)
{
  canTableConfig_t cfg;
  canTableClear(cfg);
  TEST_ASSERT_EQUAL(CAN_TABLE_OK, validate(cfg));
  TEST_ASSERT_EQUAL_UINT8(CAN_TABLE_NO_PIN, cfg.channel[0].terminationPin);
  TEST_ASSERT_EQUAL_UINT8(CAN_TABLE_NO_PIN, cfg.channel[1].terminationPin);
  TEST_ASSERT_EQUAL_UINT8(0, cfg.channel[0].enabled);
}

static void test_validate_messages(void)
{
  canTableConfig_t cfg = baseTable();
  TEST_ASSERT_EQUAL(CAN_TABLE_OK, validate(cfg));

  cfg.message[0].id = 0x7FF;                       // largest standard ID: fine
  TEST_ASSERT_EQUAL(CAN_TABLE_OK, validate(cfg));
  cfg.message[0].id = 0x800;
  TEST_ASSERT_EQUAL(CAN_TABLE_ERR_ID, validate(cfg));

  cfg = baseTable();
  cfg.message[0].intervalMs = 0;                   // outgoing message needs an interval
  TEST_ASSERT_EQUAL(CAN_TABLE_ERR_INTERVAL, validate(cfg));
  cfg.message[0].flags |= CAN_MSG_FLAG_INPUT;      // an incoming message does not
  TEST_ASSERT_EQUAL(CAN_TABLE_OK, validate(cfg));

  cfg = baseTable();
  cfg.channel[0].bitrateKbit = 0;                  // channel without bitrate
  TEST_ASSERT_EQUAL(CAN_TABLE_ERR_BITRATE, validate(cfg));

  cfg = baseTable();                               // same ID twice, same direction and channel
  cfg.message[3] = cfg.message[0];
  uint8_t index = 0;
  TEST_ASSERT_EQUAL(CAN_TABLE_ERR_DUPLICATE_ID, canTableValidate(cfg, index));
  TEST_ASSERT_EQUAL_UINT8(3, index);
  cfg.message[3].flags |= CAN_MSG_FLAG_INPUT;      // other direction: allowed
  TEST_ASSERT_EQUAL(CAN_TABLE_OK, validate(cfg));
  cfg.message[3].flags = (uint8_t)(CAN_MSG_FLAG_ENABLED | CAN_MSG_FLAG_CHANNEL1);   // other channel: allowed
  cfg.channel[1].bitrateKbit = 1000;
  TEST_ASSERT_EQUAL(CAN_TABLE_OK, validate(cfg));
}

static void test_validate_signals(void)
{
  canTableConfig_t cfg = baseTable();
  addSignal(cfg, 0, CAN_SRC_RPM, 0, 16, false);
  TEST_ASSERT_EQUAL(CAN_TABLE_OK, validate(cfg));

  addSignal(cfg, 1, CAN_SRC_RPM, 16, 0, false);            // length 0
  TEST_ASSERT_EQUAL(CAN_TABLE_ERR_SIGNAL_LENGTH, validate(cfg));
  addSignal(cfg, 1, CAN_SRC_RPM, 16, 33, false);           // length 33
  TEST_ASSERT_EQUAL(CAN_TABLE_ERR_SIGNAL_LENGTH, validate(cfg));
  addSignal(cfg, 1, CAN_SRC_RPM, 32, 32, false);           // ends exactly at bit 63: fine
  TEST_ASSERT_EQUAL(CAN_TABLE_OK, validate(cfg));
  addSignal(cfg, 1, CAN_SRC_RPM, 33, 32, false);           // one bit too far
  TEST_ASSERT_EQUAL(CAN_TABLE_ERR_SIGNAL_RANGE, validate(cfg));

  addSignal(cfg, 1, CAN_SRC_RPM, 16, 8, false, 1, 0);      // divisor 0
  TEST_ASSERT_EQUAL(CAN_TABLE_ERR_SIGNAL_DIVISOR, validate(cfg));

  addSignal(cfg, 1, CAN_SRC_COUNT, 16, 8, false);          // source out of range
  TEST_ASSERT_EQUAL(CAN_TABLE_ERR_SIGNAL_SOURCE, validate(cfg));

  addSignal(cfg, 1, CAN_SRC_RPM, 15, 8, false);            // overlaps bit 15 of the first signal (bits 0-15)
  TEST_ASSERT_EQUAL(CAN_TABLE_ERR_SIGNAL_OVERLAP, validate(cfg));
  addSignal(cfg, 1, CAN_SRC_RPM, 16, 8, false);            // directly behind it: fine
  TEST_ASSERT_EQUAL(CAN_TABLE_OK, validate(cfg));

  addSignal(cfg, 1, CAN_SRC_RPM, 16, 8, false);
  cfg.signal[1].message = 5;                               // message 5 does not exist
  TEST_ASSERT_EQUAL(CAN_TABLE_ERR_SIGNAL_MESSAGE, validate(cfg));
  cfg.signal[1].message = 16;                              // beyond the table
  TEST_ASSERT_EQUAL(CAN_TABLE_ERR_SIGNAL_MESSAGE, validate(cfg));

  // the same bits in a DIFFERENT message are fine
  cfg.message[1].id = 0x124; cfg.message[1].intervalMs = 50; cfg.message[1].flags = CAN_MSG_FLAG_ENABLED;
  addSignal(cfg, 1, CAN_SRC_RPM, 0, 16, false);
  cfg.signal[1].message = 1;
  TEST_ASSERT_EQUAL(CAN_TABLE_OK, validate(cfg));

  // big endian only on byte borders
  addSignal(cfg, 1, CAN_SRC_RPM, 20, 16, false);
  cfg.signal[1].message = 1;
  cfg.signal[1].flags |= CAN_SIG_FLAG_BIGENDIAN;
  TEST_ASSERT_EQUAL(CAN_TABLE_ERR_SIGNAL_ALIGN, validate(cfg));
  cfg.signal[1].startBit = 16;
  TEST_ASSERT_EQUAL(CAN_TABLE_OK, validate(cfg));
  cfg.signal[1].lengthBits = 12;
  TEST_ASSERT_EQUAL(CAN_TABLE_ERR_SIGNAL_ALIGN, validate(cfg));

  // a disabled signal is ignored even if it is nonsense
  cfg.signal[1].flags = 0;
  TEST_ASSERT_EQUAL(CAN_TABLE_OK, validate(cfg));
}

void testCanTableValidate(void)
{
  RUN_TEST(test_emptyTable_isValid_andHasNoTerminationPins);
  RUN_TEST(test_validate_messages);
  RUN_TEST(test_validate_signals);
}

// ------------------------------------------------------------------------------------------------
//  Scaling: value -> wire number
// ------------------------------------------------------------------------------------------------

static canTableSignal_t sig(uint8_t length, bool isSigned, int16_t mul, int16_t div, int16_t off)
{
  canTableSignal_t s;
  memset(&s, 0, sizeof(s));
  s.flags = (uint8_t)(CAN_SIG_FLAG_ENABLED | (isSigned ? CAN_SIG_FLAG_SIGNED : 0U));
  s.lengthBits = length;
  s.multiplier = mul;
  s.divisor = div;
  s.offset = off;
  return s;
}

static void test_scaling_roundsToNearest_notDown(void)
{
  const canTableSignal_t tenth = sig(16, false, 1, 10, 0);          // kPa -> 0.1 bar steps
  TEST_ASSERT_EQUAL_INT32(25, canTableToWire(tenth, 250));
  TEST_ASSERT_EQUAL_INT32(25, canTableToWire(tenth, 254));          // 25.4 -> 25
  TEST_ASSERT_EQUAL_INT32(26, canTableToWire(tenth, 255));          // 25.5 -> 26 (half rounds up)
  TEST_ASSERT_EQUAL_INT32(26, canTableToWire(tenth, 259));          // 25.9 -> 26, not 25

  const canTableSignal_t signedTenth = sig(16, true, 1, 10, 0);
  TEST_ASSERT_EQUAL_INT32(-25, canTableToWire(signedTenth, -254));
  TEST_ASSERT_EQUAL_INT32(-26, canTableToWire(signedTenth, -255));  // half rounds away from zero
}

static void test_scaling_multiplierDivisorOffsetAndNegativeDivisor(void)
{
  TEST_ASSERT_EQUAL_INT32(1000, canTableToWire(sig(16, false, 10, 1, 0), 100));   // percent -> 0.1 % steps
  TEST_ASSERT_EQUAL_INT32(30,   canTableToWire(sig(8, false, 1, 1, 40), -10));    // temperature with +40 offset
  TEST_ASSERT_EQUAL_INT32(2,    canTableToWire(sig(4, false, 1, 1, 2), 0));       // fixed value through the offset
  TEST_ASSERT_EQUAL_INT32(75,   canTableToWire(sig(8, true, 1, 2, 0), 150));      // 2 degrees per step
  TEST_ASSERT_EQUAL_INT32(-50,  canTableToWire(sig(16, true, 1, -2, 0), 100));    // negative divisor inverts
}

static void test_scaling_clampsToTheBitLength(void)
{
  const canTableSignal_t u8 = sig(8, false, 1, 1, 0);
  TEST_ASSERT_EQUAL_INT32(255, canTableToWire(u8, 255));
  TEST_ASSERT_EQUAL_INT32(255, canTableToWire(u8, 256));            // too large: stuck at maximum
  TEST_ASSERT_EQUAL_INT32(0,   canTableToWire(u8, -1));             // negative into unsigned: stuck at 0

  const canTableSignal_t s8 = sig(8, true, 1, 1, 0);
  TEST_ASSERT_EQUAL_INT32(127,  canTableToWire(s8, 127));
  TEST_ASSERT_EQUAL_INT32(127,  canTableToWire(s8, 150));           // the known limit: 150 C does not fit signed 8 bit
  TEST_ASSERT_EQUAL_INT32(-128, canTableToWire(s8, -128));
  TEST_ASSERT_EQUAL_INT32(-128, canTableToWire(s8, -129));

  const canTableSignal_t u4 = sig(4, false, 1, 1, 0);
  TEST_ASSERT_EQUAL_INT32(15, canTableToWire(u4, 99));

  // 32-bit signals and huge factors must not overflow
  const canTableSignal_t u32 = sig(32, false, 32767, 1, 0);
  TEST_ASSERT_EQUAL_INT32(-1, canTableToWire(u32, 1000000));        // largest 32 bit value is 0xFFFFFFFF = -1 as int32
  const canTableSignal_t s32 = sig(32, true, 32767, 1, 0);
  TEST_ASSERT_EQUAL_INT32(INT32_MAX, canTableToWire(s32, 1000000));
  TEST_ASSERT_EQUAL_INT32(INT32_MIN, canTableToWire(s32, -1000000));
}

void testCanTableScaling(void)
{
  RUN_TEST(test_scaling_roundsToNearest_notDown);
  RUN_TEST(test_scaling_multiplierDivisorOffsetAndNegativeDivisor);
  RUN_TEST(test_scaling_clampsToTheBitLength);
}

// ------------------------------------------------------------------------------------------------
//  Packing: wire number <-> payload bits
// ------------------------------------------------------------------------------------------------

static void test_pack_knownByteLayouts(void)
{
  uint8_t p[8] = {0};
  canTableSignal_t s = sig(16, false, 1, 1, 0);
  s.startBit = 0;
  canTablePackSignal(s, 12000, p);                                  // 12000 = 0x2EE0, low byte first
  TEST_ASSERT_EQUAL_HEX8(0xE0, p[0]);
  TEST_ASSERT_EQUAL_HEX8(0x2E, p[1]);

  s.flags |= CAN_SIG_FLAG_BIGENDIAN;                                // high byte first
  canTablePackSignal(s, 12000, p);
  TEST_ASSERT_EQUAL_HEX8(0x2E, p[0]);
  TEST_ASSERT_EQUAL_HEX8(0xE0, p[1]);

  uint8_t q[8] = {0};
  canTableSignal_t nib = sig(4, false, 1, 1, 0);
  nib.startBit = 24;                                                // upper-left nibble of byte 3? no: bits 24-27 = LOW nibble of byte 3
  canTablePackSignal(nib, 2, q);
  TEST_ASSERT_EQUAL_HEX8(0x02, q[3]);
  nib.startBit = 28;                                                // high nibble of byte 3
  canTablePackSignal(nib, 0xA, q);
  TEST_ASSERT_EQUAL_HEX8(0xA2, q[3]);

  uint8_t r[8] = {0};
  canTableSignal_t neg = sig(8, true, 1, 1, 0);
  neg.startBit = 8;
  canTablePackSignal(neg, -10, r);                                  // -10 in two's complement = 0xF6
  TEST_ASSERT_EQUAL_HEX8(0xF6, r[1]);
  TEST_ASSERT_EQUAL_HEX8(0x00, r[0]);
  TEST_ASSERT_EQUAL_HEX8(0x00, r[2]);
}

static void test_pack_overwritesOnlyItsOwnBits(void)
{
  uint8_t p[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  canTableSignal_t s = sig(12, false, 1, 1, 0);
  s.startBit = 10;
  canTablePackSignal(s, 0, p);                                      // clear bits 10..21
  for (uint8_t bit = 0; bit < 64; bit++)
  {
    const bool isSet = ((p[bit / 8] >> (bit % 8)) & 1U) != 0U;
    const bool inside = (bit >= 10) && (bit < 22);
    TEST_ASSERT_EQUAL_MESSAGE(!inside, isSet, "bit outside the signal was changed, or a bit inside was not cleared");
  }
}

static void test_pack_unpack_roundTripEveryStartAndLength(void)
{
  // For every legal position and length, the extreme values and a middle value must survive a round trip,
  // for both signedness and (where legal) both byte orders.
  for (uint8_t length = 1; length <= 32; length++)
  {
    for (uint8_t start = 0; (start + length) <= 64; start++)
    {
      for (int isSigned = 0; isSigned < 2; isSigned++)
      {
        for (int big = 0; big < 2; big++)
        {
          if (big && (((start % 8) != 0) || ((length % 8) != 0))) { continue; }   // big endian only on byte borders
          canTableSignal_t s = sig(length, isSigned != 0, 1, 1, 0);
          s.startBit = start;
          if (big) { s.flags |= CAN_SIG_FLAG_BIGENDIAN; }

          const int64_t lo = isSigned ? -((int64_t)1 << (length - 1)) : 0;
          const int64_t hi = isSigned ? (((int64_t)1 << (length - 1)) - 1) : (((int64_t)1 << length) - 1);
          const int64_t values[3] = { lo, hi, (lo + hi) / 2 };
          for (int v = 0; v < 3; v++)
          {
            uint8_t p[8] = {0xA5, 0x5A, 0xA5, 0x5A, 0xA5, 0x5A, 0xA5, 0x5A};   // junk that has to be overwritten
            const uint8_t before[8] = {0xA5, 0x5A, 0xA5, 0x5A, 0xA5, 0x5A, 0xA5, 0x5A};
            const int32_t wire = (int32_t)(uint32_t)(uint64_t)values[v];       // 32-bit pattern of the value
            canTablePackSignal(s, wire, p);
            TEST_ASSERT_EQUAL_INT32(wire, canTableUnpackSignal(s, p));
            // bits outside the signal unchanged
            for (uint8_t bit = 0; bit < 64; bit++)
            {
              if ((bit >= start) && (bit < (start + length))) { continue; }
              TEST_ASSERT_EQUAL_UINT8((before[bit / 8] >> (bit % 8)) & 1U, (p[bit / 8] >> (bit % 8)) & 1U);
            }
          }
        }
      }
    }
  }
}

void testCanTablePacking(void)
{
  RUN_TEST(test_pack_knownByteLayouts);
  RUN_TEST(test_pack_overwritesOnlyItsOwnBits);
  RUN_TEST(test_pack_unpack_roundTripEveryStartAndLength);
}

// ------------------------------------------------------------------------------------------------
//  DDU9 example table
// ------------------------------------------------------------------------------------------------

static void test_ddu9_exampleIsValidAndUsesCan3(void)
{
  canTableConfig_t cfg;
  canTableLoadDdu9Example(cfg);
  TEST_ASSERT_EQUAL(CAN_TABLE_OK, validate(cfg));
  TEST_ASSERT_EQUAL_UINT8(0, cfg.channel[0].enabled);               // CAN1 stays off
  TEST_ASSERT_EQUAL_UINT8(1, cfg.channel[1].enabled);
  TEST_ASSERT_EQUAL_UINT16(1000, cfg.channel[1].bitrateKbit);
  uint8_t enabledMessages = 0, enabledSignals = 0;
  for (uint8_t m = 0; m < CAN_TABLE_MAX_MESSAGES; m++) { if (cfg.message[m].flags & CAN_MSG_FLAG_ENABLED) { enabledMessages++; } }
  for (uint8_t s = 0; s < CAN_TABLE_MAX_SIGNALS; s++)  { if (cfg.signal[s].flags & CAN_SIG_FLAG_ENABLED) { enabledSignals++; } }
  TEST_ASSERT_EQUAL_UINT8(4, enabledMessages);
  TEST_ASSERT_EQUAL_UINT8(16, enabledSignals);
}

static void test_ddu9_payloadsMatchTheTestedProgram(void)
{
  canTableConfig_t cfg;
  canTableLoadDdu9Example(cfg);

  canTableSources_t src;
  memset(&src, 0, sizeof(src));
  src.value[CAN_SRC_RPM] = 12000;
  src.value[CAN_SRC_GEAR] = 6;
  src.value[CAN_SRC_MAP_KPA] = 200;
  src.value[CAN_SRC_TPS_PERCENT] = 100;
  src.value[CAN_SRC_LAMBDA_MILLI] = 1000;
  src.value[CAN_SRC_OIL_PRESSURE_KPA] = 250;
  src.value[CAN_SRC_SPEED_KPH] = 200;
  src.value[CAN_SRC_USER1] = -180;                                  // steering
  src.value[CAN_SRC_USER2] = 3;                                     // map position
  src.value[CAN_SRC_FUEL_TEMP_C] = 80;
  src.value[CAN_SRC_IAT_C] = -10;
  src.value[CAN_SRC_CLT_C] = 120;
  src.value[CAN_SRC_USER3] = 150;                                   // oil temperature
  src.value[CAN_SRC_FUEL_PRESSURE_KPA] = 300;
  src.value[CAN_SRC_USER4] = 1250;                                  // exhaust temperature

  uint8_t p[8];
  canTableBuildPayload(cfg, 0, src, p);                             // 0x011
  const uint8_t expected11[8] = { 0xE0, 0x2E, 0x06, 0xD0, 0x07, 0xE8, 0x03, 0x00 };   // rpm, gear, MAP 2000, throttle 1000
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected11, p, 8);

  canTableBuildPayload(cfg, 1, src, p);                             // 0x033
  const uint8_t expected33[8] = { 200, 0x00, 25, 0xE8, 0x03, 0x4C, 0xFF, 0x00 };      // speed 200, oil 25, lambda 1000, steer -180 = 0xFF4C
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected33, p, 8);

  canTableBuildPayload(cfg, 2, src, p);                             // 0x055
  const uint8_t expected55[8] = { 75, 0xF6, 120, 3, 80, 0x00, 0x00, 0x00 };           // toil/2, tint -10, tmot, mappos, tfuel
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected55, p, 8);

  canTableBuildPayload(cfg, 3, src, p);                             // 0x066
  const uint8_t expected66[8] = { 0xE2, 0x04, 30, 0x02, 0x00, 0x00, 0x00, 0x00 };     // texh 1250 = 0x04E2, pfuel 30, ign 2
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected66, p, 8);
}

void testCanTableDdu9(void)
{
  RUN_TEST(test_ddu9_exampleIsValidAndUsesCan3);
  RUN_TEST(test_ddu9_payloadsMatchTheTestedProgram);
}

// ------------------------------------------------------------------------------------------------
//  Scheduler
// ------------------------------------------------------------------------------------------------

struct SentFrame { uint8_t channel; uint16_t id; uint8_t data[8]; };
static SentFrame g_frames[64];
static uint8_t   g_frameCount;
static bool      g_sendResult = true;

static bool fakeSend(uint8_t channel, uint16_t id, const uint8_t data[8])
{
  if (g_frameCount < 64)
  {
    g_frames[g_frameCount].channel = channel;
    g_frames[g_frameCount].id = id;
    memcpy(g_frames[g_frameCount].data, data, 8);
    g_frameCount++;
  }
  return g_sendResult;
}

static void resetFakeBus(void) { g_frameCount = 0; g_sendResult = true; }

static void test_scheduler_emptyTableSendsNothing(void)
{
  resetFakeBus();
  canTableConfig_t cfg; canTableClear(cfg);
  canTableState_t st; canTableResetState(st);
  canTableSources_t src; memset(&src, 0, sizeof(src));
  TEST_ASSERT_EQUAL_UINT8(0, canTableProcess(cfg, st, src, 0, fakeSend));
  TEST_ASSERT_EQUAL_UINT8(0, canTableProcess(cfg, st, src, 100000, fakeSend));
  TEST_ASSERT_EQUAL_UINT8(0, g_frameCount);
}

static void test_scheduler_sendsAtStartThenEveryInterval(void)
{
  resetFakeBus();
  canTableConfig_t cfg = baseTable();                               // 100 ms
  addSignal(cfg, 0, CAN_SRC_RPM, 0, 16, false);
  canTableState_t st; canTableResetState(st);
  canTableSources_t src; memset(&src, 0, sizeof(src));
  src.value[CAN_SRC_RPM] = 3000;

  TEST_ASSERT_EQUAL_UINT8(1, canTableProcess(cfg, st, src, 5000, fakeSend));    // first call: immediately, even at "time 5000"
  TEST_ASSERT_EQUAL_UINT8(0, canTableProcess(cfg, st, src, 5099, fakeSend));    // 99 ms later: not yet
  TEST_ASSERT_EQUAL_UINT8(1, canTableProcess(cfg, st, src, 5100, fakeSend));    // exactly 100 ms: send
  TEST_ASSERT_EQUAL_UINT8(0, canTableProcess(cfg, st, src, 5101, fakeSend));
  TEST_ASSERT_EQUAL_UINT8(1, canTableProcess(cfg, st, src, 5300, fakeSend));    // late call: still just one frame, no catch-up burst
  TEST_ASSERT_EQUAL_UINT8(3, g_frameCount);
  TEST_ASSERT_EQUAL_UINT16(0x123, g_frames[0].id);
  TEST_ASSERT_EQUAL_UINT8(0, g_frames[0].channel);
  TEST_ASSERT_EQUAL_HEX8(0xB8, g_frames[0].data[0]);                // 3000 = 0x0BB8
  TEST_ASSERT_EQUAL_HEX8(0x0B, g_frames[0].data[1]);
}

static void test_scheduler_survivesMillisecondCounterWrap(void)
{
  resetFakeBus();
  canTableConfig_t cfg = baseTable();
  canTableState_t st; canTableResetState(st);
  canTableSources_t src; memset(&src, 0, sizeof(src));
  TEST_ASSERT_EQUAL_UINT8(1, canTableProcess(cfg, st, src, 0xFFFFFFC0UL, fakeSend));
  TEST_ASSERT_EQUAL_UINT8(0, canTableProcess(cfg, st, src, 0xFFFFFFFFUL, fakeSend));   // 63 ms
  TEST_ASSERT_EQUAL_UINT8(0, canTableProcess(cfg, st, src, 0x00000010UL, fakeSend));   // 80 ms (counter wrapped)
  TEST_ASSERT_EQUAL_UINT8(1, canTableProcess(cfg, st, src, 0x00000024UL, fakeSend));   // 100 ms
}

static void test_scheduler_skipsDisabledChannelDisabledMessageAndInputMessage(void)
{
  resetFakeBus();
  canTableConfig_t cfg = baseTable();
  cfg.message[1].id = 0x200; cfg.message[1].intervalMs = 10; cfg.message[1].flags = CAN_MSG_FLAG_ENABLED | CAN_MSG_FLAG_INPUT;   // input
  cfg.message[2].id = 0x201; cfg.message[2].intervalMs = 10; cfg.message[2].flags = 0;                                            // not enabled
  cfg.message[3].id = 0x202; cfg.message[3].intervalMs = 10; cfg.message[3].flags = CAN_MSG_FLAG_ENABLED | CAN_MSG_FLAG_CHANNEL1; // channel 1 is off
  canTableState_t st; canTableResetState(st);
  canTableSources_t src; memset(&src, 0, sizeof(src));
  TEST_ASSERT_EQUAL_UINT8(1, canTableProcess(cfg, st, src, 1000, fakeSend));
  TEST_ASSERT_EQUAL_UINT16(0x123, g_frames[0].id);

  cfg.channel[1].enabled = 1; cfg.channel[1].bitrateKbit = 1000;      // switching the channel on makes message 3 appear
  resetFakeBus();
  TEST_ASSERT_EQUAL_UINT8(1, canTableProcess(cfg, st, src, 1010, fakeSend));
  TEST_ASSERT_EQUAL_UINT16(0x202, g_frames[0].id);
  TEST_ASSERT_EQUAL_UINT8(1, g_frames[0].channel);

  cfg.channel[0].enabled = 0;                                         // channel 0 off: message 0 stops
  resetFakeBus();
  canTableProcess(cfg, st, src, 5000, fakeSend);
  for (uint8_t i = 0; i < g_frameCount; i++) { TEST_ASSERT_NOT_EQUAL_UINT16(0x123, g_frames[i].id); }
}

static void test_scheduler_countsFailuresButKeepsTheSchedule(void)
{
  resetFakeBus();
  g_sendResult = false;
  canTableConfig_t cfg = baseTable();
  canTableState_t st; canTableResetState(st);
  canTableSources_t src; memset(&src, 0, sizeof(src));
  TEST_ASSERT_EQUAL_UINT8(0, canTableProcess(cfg, st, src, 0, fakeSend));
  TEST_ASSERT_EQUAL_UINT32(1, st.sendFailures);
  TEST_ASSERT_EQUAL_UINT8(0, canTableProcess(cfg, st, src, 50, fakeSend));      // not retried before the interval is over
  TEST_ASSERT_EQUAL_UINT8(1, g_frameCount);
  TEST_ASSERT_EQUAL_UINT8(0, canTableProcess(cfg, st, src, 100, fakeSend));
  TEST_ASSERT_EQUAL_UINT32(2, st.sendFailures);
  g_sendResult = true;
  TEST_ASSERT_EQUAL_UINT8(1, canTableProcess(cfg, st, src, 200, fakeSend));
}

static void test_scheduler_nullSendFunctionDoesNothing(void)
{
  canTableConfig_t cfg = baseTable();
  canTableState_t st; canTableResetState(st);
  canTableSources_t src; memset(&src, 0, sizeof(src));
  TEST_ASSERT_EQUAL_UINT8(0, canTableProcess(cfg, st, src, 0, nullptr));
}

static void test_scheduler_ddu9ExampleSendsFourFramesAtTheRightRates(void)
{
  resetFakeBus();
  canTableConfig_t cfg; canTableLoadDdu9Example(cfg);
  canTableState_t st; canTableResetState(st);
  canTableSources_t src; memset(&src, 0, sizeof(src));

  uint16_t count[4] = {0, 0, 0, 0};
  for (uint32_t t = 0; t <= 2000; t++)                              // 2 seconds in 1 ms steps
  {
    g_frameCount = 0;
    canTableProcess(cfg, st, src, t, fakeSend);
    for (uint8_t i = 0; i < g_frameCount; i++)
    {
      TEST_ASSERT_EQUAL_UINT8(1, g_frames[i].channel);              // everything goes out on CAN3
      if (g_frames[i].id == 0x011) { count[0]++; }
      if (g_frames[i].id == 0x033) { count[1]++; }
      if (g_frames[i].id == 0x055) { count[2]++; }
      if (g_frames[i].id == 0x066) { count[3]++; }
    }
  }
  TEST_ASSERT_EQUAL_UINT16(41, count[0]);                           // t = 0, 50, ..., 2000
  TEST_ASSERT_EQUAL_UINT16(21, count[1]);                           // t = 0, 100, ..., 2000
  TEST_ASSERT_EQUAL_UINT16(3, count[2]);                            // t = 0, 1000, 2000
  TEST_ASSERT_EQUAL_UINT16(3, count[3]);
}

void testCanTableScheduler(void)
{
  RUN_TEST(test_scheduler_emptyTableSendsNothing);
  RUN_TEST(test_scheduler_sendsAtStartThenEveryInterval);
  RUN_TEST(test_scheduler_survivesMillisecondCounterWrap);
  RUN_TEST(test_scheduler_skipsDisabledChannelDisabledMessageAndInputMessage);
  RUN_TEST(test_scheduler_countsFailuresButKeepsTheSchedule);
  RUN_TEST(test_scheduler_nullSendFunctionDoesNothing);
  RUN_TEST(test_scheduler_ddu9ExampleSendsFourFramesAtTheRightRates);
}
