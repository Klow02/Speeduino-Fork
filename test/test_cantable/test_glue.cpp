/**
 * Tests of the CAN table glue (src/controllers/canTable/canTableGlue.*): unit conversion of the
 * source values, change detection, channel/pin handling and sending. The CAN hardware is replaced
 * by a fake that records every call, so everything runs on a PC.
 */
#include <unity.h>
#include <string.h>
#include "src/controllers/canTable/canTableGlue.h"

// ------------------------------------------------------------------------------------------------
//  Fake hardware: remembers what the controller asked for
// ------------------------------------------------------------------------------------------------
struct FakeHw
{
  uint8_t  beginCount[CAN_TABLE_CHANNELS];
  uint16_t beginBitrate[CAN_TABLE_CHANNELS];
  bool     beginResult;
  uint8_t  frameCount;
  uint8_t  frameChannel[16];
  uint16_t frameId[16];
  uint8_t  frameData[16][8];
  bool     pinFree;            // answer of pinIsFree() for every pin
  int8_t   pinState[64];       // -1 = never driven, 0 = low, 1 = high
};
static FakeHw hw;

static bool fakeBegin(uint8_t ch, uint16_t kbit) { hw.beginCount[ch]++; hw.beginBitrate[ch] = kbit; return hw.beginResult; }
static bool fakeSend(uint8_t ch, uint16_t id, const uint8_t d[8])
{
  if (hw.frameCount < 16) { hw.frameChannel[hw.frameCount] = ch; hw.frameId[hw.frameCount] = id; memcpy(hw.frameData[hw.frameCount], d, 8); }
  hw.frameCount++;
  return true;
}
static bool fakePinFree(uint8_t) { return hw.pinFree; }
static void fakeSetPin(uint8_t pin, bool on) { hw.pinState[pin] = on ? 1 : 0; }
static const canTableHal_t fakeHal = { fakeBegin, fakeSend, fakePinFree, fakeSetPin };

static void resetHw(void)
{
  memset(&hw, 0, sizeof(hw));
  hw.beginResult = true;
  hw.pinFree = true;
  memset(hw.pinState, -1, sizeof(hw.pinState));
}

/** A table with one message 0x123 every 100 ms on channel 0 (CAN1, 500 kbit/s) carrying the RPM in 16 bits. */
static canTableConfig_t rpmTable(void)
{
  canTableConfig_t cfg;
  canTableClear(cfg);
  cfg.channel[0].enabled = 1;
  cfg.channel[0].bitrateKbit = 500;
  cfg.message[0].id = 0x123;
  cfg.message[0].intervalMs = 100;
  cfg.message[0].flags = CAN_MSG_FLAG_ENABLED;
  cfg.signal[0].message = 0;
  cfg.signal[0].source = CAN_SRC_RPM;
  cfg.signal[0].startBit = 0;
  cfg.signal[0].lengthBits = 16;
  cfg.signal[0].flags = CAN_SIG_FLAG_ENABLED;
  cfg.signal[0].multiplier = 1;
  cfg.signal[0].divisor = 1;
  return cfg;
}

// ------------------------------------------------------------------------------------------------
//  Source conversion
// ------------------------------------------------------------------------------------------------
/** All-zero live values (no statuses struct needed any more). */
static canTableLive_t zeroLive(void)
{
  canTableLive_t l;
  memset(&l, 0, sizeof(l));
  l.stoichX10 = 147;
  return l;
}

static void test_glue_sources_conversion(void)
{
  canTableLive_t st = zeroLive();
  st.rpm = 4321; st.mapKpa = 101; st.tpsPercent = 55; st.cltC = 88; st.iatC = -5; st.battery100mV = 138;
  st.speedKph = 120; st.gear = 3; st.oilPressurePsi = 50; st.fuelPressurePsi = 43; st.fuelTempC = 21; st.afrX10 = 147;
  canTableSources_t s;
  memset(&s, 0, sizeof(s));
  s.value[CAN_SRC_USER3] = 777; // must survive

  canTableFillSources(st, s);

  TEST_ASSERT_EQUAL_INT32(4321, s.value[CAN_SRC_RPM]);
  TEST_ASSERT_EQUAL_INT32(101, s.value[CAN_SRC_MAP_KPA]);
  TEST_ASSERT_EQUAL_INT32(55, s.value[CAN_SRC_TPS_PERCENT]);
  TEST_ASSERT_EQUAL_INT32(88, s.value[CAN_SRC_CLT_C]);
  TEST_ASSERT_EQUAL_INT32(-5, s.value[CAN_SRC_IAT_C]);   // negative temperatures stay negative
  TEST_ASSERT_EQUAL_INT32(138, s.value[CAN_SRC_BATTERY_100MV]);
  TEST_ASSERT_EQUAL_INT32(120, s.value[CAN_SRC_SPEED_KPH]);
  TEST_ASSERT_EQUAL_INT32(3, s.value[CAN_SRC_GEAR]);
  TEST_ASSERT_EQUAL_INT32(345, s.value[CAN_SRC_OIL_PRESSURE_KPA]);  // 50 psi * 6.895 = 344.75 -> 345
  TEST_ASSERT_EQUAL_INT32(296, s.value[CAN_SRC_FUEL_PRESSURE_KPA]); // 43 psi * 6.895 = 296.5 -> 296 (296.485)
  TEST_ASSERT_EQUAL_INT32(21, s.value[CAN_SRC_FUEL_TEMP_C]);
  TEST_ASSERT_EQUAL_INT32(1000, s.value[CAN_SRC_LAMBDA_MILLI]);     // AFR 14.7 at stoich 14.7 = lambda 1.000
  TEST_ASSERT_EQUAL_INT32(777, s.value[CAN_SRC_USER3]);
}

static void test_glue_sources_lambda_edges(void)
{
  canTableLive_t st = zeroLive();
  canTableSources_t s;
  memset(&s, 0, sizeof(s));
  st.afrX10 = 120; // AFR 12.0
  canTableFillSources(st, s);
  TEST_ASSERT_EQUAL_INT32(816, s.value[CAN_SRC_LAMBDA_MILLI]); // 120000/147 = 816
  st.stoichX10 = 0;                                             // stoich unknown: no division by zero
  canTableFillSources(st, s);
  TEST_ASSERT_EQUAL_INT32(0, s.value[CAN_SRC_LAMBDA_MILLI]);
  st.stoichX10 = 147;
  st.afrX10 = 255; // largest value fits without overflow
  canTableFillSources(st, s);
  TEST_ASSERT_EQUAL_INT32(1734, s.value[CAN_SRC_LAMBDA_MILLI]);
}

// ------------------------------------------------------------------------------------------------
//  Controller
// ------------------------------------------------------------------------------------------------
static void test_glue_empty_table_sends_nothing_and_starts_nothing(void)
{
  resetHw();
  canTableController_t c; canTableControllerReset(c);
  canTableConfig_t cfg; canTableClear(cfg);
  canTableLive_t st = zeroLive();
  for (uint32_t t = 0; t < 2000; t += 10) { canTableControllerUpdate(c, cfg, st, fakeHal, t); }
  TEST_ASSERT_EQUAL_UINT8(0, hw.frameCount);
  TEST_ASSERT_EQUAL_UINT8(0, hw.beginCount[0]);
  TEST_ASSERT_EQUAL_UINT8(0, hw.beginCount[1]);
}

static void test_glue_sends_rpm_frame_at_interval(void)
{
  resetHw();
  canTableController_t c; canTableControllerReset(c);
  canTableConfig_t cfg = rpmTable();
  canTableLive_t st = zeroLive(); st.rpm = 0x1234;
  for (uint32_t t = 0; t < 1000; ++t) { canTableControllerUpdate(c, cfg, st, fakeHal, t); }
  TEST_ASSERT_EQUAL_UINT8(1, hw.beginCount[0]);
  TEST_ASSERT_EQUAL_UINT16(500, hw.beginBitrate[0]);
  TEST_ASSERT_EQUAL_UINT8(10, hw.frameCount);              // 0, 100, ... 900 ms
  TEST_ASSERT_EQUAL_UINT16(0x123, hw.frameId[0]);
  TEST_ASSERT_EQUAL_UINT8(0, hw.frameChannel[0]);
  TEST_ASSERT_EQUAL_HEX8(0x34, hw.frameData[0][0]);        // little endian: low byte first
  TEST_ASSERT_EQUAL_HEX8(0x12, hw.frameData[0][1]);
}

static void test_glue_uses_current_values_each_time(void)
{
  resetHw();
  canTableController_t c; canTableControllerReset(c);
  canTableConfig_t cfg = rpmTable();
  canTableLive_t st = zeroLive(); st.rpm = 1000;
  canTableControllerUpdate(c, cfg, st, fakeHal, 0);
  st.rpm = 2000;
  canTableControllerUpdate(c, cfg, st, fakeHal, 100);
  TEST_ASSERT_EQUAL_UINT8(2, hw.frameCount);
  TEST_ASSERT_EQUAL_HEX8(0xE8, hw.frameData[0][0]); TEST_ASSERT_EQUAL_HEX8(0x03, hw.frameData[0][1]); // 1000
  TEST_ASSERT_EQUAL_HEX8(0xD0, hw.frameData[1][0]); TEST_ASSERT_EQUAL_HEX8(0x07, hw.frameData[1][1]); // 2000
}

static void test_glue_invalid_table_is_silent(void)
{
  resetHw();
  canTableController_t c; canTableControllerReset(c);
  canTableConfig_t cfg = rpmTable();
  cfg.message[0].id = 0x800;                 // above 0x7FF: invalid
  canTableLive_t st = zeroLive();
  for (uint32_t t = 0; t < 500; ++t) { canTableControllerUpdate(c, cfg, st, fakeHal, t); }
  TEST_ASSERT_EQUAL_UINT8(0, hw.frameCount);
  TEST_ASSERT_EQUAL_UINT8(0, hw.beginCount[0]);   // not even started
  TEST_ASSERT_EQUAL(CAN_TABLE_ERR_ID, c.lastError);
  TEST_ASSERT_FALSE(c.sending);
}

static void test_glue_change_is_picked_up_and_fixing_restarts_sending(void)
{
  resetHw();
  canTableController_t c; canTableControllerReset(c);
  canTableConfig_t cfg = rpmTable();
  canTableLive_t st = zeroLive(); st.rpm = 1;
  canTableControllerUpdate(c, cfg, st, fakeHal, 0);
  TEST_ASSERT_EQUAL_UINT8(1, hw.frameCount);

  cfg.message[0].id = 0x800;                                  // user makes a mistake...
  // The change is noticed within CAN_TABLE_CHANGE_CHECK_MS (250 ms); until then the old table may still send.
  for (uint32_t t = 1; t <= 300; ++t) { canTableControllerUpdate(c, cfg, st, fakeHal, t); }
  const uint8_t framesWhenNoticed = hw.frameCount;
  for (uint32_t t = 301; t <= 700; ++t) { canTableControllerUpdate(c, cfg, st, fakeHal, t); }
  TEST_ASSERT_EQUAL_UINT8(framesWhenNoticed, hw.frameCount);  // ...and then it stays silent
  TEST_ASSERT_FALSE(c.sending);

  cfg.message[0].id = 0x124;                                  // ...and fixes it
  for (uint32_t t = 701; t <= 1100; ++t) { canTableControllerUpdate(c, cfg, st, fakeHal, t); }
  TEST_ASSERT_TRUE(hw.frameCount > framesWhenNoticed);
  TEST_ASSERT_EQUAL_UINT16(0x124, hw.frameId[hw.frameCount - 1]);
  TEST_ASSERT_EQUAL_UINT8(2, hw.beginCount[0]);               // channel was re-started after the change
}

static void test_glue_unchanged_table_does_not_restart_channels(void)
{
  resetHw();
  canTableController_t c; canTableControllerReset(c);
  canTableConfig_t cfg = rpmTable();
  canTableLive_t st = zeroLive();
  for (uint32_t t = 0; t < 5000; ++t) { canTableControllerUpdate(c, cfg, st, fakeHal, t); }
  TEST_ASSERT_EQUAL_UINT8(1, hw.beginCount[0]);
}

static void test_glue_disabled_channel_is_not_started_and_not_used(void)
{
  resetHw();
  canTableController_t c; canTableControllerReset(c);
  canTableConfig_t cfg = rpmTable();
  cfg.channel[0].enabled = 0;
  canTableLive_t st = zeroLive();
  for (uint32_t t = 0; t < 500; ++t) { canTableControllerUpdate(c, cfg, st, fakeHal, t); }
  TEST_ASSERT_EQUAL_UINT8(0, hw.beginCount[0]);
  TEST_ASSERT_EQUAL_UINT8(0, hw.frameCount);
}

static void test_glue_second_channel_is_used_for_channel1_messages(void)
{
  resetHw();
  canTableController_t c; canTableControllerReset(c);
  canTableConfig_t cfg = rpmTable();
  cfg.channel[0].enabled = 0;
  cfg.channel[1].enabled = 1; cfg.channel[1].bitrateKbit = 1000;
  cfg.message[0].flags |= CAN_MSG_FLAG_CHANNEL1;
  canTableLive_t st = zeroLive();
  canTableControllerUpdate(c, cfg, st, fakeHal, 0);
  TEST_ASSERT_EQUAL_UINT8(1, hw.beginCount[1]);
  TEST_ASSERT_EQUAL_UINT16(1000, hw.beginBitrate[1]);
  TEST_ASSERT_EQUAL_UINT8(1, hw.frameCount);
  TEST_ASSERT_EQUAL_UINT8(1, hw.frameChannel[0]);
}

static void test_glue_termination_pin_on_only_when_channel_on_and_pin_free(void)
{
  resetHw();
  canTableController_t c; canTableControllerReset(c);
  canTableConfig_t cfg = rpmTable();
  cfg.channel[0].terminationPin = 40;
  canTableLive_t st = zeroLive();
  canTableControllerUpdate(c, cfg, st, fakeHal, 0);
  TEST_ASSERT_EQUAL_INT8(1, hw.pinState[40]);                 // termination switched in

  // Channel switched off: pin goes low again.
  cfg.channel[0].enabled = 0;
  for (uint32_t t = 1; t <= 300; ++t) { canTableControllerUpdate(c, cfg, st, fakeHal, t); }
  TEST_ASSERT_EQUAL_INT8(0, hw.pinState[40]);
}

static void test_glue_termination_pin_used_by_speeduino_is_never_driven(void)
{
  resetHw();
  hw.pinFree = false;                                         // e.g. it is an injector or coil output
  canTableController_t c; canTableControllerReset(c);
  canTableConfig_t cfg = rpmTable();
  cfg.channel[0].terminationPin = 31;
  canTableLive_t st = zeroLive();
  canTableControllerUpdate(c, cfg, st, fakeHal, 0);
  TEST_ASSERT_EQUAL_INT8(-1, hw.pinState[31]);                // untouched, neither high nor low
  TEST_ASSERT_EQUAL_UINT8(1, hw.frameCount);                  // the channel still works
}

static void test_glue_no_termination_pin_means_no_pin_access(void)
{
  resetHw();
  canTableController_t c; canTableControllerReset(c);
  canTableConfig_t cfg = rpmTable();                          // pin = 0xFF ("none") from canTableClear
  canTableLive_t st = zeroLive();
  canTableControllerUpdate(c, cfg, st, fakeHal, 0);
  for (int i = 0; i < 64; ++i) { TEST_ASSERT_EQUAL_INT8(-1, hw.pinState[i]); }
}

static void test_glue_channel_that_fails_to_start_sends_nothing_on_it(void)
{
  resetHw();
  hw.beginResult = false;
  canTableController_t c; canTableControllerReset(c);
  canTableConfig_t cfg = rpmTable();
  cfg.channel[0].terminationPin = 40;
  canTableLive_t st = zeroLive();
  canTableControllerUpdate(c, cfg, st, fakeHal, 0);
  TEST_ASSERT_EQUAL_INT8(-1, hw.pinState[40]);                // no termination for a dead channel
  TEST_ASSERT_FALSE(c.channelUp[0]);
}

static void test_glue_survives_millisecond_counter_wraparound(void)
{
  resetHw();
  canTableController_t c; canTableControllerReset(c);
  canTableConfig_t cfg = rpmTable();
  canTableLive_t st = zeroLive();
  uint32_t t = 0xFFFFFF00UL;                                  // 256 ms before the counter wraps
  for (uint32_t i = 0; i < 600; ++i, ++t) { canTableControllerUpdate(c, cfg, st, fakeHal, t); }
  TEST_ASSERT_EQUAL_UINT8(6, hw.frameCount);                  // 600 ms at 100 ms = 6 frames, wrap or not
  TEST_ASSERT_EQUAL_UINT8(1, hw.beginCount[0]);               // and no false "table changed"
}

static void test_glue_change_made_after_wraparound_is_noticed(void)
{
  resetHw();
  canTableController_t c; canTableControllerReset(c);
  canTableConfig_t cfg = rpmTable();
  canTableLive_t st = zeroLive();
  uint32_t t = 0xFFFFFF80UL;
  for (uint32_t i = 0; i < 300; ++i, ++t) { canTableControllerUpdate(c, cfg, st, fakeHal, t); } // now past the wrap
  cfg.message[0].id = 0x800;                                  // break the table after the wrap
  for (uint32_t i = 0; i < 300; ++i, ++t) { canTableControllerUpdate(c, cfg, st, fakeHal, t); }
  TEST_ASSERT_FALSE(c.sending);
  TEST_ASSERT_EQUAL(CAN_TABLE_ERR_ID, c.lastError);
}

void testCanTableGlue(void)
{
  RUN_TEST(test_glue_change_made_after_wraparound_is_noticed);
  RUN_TEST(test_glue_sources_conversion);
  RUN_TEST(test_glue_sources_lambda_edges);
  RUN_TEST(test_glue_empty_table_sends_nothing_and_starts_nothing);
  RUN_TEST(test_glue_sends_rpm_frame_at_interval);
  RUN_TEST(test_glue_uses_current_values_each_time);
  RUN_TEST(test_glue_invalid_table_is_silent);
  RUN_TEST(test_glue_change_is_picked_up_and_fixing_restarts_sending);
  RUN_TEST(test_glue_unchanged_table_does_not_restart_channels);
  RUN_TEST(test_glue_disabled_channel_is_not_started_and_not_used);
  RUN_TEST(test_glue_second_channel_is_used_for_channel1_messages);
  RUN_TEST(test_glue_termination_pin_on_only_when_channel_on_and_pin_free);
  RUN_TEST(test_glue_termination_pin_used_by_speeduino_is_never_driven);
  RUN_TEST(test_glue_no_termination_pin_means_no_pin_access);
  RUN_TEST(test_glue_channel_that_fails_to_start_sends_nothing_on_it);
  RUN_TEST(test_glue_survives_millisecond_counter_wraparound);
}
