/**
 * Tests of the gearbox settings page (gearboxSettings.*): the 64-byte TunerStudio page is turned
 * into the controller configuration, clamped, and re-applied only when sensible. Pins and clock are
 * faked, so everything runs on a PC.
 */
#include <unity.h>
#include <string.h>
#include "src/controllers/gearbox/gearboxSettings.h"

// ---- fake hardware: counts how often pins are set up -----------------------------------------
static int setupCalls;
static bool fakeBlocked[64];
static void fSetup(uint8_t, bool) { setupCalls++; }
static bool fRead(uint8_t) { return true; }
static uint32_t fNow(void) { return 0U; }
static bool fAvail(uint8_t pin) { return pin < 64 && !fakeBlocked[pin]; }
static const gearboxHal_t hal = { fSetup, fRead, fNow, fAvail };

/** A complete, valid page as TunerStudio would deliver it. */
static gearboxSettingsPage_t validPage(void)
{
  gearboxSettingsPage_t p;
  memset(&p, 0, sizeof(p));
  p.enabled = 1; p.cutOnUpshift = 1; p.cutIgnition = 1; p.paddlesActiveLow = 1; p.neutralActiveLow = 1;
  p.maxGear = 5; p.pinPaddleUp = 30; p.pinPaddleDown = 31; p.pinNeutral = 255;
  p.cutMinRpm = 3000; p.maxCutMs = 150; p.upTravel = 800; p.downTravel = 700; p.positionTolerance = 50;
  p.debounceMs = 20; p.cutLeadMs = 15; p.moveMs = 80; p.holdMs = 30; p.returnMs = 80;
  p.minIntervalMs = 250; p.maxShiftMs = 600; p.faultLockoutMs = 1000;
  return p;
}

static void resetFakes(void) { setupCalls = 0; memset(fakeBlocked, 0, sizeof(fakeBlocked)); }

static void test_layout_matchesTheIniOffsets(void)
{
  gearboxSettingsPage_t p; memset(&p, 0, sizeof(p));
  uint8_t* raw = reinterpret_cast<uint8_t*>(&p);
  p.enabled = 1;           TEST_ASSERT_EQUAL_UINT8(0x01, raw[0]); p.enabled = 0;
  p.cutFuel = 1;           TEST_ASSERT_EQUAL_UINT8(0x20, raw[0]); p.cutFuel = 0;
  p.useFeedback = 1;       TEST_ASSERT_EQUAL_UINT8(0x40, raw[0]); p.useFeedback = 0;
  p.neutralActiveLow = 1;  TEST_ASSERT_EQUAL_UINT8(0x02, raw[1]); p.neutralActiveLow = 0;
  p.cutMinRpm = 0x1234;    TEST_ASSERT_EQUAL_UINT8(0x34, raw[6]); TEST_ASSERT_EQUAL_UINT8(0x12, raw[7]);
  p.faultLockoutMs = 0x0102; TEST_ASSERT_EQUAL_UINT8(0x02, raw[34]); TEST_ASSERT_EQUAL_UINT8(0x01, raw[35]);
}

static void test_zeroPageMeansDisabled(void)
{
  gearboxSettingsPage_t p; memset(&p, 0, sizeof(p));
  gearboxController_t c; c.config.enabled = true;
  gearboxApplySettings(p, c);
  TEST_ASSERT_FALSE(c.config.enabled);
}

static void test_apply_copiesEverything(void)
{
  gearboxSettingsPage_t p = validPage();
  p.allowDownToNeutral = 1; p.cutOnDownshift = 1; p.cutFuel = 1; p.useFeedback = 1; p.downshiftMaxRpm = 9000; p.verifyMs = 300;
  gearboxController_t c;
  gearboxApplySettings(p, c);
  TEST_ASSERT_TRUE(c.config.enabled);
  TEST_ASSERT_TRUE(c.config.allowDownToNeutral);
  TEST_ASSERT_TRUE(c.config.cutOnUpshift);
  TEST_ASSERT_TRUE(c.config.cutOnDownshift);
  TEST_ASSERT_TRUE(c.config.cutIgnition);
  TEST_ASSERT_TRUE(c.config.cutFuel);
  TEST_ASSERT_TRUE(c.config.useFeedback);
  TEST_ASSERT_EQUAL_UINT8(5, c.config.maxGear);
  TEST_ASSERT_EQUAL_UINT16(3000, c.config.cutMinRpm);
  TEST_ASSERT_EQUAL_UINT16(150, c.config.maxCutMs);
  TEST_ASSERT_EQUAL_UINT16(9000, c.config.downshiftMaxRpm);
  TEST_ASSERT_EQUAL_UINT16(800, c.config.upTravel);
  TEST_ASSERT_EQUAL_UINT16(700, c.config.downTravel);
  TEST_ASSERT_EQUAL_UINT16(50, c.config.positionTolerance);
  TEST_ASSERT_EQUAL_UINT16(20, c.config.debounceMs);
  TEST_ASSERT_EQUAL_UINT16(15, c.config.cutLeadMs);
  TEST_ASSERT_EQUAL_UINT16(80, c.config.moveMs);
  TEST_ASSERT_EQUAL_UINT16(30, c.config.holdMs);
  TEST_ASSERT_EQUAL_UINT16(80, c.config.returnMs);
  TEST_ASSERT_EQUAL_UINT16(300, c.config.verifyMs);
  TEST_ASSERT_EQUAL_UINT16(250, c.config.minIntervalMs);
  TEST_ASSERT_EQUAL_UINT16(600, c.config.maxShiftMs);
  TEST_ASSERT_EQUAL_UINT16(1000, c.config.faultLockoutMs);
  TEST_ASSERT_TRUE(c.io.paddlesActiveLow);
  TEST_ASSERT_TRUE(c.io.neutralActiveLow);
  TEST_ASSERT_EQUAL_UINT8(30, c.pinPaddleUp);
  TEST_ASSERT_EQUAL_UINT8(31, c.pinPaddleDown);
  TEST_ASSERT_EQUAL_UINT8(255, c.pinNeutral);
}

static void test_apply_polarityBitsAreIndependent(void)
{
  gearboxSettingsPage_t p = validPage();
  p.paddlesActiveLow = 0; p.neutralActiveLow = 1;
  gearboxController_t c; gearboxApplySettings(p, c);
  TEST_ASSERT_FALSE(c.io.paddlesActiveLow);
  TEST_ASSERT_TRUE(c.io.neutralActiveLow);
  p.paddlesActiveLow = 1; p.neutralActiveLow = 0;
  gearboxApplySettings(p, c);
  TEST_ASSERT_TRUE(c.io.paddlesActiveLow);
  TEST_ASSERT_FALSE(c.io.neutralActiveLow);
}

static void test_apply_clampsMaxGear(void)
{
  gearboxSettingsPage_t p = validPage(); gearboxController_t c;
  p.maxGear = 0;   gearboxApplySettings(p, c); TEST_ASSERT_EQUAL_UINT8(1, c.config.maxGear);
  p.maxGear = 6;   gearboxApplySettings(p, c); TEST_ASSERT_EQUAL_UINT8(6, c.config.maxGear);
  p.maxGear = 7;   gearboxApplySettings(p, c); TEST_ASSERT_EQUAL_UINT8(6, c.config.maxGear);
  p.maxGear = 255; gearboxApplySettings(p, c); TEST_ASSERT_EQUAL_UINT8(6, c.config.maxGear);
}

static void test_apply_clampsTravelsAndTolerance(void)
{
  gearboxSettingsPage_t p = validPage(); gearboxController_t c;
  p.upTravel = 1000; p.downTravel = 1001; p.positionTolerance = 65535;
  gearboxApplySettings(p, c);
  TEST_ASSERT_EQUAL_UINT16(1000, c.config.upTravel);
  TEST_ASSERT_EQUAL_UINT16(1000, c.config.downTravel);
  TEST_ASSERT_EQUAL_UINT16(1000, c.config.positionTolerance);
}

static void test_refresh_firstCallAppliesAndInitialisesPins(void)
{
  resetFakes();
  gearboxSettingsPage_t page = validPage(), cached; memset(&cached, 0, sizeof(cached)); bool valid = false;
  gearboxController_t c;
  TEST_ASSERT_TRUE(gearboxRefreshSettings(page, cached, valid, c, hal));
  TEST_ASSERT_TRUE(valid);
  TEST_ASSERT_TRUE(c.config.enabled);
  TEST_ASSERT_EQUAL_INT(2, setupCalls);   // two paddles, neutral not fitted
}

static void test_refresh_unchangedDoesNothing(void)
{
  resetFakes();
  gearboxSettingsPage_t page = validPage(), cached; memset(&cached, 0, sizeof(cached)); bool valid = false;
  gearboxController_t c;
  gearboxRefreshSettings(page, cached, valid, c, hal);
  setupCalls = 0;
  TEST_ASSERT_FALSE(gearboxRefreshSettings(page, cached, valid, c, hal));
  TEST_ASSERT_EQUAL_INT(0, setupCalls);
}

static void test_refresh_changeIsAppliedAndPinsReinitialised(void)
{
  resetFakes();
  gearboxSettingsPage_t page = validPage(), cached; memset(&cached, 0, sizeof(cached)); bool valid = false;
  gearboxController_t c;
  gearboxRefreshSettings(page, cached, valid, c, hal);
  setupCalls = 0;
  page.pinPaddleUp = 40; page.moveMs = 99;
  TEST_ASSERT_TRUE(gearboxRefreshSettings(page, cached, valid, c, hal));
  TEST_ASSERT_EQUAL_UINT8(40, c.pinPaddleUp);
  TEST_ASSERT_EQUAL_UINT16(99, c.config.moveMs);
  TEST_ASSERT_EQUAL_INT(2, setupCalls);
}

static void test_refresh_lastByteChangeIsDetected(void)
{
  resetFakes();
  gearboxSettingsPage_t page = validPage(), cached; memset(&cached, 0, sizeof(cached)); bool valid = false;
  gearboxController_t c;
  gearboxRefreshSettings(page, cached, valid, c, hal);
  page.reserved[27] = 1; // even a change in the unused tail is a change (the page is compared whole)
  TEST_ASSERT_TRUE(gearboxRefreshSettings(page, cached, valid, c, hal));
}

static void test_refresh_notWhileAShiftIsRunning(void)
{
  resetFakes();
  gearboxSettingsPage_t page = validPage(), cached; memset(&cached, 0, sizeof(cached)); bool valid = false;
  gearboxController_t c;
  gearboxRefreshSettings(page, cached, valid, c, hal);
  c.state.phase = ShiftPhase::Move;
  page.moveMs = 123;
  setupCalls = 0;
  TEST_ASSERT_FALSE(gearboxRefreshSettings(page, cached, valid, c, hal));
  TEST_ASSERT_EQUAL_UINT16(80, c.config.moveMs);       // old value still in use
  TEST_ASSERT_EQUAL_INT(0, setupCalls);
  c.state.phase = ShiftPhase::Idle;                      // shift over: now it is applied
  TEST_ASSERT_TRUE(gearboxRefreshSettings(page, cached, valid, c, hal));
  TEST_ASSERT_EQUAL_UINT16(123, c.config.moveMs);
}

static void test_refresh_disablingSwitchesTheModuleOffWithoutTouchingPins(void)
{
  resetFakes();
  gearboxSettingsPage_t page = validPage(), cached; memset(&cached, 0, sizeof(cached)); bool valid = false;
  gearboxController_t c;
  gearboxRefreshSettings(page, cached, valid, c, hal);
  setupCalls = 0;
  page.enabled = 0;
  TEST_ASSERT_TRUE(gearboxRefreshSettings(page, cached, valid, c, hal));
  TEST_ASSERT_FALSE(c.config.enabled);
  TEST_ASSERT_EQUAL_INT(0, setupCalls);
}

static void test_refresh_unavailablePaddlePinKeepsModuleOff(void)
{
  resetFakes();
  fakeBlocked[30] = true;
  gearboxSettingsPage_t page = validPage(), cached; memset(&cached, 0, sizeof(cached)); bool valid = false;
  gearboxController_t c;
  gearboxRefreshSettings(page, cached, valid, c, hal);
  TEST_ASSERT_FALSE(c.config.enabled);
  TEST_ASSERT_EQUAL_INT(0, setupCalls);
}

void testGearboxSettings(void)
{
  RUN_TEST(test_layout_matchesTheIniOffsets);
  RUN_TEST(test_zeroPageMeansDisabled);
  RUN_TEST(test_apply_copiesEverything);
  RUN_TEST(test_apply_polarityBitsAreIndependent);
  RUN_TEST(test_apply_clampsMaxGear);
  RUN_TEST(test_apply_clampsTravelsAndTolerance);
  RUN_TEST(test_refresh_firstCallAppliesAndInitialisesPins);
  RUN_TEST(test_refresh_unchangedDoesNothing);
  RUN_TEST(test_refresh_changeIsAppliedAndPinsReinitialised);
  RUN_TEST(test_refresh_lastByteChangeIsDetected);
  RUN_TEST(test_refresh_notWhileAShiftIsRunning);
  RUN_TEST(test_refresh_disablingSwitchesTheModuleOffWithoutTouchingPins);
  RUN_TEST(test_refresh_unavailablePaddlePinKeepsModuleOff);
}
