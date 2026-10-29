/**
 * Tests of the gearbox controller glue (gearboxController.*): input handling, start-up checks,
 * paddle reading and the shift cut mask. The pins and the clock are replaced by a fake that a
 * test can set, so everything runs on a PC.
 */
#include <unity.h>
#include <string.h>
#include "src/controllers/gearbox/gearboxController.h"

// ------------------------------------------------------------------------------------------------
//  Fake hardware
// ------------------------------------------------------------------------------------------------
struct FakeGearboxHw
{
  bool     level[64];        // pin level (true = HIGH)
  bool     inputSetup[64];   // setupInput() was called for the pin
  bool     pullup[64];       // ... with pull-up
  uint32_t now;
};
static FakeGearboxHw fake;

static void fakeSetup(uint8_t pin, bool pullup) { fake.inputSetup[pin] = true; fake.pullup[pin] = pullup; }
static bool fakeRead(uint8_t pin) { return fake.level[pin]; }
static uint32_t fakeNow(void) { return fake.now; }
static const gearboxHal_t hal = { fakeSetup, fakeRead, fakeNow };

static int actuatorCalls;
static bool lastEnable;
static int16_t lastPermille;
static void fakeActuator(bool enable, int16_t permille) { actuatorCalls++; lastEnable = enable; lastPermille = permille; }

struct controller_fixture
{
  gearboxController_t c;
  gearbox_live_t live;

  controller_fixture()
  {
    memset(&fake, 0, sizeof(fake));
    for (int i = 0; i < 64; ++i) { fake.level[i] = true; } // all switches released (active low, pulled up)
    fake.now = 1000U;
    actuatorCalls = 0;
    c.pinPaddleUp = 30; c.pinPaddleDown = 31; c.pinNeutral = 32;
    live.vssEnabled = true;
    const uint16_t ratios[6] = { 20U, 36U, 52U, 68U, 84U, 100U };
    for (uint8_t i = 0U; i < 6U; ++i) { live.ratio[i] = ratios[i]; }
  }
  void init(void) { gearboxInit(c, hal); }
  gearbox_inputs_t inputs(void) { return gearboxBuildInputs(c, hal, live, 1234U, false, false); }
};

// ------------------------------------------------------------------------------------------------
//  Start-up
// ------------------------------------------------------------------------------------------------
static void test_init_disabledStaysDisabled(void)
{
  controller_fixture f;
  f.c.config.enabled = false;
  f.init();
  TEST_ASSERT_FALSE(f.c.config.enabled);
}

static void test_init_enabledWithValidPaddlePins(void)
{
  controller_fixture f;
  f.c.config.enabled = true;
  f.init();
  TEST_ASSERT_TRUE(f.c.config.enabled);
  TEST_ASSERT_TRUE(fake.inputSetup[30] && fake.inputSetup[31] && fake.inputSetup[32]);
  TEST_ASSERT_TRUE(fake.pullup[30] && fake.pullup[31]);   // active-low paddles use the internal pull-up
}

static void test_init_missingUpPaddleDisablesTheModule(void)
{
  controller_fixture f;
  f.c.config.enabled = true;
  f.c.pinPaddleUp = GEARBOX_NO_PIN;
  f.init();
  TEST_ASSERT_FALSE(f.c.config.enabled);
}

static void test_init_missingDownPaddleDisablesTheModule(void)
{
  controller_fixture f;
  f.c.config.enabled = true;
  f.c.pinPaddleDown = GEARBOX_NO_PIN;
  f.init();
  TEST_ASSERT_FALSE(f.c.config.enabled);
}

static void test_init_neutralPinIsOptional(void)
{
  controller_fixture f;
  f.c.config.enabled = true;
  f.c.pinNeutral = GEARBOX_NO_PIN;
  f.init();
  TEST_ASSERT_TRUE(f.c.config.enabled);
  TEST_ASSERT_FALSE(fake.inputSetup[32]);
}

static void test_init_activeHighPaddlesUseNoPullup(void)
{
  controller_fixture f;
  f.c.io.paddlesActiveLow = false;
  f.init();
  TEST_ASSERT_FALSE(fake.pullup[30]);
}

static void test_init_resetsPublishedStatus(void)
{
  controller_fixture f;
  f.c.status.shiftCount = 7U;
  f.c.status.cutIgnition = true;
  f.init();
  TEST_ASSERT_EQUAL_UINT16(0, f.c.status.shiftCount);
  TEST_ASSERT_FALSE(f.c.status.cutIgnition);
}

// ------------------------------------------------------------------------------------------------
//  Inputs
// ------------------------------------------------------------------------------------------------
static void test_inputs_gearTakenFromLiveValues(void)
{
  controller_fixture f;
  f.init();
  f.live.gear = 3U;
  f.live.rpm = 6000U;
  const auto in = f.inputs();
  TEST_ASSERT_TRUE(in.gearValid);
  TEST_ASSERT_EQUAL_UINT8(3, in.gear);
  TEST_ASSERT_EQUAL_UINT16(6000, in.rpm);
  TEST_ASSERT_EQUAL_UINT16(52, in.ratio[2]);
}

static void test_inputs_gearZeroIsUnknownNotNeutral(void)
{
  controller_fixture f;
  f.init();
  f.live.gear = 0U;
  TEST_ASSERT_FALSE(f.inputs().gearValid);
}

static void test_inputs_gearUnknownWhenVssIsOff(void)
{
  controller_fixture f;
  f.init();
  f.live.vssEnabled = false;
  f.live.gear = 3U;
  TEST_ASSERT_FALSE(f.inputs().gearValid);
}

static void test_inputs_neutralSwitchWins(void)
{
  controller_fixture f;
  f.init();
  f.live.gear = 3U;
  fake.level[32] = false; // active low: pulled to ground = in neutral
  const auto in = f.inputs();
  TEST_ASSERT_TRUE(in.gearValid);
  TEST_ASSERT_EQUAL_UINT8(0, in.gear);
}

static void test_inputs_neutralSwitchWinsEvenWithoutVss(void)
{
  controller_fixture f;
  f.init();
  f.live.vssEnabled = false;
  fake.level[32] = false;
  const auto in = f.inputs();
  TEST_ASSERT_TRUE(in.gearValid);
  TEST_ASSERT_EQUAL_UINT8(0, in.gear);
}

static void test_inputs_neutralActiveHigh(void)
{
  controller_fixture f;
  f.c.io.neutralActiveLow = false;
  f.init();
  f.live.gear = 3U;
  fake.level[32] = false;
  TEST_ASSERT_EQUAL_UINT8(3, f.inputs().gear);
  fake.level[32] = true;
  TEST_ASSERT_EQUAL_UINT8(0, f.inputs().gear);
}

static void test_inputs_unconnectedNeutralIsNeverNeutral(void)
{
  controller_fixture f;
  f.c.pinNeutral = GEARBOX_NO_PIN;
  f.init();
  f.live.gear = 4U;
  TEST_ASSERT_EQUAL_UINT8(4, f.inputs().gear);
}

static void test_inputs_noPositionFeedbackYet(void)
{
  controller_fixture f;
  f.init();
  TEST_ASSERT_FALSE(f.inputs().positionValid);
}

// ------------------------------------------------------------------------------------------------
//  Paddles through gearboxUpdate()
// ------------------------------------------------------------------------------------------------
static void test_update_paddlePressStartsAShiftAndDrivesTheActuator(void)
{
  controller_fixture f;
  f.c.config.enabled = true;
  f.c.actuator = fakeActuator;
  f.init();
  f.live.gear = 2U;
  f.live.rpm = 8000U;

  gearboxUpdate(f.c, f.live, hal);                 // primes the debouncers: nothing pressed
  TEST_ASSERT_EQUAL_INT(ShiftPhase::Idle, (int)f.c.status.phase);

  fake.level[30] = false;                          // up paddle pressed (active low)
  const uint32_t pressedAt = fake.now;
  bool started = false;
  for (uint32_t t = 0; t < 400 && !started; ++t)   // wait out the debounce time
  {
    fake.now = pressedAt + t;
    gearboxUpdate(f.c, f.live, hal);
    started = (f.c.status.phase != ShiftPhase::Idle);
  }
  TEST_ASSERT_TRUE(started);
  TEST_ASSERT_TRUE(f.c.status.direction == ShiftDirection::Up);
  TEST_ASSERT_TRUE(actuatorCalls > 0);
}

static void test_update_paddleHeldAtStartupDoesNotShift(void)
{
  controller_fixture f;
  f.c.config.enabled = true;
  f.init();
  f.live.gear = 2U;
  f.live.rpm = 8000U;
  fake.level[30] = false;                          // held down from the very beginning
  for (uint32_t t = 0; t < 500; ++t)
  {
    fake.now = 1000U + t;
    gearboxUpdate(f.c, f.live, hal);
  }
  TEST_ASSERT_TRUE(f.c.status.phase == ShiftPhase::Idle);
  TEST_ASSERT_EQUAL_UINT16(0, f.c.status.shiftCount);
}

static void test_update_noActuatorIsFine(void)
{
  controller_fixture f;
  f.c.config.enabled = true;
  f.c.actuator = nullptr;
  f.init();
  gearboxUpdate(f.c, f.live, hal);
  TEST_ASSERT_EQUAL_INT(0, actuatorCalls);
}

// ------------------------------------------------------------------------------------------------
//  Shift cut mask
// ------------------------------------------------------------------------------------------------
static void test_cut_noCutLeavesTheMasksAlone(void)
{
  gearboxStatus_t s;
  uint8_t ign = 0xFF, fuel = 0xFF, pend = 0x05;
  gearboxApplyCut(s, ign, fuel, pend);
  TEST_ASSERT_EQUAL_HEX8(0xFF, ign);
  TEST_ASSERT_EQUAL_HEX8(0xFF, fuel);
  TEST_ASSERT_EQUAL_HEX8(0x05, pend);
}

static void test_cut_ignitionCutClearsIgnitionAndPending(void)
{
  gearboxStatus_t s; s.cutIgnition = true;
  uint8_t ign = 0xFF, fuel = 0xFF, pend = 0x05;
  gearboxApplyCut(s, ign, fuel, pend);
  TEST_ASSERT_EQUAL_HEX8(0x00, ign);
  TEST_ASSERT_EQUAL_HEX8(0x00, pend);
  TEST_ASSERT_EQUAL_HEX8(0xFF, fuel);
}

static void test_cut_fuelCutClearsFuelOnly(void)
{
  gearboxStatus_t s; s.cutFuel = true;
  uint8_t ign = 0xFF, fuel = 0xFF, pend = 0x05;
  gearboxApplyCut(s, ign, fuel, pend);
  TEST_ASSERT_EQUAL_HEX8(0xFF, ign);
  TEST_ASSERT_EQUAL_HEX8(0x00, fuel);
  TEST_ASSERT_EQUAL_HEX8(0x05, pend);
}

static void test_cut_neverSwitchesAnythingBackOn(void)
{
  // The rev limiter already cut some channels. A shift cut (or no cut) must not bring them back.
  for (int mode = 0; mode < 4; ++mode)
  {
    gearboxStatus_t s; s.cutIgnition = (mode & 1); s.cutFuel = (mode & 2);
    uint8_t ign = 0x0A, fuel = 0x05, pend = 0x01;
    gearboxApplyCut(s, ign, fuel, pend);
    TEST_ASSERT_TRUE((ign & ~0x0A) == 0);
    TEST_ASSERT_TRUE((fuel & ~0x05) == 0);
    TEST_ASSERT_TRUE((pend & ~0x01) == 0);
  }
}

void testGearboxController(void)
{
  RUN_TEST(test_init_disabledStaysDisabled);
  RUN_TEST(test_init_enabledWithValidPaddlePins);
  RUN_TEST(test_init_missingUpPaddleDisablesTheModule);
  RUN_TEST(test_init_missingDownPaddleDisablesTheModule);
  RUN_TEST(test_init_neutralPinIsOptional);
  RUN_TEST(test_init_activeHighPaddlesUseNoPullup);
  RUN_TEST(test_init_resetsPublishedStatus);
  RUN_TEST(test_inputs_gearTakenFromLiveValues);
  RUN_TEST(test_inputs_gearZeroIsUnknownNotNeutral);
  RUN_TEST(test_inputs_gearUnknownWhenVssIsOff);
  RUN_TEST(test_inputs_neutralSwitchWins);
  RUN_TEST(test_inputs_neutralSwitchWinsEvenWithoutVss);
  RUN_TEST(test_inputs_neutralActiveHigh);
  RUN_TEST(test_inputs_unconnectedNeutralIsNeverNeutral);
  RUN_TEST(test_inputs_noPositionFeedbackYet);
  RUN_TEST(test_update_paddlePressStartsAShiftAndDrivesTheActuator);
  RUN_TEST(test_update_paddleHeldAtStartupDoesNotShift);
  RUN_TEST(test_update_noActuatorIsFine);
  RUN_TEST(test_cut_noCutLeavesTheMasksAlone);
  RUN_TEST(test_cut_ignitionCutClearsIgnitionAndPending);
  RUN_TEST(test_cut_fuelCutClearsFuelOnly);
  RUN_TEST(test_cut_neverSwitchesAnythingBackOn);
}
