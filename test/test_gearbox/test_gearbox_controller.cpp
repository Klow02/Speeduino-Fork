#include "../test_utils.h"
#include "src/controllers/gearbox/gearboxController.h"
#include "src/pins/inputPin.h"

extern inputPin_t paddleUpPin;
extern inputPin_t paddleDownPin;
extern inputPin_t neutralPin;
extern gearbox_inputs_t buildInputs(const statuses &current, const config2 &page2, uint32_t nowMs, bool upRequest, bool downRequest);

struct controller_fixture
{
    statuses current = {};
    config2 page2 = {};
    pinNumbers_t pins = {};

    controller_fixture()
    {
        pins.pinPaddleUp = 30;
        pins.pinPaddleDown = 31;
        pins.pinNeutral = 32;
        page2.vssMode = VSS_MODE_INTERNAL_PIN;
        const uint16_t ratios[6] = { 20U, 36U, 52U, 68U, 84U, 100U };
        for (uint8_t i = 0U; i < 6U; ++i) { page2.vssRatios[i] = ratios[i]; }
        gearboxConfig() = gearbox_config_t();
        gearboxIoConfig() = gearbox_io_config_t();
    }

    void init(void) { initialiseGearbox(current, pins); }
    gearbox_inputs_t inputs(void) { return buildInputs(current, page2, 1234U, false, false); }
};

static void test_init_disabledStaysDisabled(void)
{
    controller_fixture f;
    gearboxConfig().enabled = false;
    f.init();
    TEST_ASSERT_FALSE(gearboxConfig().enabled);
}

static void test_init_enabledWithValidPaddlePins(void)
{
    controller_fixture f;
    gearboxConfig().enabled = true;
    f.init();
    TEST_ASSERT_TRUE(gearboxConfig().enabled);
}

static void test_init_missingUpPaddleDisablesTheModule(void)
{
    controller_fixture f;
    gearboxConfig().enabled = true;
    f.pins.pinPaddleUp = NOT_A_PIN;
    f.init();
    TEST_ASSERT_FALSE(gearboxConfig().enabled);
}

static void test_init_missingDownPaddleDisablesTheModule(void)
{
    controller_fixture f;
    gearboxConfig().enabled = true;
    f.pins.pinPaddleDown = NOT_A_PIN;
    f.init();
    TEST_ASSERT_FALSE(gearboxConfig().enabled);
}

static void test_init_neutralPinIsOptional(void)
{
    controller_fixture f;
    gearboxConfig().enabled = true;
    f.pins.pinNeutral = NOT_A_PIN;
    f.init();
    TEST_ASSERT_TRUE(gearboxConfig().enabled);
}

static void test_init_resetsPublishedStatus(void)
{
    controller_fixture f;
    f.current.gearbox.shiftCount = 7U;
    f.current.gearbox.cutIgnition = true;
    f.init();
    TEST_ASSERT_EQUAL_UINT16(0, f.current.gearbox.shiftCount);
    TEST_ASSERT_FALSE(f.current.gearbox.cutIgnition);
}

static void test_inputs_gearTakenFromStatus(void)
{
    controller_fixture f;
    f.init();
    neutralPin._pin.setPinHigh(); // not in neutral (active low)
    f.current.gear = 3U;
    f.current.setRpm(6000U);

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
    neutralPin._pin.setPinHigh();
    f.current.gear = 0U;

    TEST_ASSERT_FALSE(f.inputs().gearValid);
}

static void test_inputs_gearUnknownWhenVssIsOff(void)
{
    controller_fixture f;
    f.init();
    neutralPin._pin.setPinHigh();
    f.page2.vssMode = VSS_MODE_OFF;
    f.current.gear = 3U;

    TEST_ASSERT_FALSE(f.inputs().gearValid);
}

static void test_inputs_neutralSwitchWins(void)
{
    controller_fixture f;
    f.init();
    f.current.gear = 3U;
    neutralPin._pin.setPinLow(); // active low: in neutral

    const auto in = f.inputs();

    TEST_ASSERT_TRUE(in.gearValid);
    TEST_ASSERT_EQUAL_UINT8(0, in.gear);
}

static void test_inputs_neutralSwitchWinsEvenWithoutVss(void)
{
    controller_fixture f;
    f.init();
    f.page2.vssMode = VSS_MODE_OFF;
    neutralPin._pin.setPinLow();

    const auto in = f.inputs();

    TEST_ASSERT_TRUE(in.gearValid);
    TEST_ASSERT_EQUAL_UINT8(0, in.gear);
}

static void test_inputs_neutralActiveHigh(void)
{
    controller_fixture f;
    gearboxIoConfig().neutralActiveLow = false;
    f.init();
    f.current.gear = 3U;

    neutralPin._pin.setPinLow();
    TEST_ASSERT_EQUAL_UINT8(3, f.inputs().gear);

    neutralPin._pin.setPinHigh();
    TEST_ASSERT_EQUAL_UINT8(0, f.inputs().gear);
}

static void test_inputs_noPositionFeedbackYet(void)
{
    controller_fixture f;
    f.init();
    TEST_ASSERT_FALSE(f.inputs().positionValid);
}

void testGearboxController(void)
{
    SET_UNITY_FILENAME() {
        RUN_TEST_P(test_init_disabledStaysDisabled);
        RUN_TEST_P(test_init_enabledWithValidPaddlePins);
        RUN_TEST_P(test_init_missingUpPaddleDisablesTheModule);
        RUN_TEST_P(test_init_missingDownPaddleDisablesTheModule);
        RUN_TEST_P(test_init_neutralPinIsOptional);
        RUN_TEST_P(test_init_resetsPublishedStatus);
        RUN_TEST_P(test_inputs_gearTakenFromStatus);
        RUN_TEST_P(test_inputs_gearZeroIsUnknownNotNeutral);
        RUN_TEST_P(test_inputs_gearUnknownWhenVssIsOff);
        RUN_TEST_P(test_inputs_neutralSwitchWins);
        RUN_TEST_P(test_inputs_neutralSwitchWinsEvenWithoutVss);
        RUN_TEST_P(test_inputs_neutralActiveHigh);
        RUN_TEST_P(test_inputs_noPositionFeedbackYet);
    }
}
