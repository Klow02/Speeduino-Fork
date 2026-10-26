#include "../test_utils.h"
#include "gearbox_fixture.h"

#define ASSERT_PHASE(expected, f) TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(expected), static_cast<uint8_t>((f).state.phase))
#define ASSERT_REJECT(expected, f) TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(expected), static_cast<uint8_t>((f).state.lastReject))
#define ASSERT_FAULT(expected, f) TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(expected), static_cast<uint8_t>((f).state.lastFault))

// ---------------------------------------------------------------------------
// Disabled
// ---------------------------------------------------------------------------

static void test_disabled_ignoresRequestsAndOutputsNothing(void)
{
    gearbox_fixture f;
    f.cfg.enabled = false;

    f.pressUp();

    ASSERT_PHASE(ShiftPhase::Idle, f);
    ASSERT_REJECT(ShiftReject::Disabled, f);
    TEST_ASSERT_FALSE(f.out.cutIgnition);
    TEST_ASSERT_FALSE(f.out.cutFuel);
    TEST_ASSERT_FALSE(f.out.actuatorEnable);
    TEST_ASSERT_EQUAL_INT16(0, f.out.actuatorPermille);
}

static void test_disablingMidShift_dropsEverythingButKeepsShiftCount(void)
{
    gearbox_fixture f;
    f.pressUp();
    f.runUntil(ShiftPhase::Lockout);
    TEST_ASSERT_EQUAL_UINT16(1, f.state.shiftCount);
    f.runUntil(ShiftPhase::Idle);
    f.pressUp();
    f.runUntil(ShiftPhase::Move);
    TEST_ASSERT_TRUE(f.out.cutIgnition);

    f.cfg.enabled = false;
    f.step();

    ASSERT_PHASE(ShiftPhase::Idle, f);
    TEST_ASSERT_FALSE(f.out.cutIgnition);
    TEST_ASSERT_FALSE(f.out.actuatorEnable);
    TEST_ASSERT_EQUAL_INT16(0, f.out.actuatorPermille);
    TEST_ASSERT_EQUAL_UINT16(1, f.state.shiftCount);
}

// ---------------------------------------------------------------------------
// Upshift
// ---------------------------------------------------------------------------

static void test_upshift_followsTheWholeSequenceWithTheExpectedTiming(void)
{
    gearbox_fixture f;

    f.pressUp();
    ASSERT_PHASE(ShiftPhase::PreCut, f);
    TEST_ASSERT_TRUE(f.out.cutIgnition);
    TEST_ASSERT_FALSE(f.out.cutFuel);
    TEST_ASSERT_TRUE(f.out.actuatorEnable);
    TEST_ASSERT_EQUAL_INT16(0, f.out.actuatorPermille);

    TEST_ASSERT_EQUAL_UINT32(f.cfg.cutLeadMs, f.runUntil(ShiftPhase::Move));
    TEST_ASSERT_TRUE(f.out.cutIgnition);
    TEST_ASSERT_EQUAL_INT16(f.cfg.upTravel, f.out.actuatorPermille);

    TEST_ASSERT_EQUAL_UINT32(f.cfg.cutLeadMs + f.cfg.moveMs, f.runUntil(ShiftPhase::Hold));
    TEST_ASSERT_TRUE(f.out.cutIgnition);
    TEST_ASSERT_EQUAL_INT16(f.cfg.upTravel, f.out.actuatorPermille);

    TEST_ASSERT_EQUAL_UINT32(f.cfg.cutLeadMs + f.cfg.moveMs + f.cfg.holdMs, f.runUntil(ShiftPhase::Return));
    TEST_ASSERT_FALSE(f.out.cutIgnition);
    TEST_ASSERT_EQUAL_INT16(0, f.out.actuatorPermille);

    TEST_ASSERT_EQUAL_UINT32(f.cfg.cutLeadMs + f.cfg.moveMs + f.cfg.holdMs + f.cfg.returnMs, f.runUntil(ShiftPhase::Lockout));
    TEST_ASSERT_EQUAL_UINT16(1, f.state.shiftCount);
    TEST_ASSERT_TRUE(f.out.actuatorEnable);

    TEST_ASSERT_EQUAL_UINT32(f.cfg.minIntervalMs, f.runUntil(ShiftPhase::Idle));
}

static void test_upshift_noCutBelowMinRpm(void)
{
    gearbox_fixture f;
    f.in.rpm = f.cfg.cutMinRpm - 1U;

    f.pressUp();

    // Straight to the move, cut never asserted
    ASSERT_PHASE(ShiftPhase::Move, f);
    TEST_ASSERT_FALSE(f.out.cutIgnition);
    TEST_ASSERT_EQUAL_INT16(f.cfg.upTravel, f.out.actuatorPermille);
    f.runUntil(ShiftPhase::Lockout);
    TEST_ASSERT_FALSE(f.out.cutIgnition);
}

static void test_upshift_cutAtExactlyMinRpm(void)
{
    gearbox_fixture f;
    f.in.rpm = f.cfg.cutMinRpm;

    f.pressUp();

    ASSERT_PHASE(ShiftPhase::PreCut, f);
    TEST_ASSERT_TRUE(f.out.cutIgnition);
}

static void test_upshift_cutDisabledInConfig(void)
{
    gearbox_fixture f;
    f.cfg.cutOnUpshift = false;

    f.pressUp();

    ASSERT_PHASE(ShiftPhase::Move, f);
    TEST_ASSERT_FALSE(f.out.cutIgnition);
    TEST_ASSERT_FALSE(f.out.cutFuel);
}

static void test_upshift_fuelOnlyCut(void)
{
    gearbox_fixture f;
    f.cfg.cutIgnition = false;
    f.cfg.cutFuel = true;

    f.pressUp();

    TEST_ASSERT_FALSE(f.out.cutIgnition);
    TEST_ASSERT_TRUE(f.out.cutFuel);
}

static void test_upshift_bothCutsWhenBothEnabled(void)
{
    gearbox_fixture f;
    f.cfg.cutFuel = true;

    f.pressUp();

    TEST_ASSERT_TRUE(f.out.cutIgnition);
    TEST_ASSERT_TRUE(f.out.cutFuel);
}

static void test_upshift_noCutWhenNeitherIgnitionNorFuelSelected(void)
{
    gearbox_fixture f;
    f.cfg.cutIgnition = false;
    f.cfg.cutFuel = false;

    f.pressUp();

    ASSERT_PHASE(ShiftPhase::Move, f);
}

static void test_upshift_zeroLeadTimeSkipsPreCut(void)
{
    gearbox_fixture f;
    f.cfg.cutLeadMs = 0U;

    f.pressUp();

    ASSERT_PHASE(ShiftPhase::Move, f);
    TEST_ASSERT_TRUE(f.out.cutIgnition);
}

static void test_upshift_travelIsClampedToFullScale(void)
{
    gearbox_fixture f;
    f.cfg.upTravel = 5000U;
    f.cfg.cutOnUpshift = false;

    f.pressUp();

    TEST_ASSERT_EQUAL_INT16(GEARBOX_PERMILLE_FULL, f.out.actuatorPermille);
}

static void test_upshift_fromNeutralIsAccepted(void)
{
    gearbox_fixture f;
    f.in.gear = 0U;
    f.in.gearValid = true;

    f.pressUp();

    TEST_ASSERT_TRUE(f.state.phase != ShiftPhase::Idle);
    TEST_ASSERT_TRUE(f.state.expectGearKnown);
    TEST_ASSERT_EQUAL_UINT8(1, f.state.expectGear);
}

// ---------------------------------------------------------------------------
// Downshift
// ---------------------------------------------------------------------------

static void test_downshift_movesTheOtherWayAndDoesNotCutByDefault(void)
{
    gearbox_fixture f;
    f.in.gear = 3U;
    f.in.rpm = 5000U;

    f.pressDown();

    ASSERT_PHASE(ShiftPhase::Move, f);
    TEST_ASSERT_FALSE(f.out.cutIgnition);
    TEST_ASSERT_EQUAL_INT16(-static_cast<int16_t>(f.cfg.downTravel), f.out.actuatorPermille);

    TEST_ASSERT_EQUAL_UINT32(f.cfg.moveMs, f.runUntil(ShiftPhase::Hold));
    TEST_ASSERT_EQUAL_INT16(-static_cast<int16_t>(f.cfg.downTravel), f.out.actuatorPermille);
    TEST_ASSERT_EQUAL_UINT32(f.cfg.moveMs + f.cfg.holdMs, f.runUntil(ShiftPhase::Return));
    TEST_ASSERT_EQUAL_INT16(0, f.out.actuatorPermille);
    f.runUntil(ShiftPhase::Lockout);
    TEST_ASSERT_EQUAL_UINT16(1, f.state.shiftCount);
}

static void test_downshift_cutWhenEnabled(void)
{
    gearbox_fixture f;
    f.cfg.cutOnDownshift = true;
    f.in.gear = 3U;

    f.pressDown();

    ASSERT_PHASE(ShiftPhase::PreCut, f);
    TEST_ASSERT_TRUE(f.out.cutIgnition);
}

static void test_downshift_firstGearRefusedByDefault(void)
{
    gearbox_fixture f;
    f.in.gear = 1U;

    f.pressDown();

    ASSERT_PHASE(ShiftPhase::Idle, f);
    ASSERT_REJECT(ShiftReject::BottomGear, f);
    TEST_ASSERT_EQUAL_INT16(0, f.out.actuatorPermille);
}

static void test_downshift_firstGearToNeutralWhenAllowed(void)
{
    gearbox_fixture f;
    f.cfg.allowDownToNeutral = true;
    f.cfg.verifyMs = 100U;
    f.in.gear = 1U;
    f.in.rpm = 2000U;

    f.pressDown();

    ASSERT_PHASE(ShiftPhase::Move, f);
    TEST_ASSERT_EQUAL_UINT8(0, f.state.expectGear);
    TEST_ASSERT_TRUE(f.state.expectGearKnown);

    // Neutral is a valid result for the verification
    f.runUntil(ShiftPhase::Verify);
    f.in.gear = 0U;
    f.step();
    ASSERT_PHASE(ShiftPhase::Lockout, f);
}

static void test_downshift_fromNeutralRefused(void)
{
    gearbox_fixture f;
    f.cfg.allowDownToNeutral = true;
    f.in.gear = 0U;

    f.pressDown();

    ASSERT_PHASE(ShiftPhase::Idle, f);
    ASSERT_REJECT(ShiftReject::BottomGear, f);
}

// ---------------------------------------------------------------------------
// Interlocks
// ---------------------------------------------------------------------------

static void test_upshift_topGearRefused(void)
{
    gearbox_fixture f;
    f.in.gear = f.cfg.maxGear;

    f.pressUp();

    ASSERT_PHASE(ShiftPhase::Idle, f);
    ASSERT_REJECT(ShiftReject::TopGear, f);
    TEST_ASSERT_FALSE(f.out.cutIgnition);
}

static void test_upshift_belowTopGearAccepted(void)
{
    gearbox_fixture f;
    f.in.gear = f.cfg.maxGear - 1U;

    f.pressUp();

    TEST_ASSERT_TRUE(f.state.phase != ShiftPhase::Idle);
}

static void test_unknownGear_upshiftAndDownshiftAreAccepted(void)
{
    gearbox_fixture f;
    f.in.gearValid = false;

    f.pressUp();
    TEST_ASSERT_TRUE(f.state.phase != ShiftPhase::Idle);
    TEST_ASSERT_FALSE(f.state.expectGearKnown);
    f.runUntil(ShiftPhase::Idle);

    f.pressDown();
    TEST_ASSERT_TRUE(f.state.phase != ShiftPhase::Idle);
}

static void test_downshift_overRevPredictionBoundary(void)
{
    gearbox_fixture f;
    f.in.gear = 3U;
    // 7000rpm * 52 / 36 = 10111rpm (integer arithmetic)
    f.cfg.downshiftMaxRpm = 10111U;

    f.in.rpm = 7000U;
    f.pressDown();
    TEST_ASSERT_TRUE(f.state.phase != ShiftPhase::Idle);
    f.runUntil(ShiftPhase::Idle);

    f.in.rpm = 7001U; // 10112
    f.pressDown();
    ASSERT_PHASE(ShiftPhase::Idle, f);
    ASSERT_REJECT(ShiftReject::OverRev, f);
}

static void test_downshift_overRevCheckOffWhenLimitIsZero(void)
{
    gearbox_fixture f;
    f.in.gear = 3U;
    f.in.rpm = 20000U;
    f.cfg.downshiftMaxRpm = 0U;

    f.pressDown();

    TEST_ASSERT_TRUE(f.state.phase != ShiftPhase::Idle);
}

static void test_downshift_overRevUsesCurrentRpmWhenRatiosUnknown(void)
{
    gearbox_fixture f;
    f.in.gear = 3U;
    f.cfg.downshiftMaxRpm = 9000U;
    for (uint8_t i = 0U; i < GEARBOX_MAX_GEARS; ++i)
    {
        f.in.ratio[i] = 0U;
    }

    f.in.rpm = 9000U;
    f.pressDown();
    TEST_ASSERT_TRUE(f.state.phase != ShiftPhase::Idle);
    f.runUntil(ShiftPhase::Idle);

    f.in.rpm = 9001U;
    f.pressDown();
    ASSERT_REJECT(ShiftReject::OverRev, f);
}

static void test_downshift_overRevUsesCurrentRpmWhenGearUnknown(void)
{
    gearbox_fixture f;
    f.in.gearValid = false;
    f.cfg.downshiftMaxRpm = 9000U;
    f.in.rpm = 9001U;

    f.pressDown();

    ASSERT_PHASE(ShiftPhase::Idle, f);
    ASSERT_REJECT(ShiftReject::OverRev, f);
}

static void test_bothPaddlesTogetherDoNothing(void)
{
    gearbox_fixture f;

    f.press(true, true);

    ASSERT_PHASE(ShiftPhase::Idle, f);
    ASSERT_REJECT(ShiftReject::BothPaddles, f);
    TEST_ASSERT_FALSE(f.out.cutIgnition);
}

static void test_requestsAreRefusedNotQueuedWhileBusyOrLockedOut(void)
{
    gearbox_fixture f;
    f.pressUp();
    f.runUntil(ShiftPhase::Move);

    f.step();
    f.pressUp();
    ASSERT_REJECT(ShiftReject::Busy, f);
    ASSERT_PHASE(ShiftPhase::Move, f);

    f.runUntil(ShiftPhase::Lockout);
    f.pressUp();
    ASSERT_REJECT(ShiftReject::Lockout, f);

    // Nothing was queued: the sequence ends and sits idle
    f.runUntil(ShiftPhase::Idle);
    f.runFor(1000U);
    ASSERT_PHASE(ShiftPhase::Idle, f);
    TEST_ASSERT_EQUAL_UINT16(1, f.state.shiftCount);
}

// ---------------------------------------------------------------------------
// Failsafes
// ---------------------------------------------------------------------------

static void test_cutNeverExceedsMaxCutEvenIfTheSequenceIsSlower(void)
{
    gearbox_fixture f;
    f.cfg.holdMs = 400U;
    f.cfg.maxShiftMs = 1000U;
    f.cfg.maxCutMs = 100U;

    f.pressUp();
    uint32_t cutMs = 0U;
    for (uint32_t i = 0U; i < 300U; ++i)
    {
        f.step();
        if (f.out.cutIgnition) { cutMs = f.sincePress(); }
    }

    TEST_ASSERT_TRUE(cutMs < f.cfg.maxCutMs);
    // The shift itself carried on regardless
    ASSERT_PHASE(ShiftPhase::Hold, f);
}

static void test_zeroMaxCutNeverCuts(void)
{
    gearbox_fixture f;
    f.cfg.maxCutMs = 0U;

    f.pressUp();

    ASSERT_PHASE(ShiftPhase::Move, f);
    TEST_ASSERT_FALSE(f.out.cutIgnition);
}

static void test_watchdogFaultsAReallySlowSequenceAndReleasesEverything(void)
{
    gearbox_fixture f;
    f.cfg.moveMs = 2000U;
    f.cfg.maxShiftMs = 300U;
    f.cfg.maxCutMs = 1000U;

    f.pressUp();
    TEST_ASSERT_EQUAL_UINT32(f.cfg.maxShiftMs, f.runUntil(ShiftPhase::Fault));

    ASSERT_FAULT(ShiftFault::SequenceTimeout, f);
    TEST_ASSERT_FALSE(f.out.cutIgnition);
    TEST_ASSERT_FALSE(f.out.cutFuel);
    TEST_ASSERT_FALSE(f.out.actuatorEnable);
    TEST_ASSERT_EQUAL_INT16(0, f.out.actuatorPermille);
}

static void test_zeroWatchdogFaultsImmediately(void)
{
    // An unconfigured watchdog fails safe rather than disabling the check
    gearbox_fixture f;
    f.cfg.maxShiftMs = 0U;

    f.pressUp();
    f.step();

    ASSERT_PHASE(ShiftPhase::Fault, f);
}

static void test_faultRecoversAfterLockoutAndClearsOnNextShift(void)
{
    gearbox_fixture f;
    f.cfg.moveMs = 2000U;
    f.cfg.maxShiftMs = 100U;
    f.pressUp();
    f.runUntil(ShiftPhase::Fault);
    ASSERT_FAULT(ShiftFault::SequenceTimeout, f);

    f.step();
    f.pressUp();
    ASSERT_REJECT(ShiftReject::Fault, f);

    f.cfg.moveMs = 80U;
    f.cfg.maxShiftMs = 600U;
    f.runUntil(ShiftPhase::Idle);
    f.out = gearboxStep(f.state, f.cfg, f.in);
    TEST_ASSERT_TRUE(f.out.actuatorEnable);

    f.pressUp();
    ASSERT_FAULT(ShiftFault::None, f);
    TEST_ASSERT_TRUE(f.state.phase != ShiftPhase::Idle);
}

static void test_timingSurvivesTheMillisecondCounterWrapping(void)
{
    gearbox_fixture f;
    f.in.nowMs = 0xFFFFFFE0U;

    f.pressUp();

    TEST_ASSERT_EQUAL_UINT32(f.cfg.cutLeadMs, f.runUntil(ShiftPhase::Move));
    TEST_ASSERT_EQUAL_UINT32(f.cfg.cutLeadMs + f.cfg.moveMs, f.runUntil(ShiftPhase::Hold));
    f.runUntil(ShiftPhase::Lockout);
    TEST_ASSERT_EQUAL_UINT16(1, f.state.shiftCount);
    f.runUntil(ShiftPhase::Idle);
    ASSERT_PHASE(ShiftPhase::Idle, f);
}

// ---------------------------------------------------------------------------
// Position feedback
// ---------------------------------------------------------------------------

static void test_feedback_moveWaitsForThePositionNotTheClock(void)
{
    gearbox_fixture f;
    f.cfg.useFeedback = true;
    f.cfg.cutOnUpshift = false;
    f.cfg.moveMs = 200U;
    f.in.positionValid = true;
    f.in.positionPermille = 0;

    f.pressUp();
    f.runFor(150U);
    ASSERT_PHASE(ShiftPhase::Move, f);

    f.in.positionPermille = static_cast<int16_t>(f.cfg.upTravel - f.cfg.positionTolerance - 1U);
    f.step();
    ASSERT_PHASE(ShiftPhase::Move, f);

    f.in.positionPermille = static_cast<int16_t>(f.cfg.upTravel - f.cfg.positionTolerance);
    f.step();
    ASSERT_PHASE(ShiftPhase::Hold, f);
}

static void test_feedback_moveTimeoutFaultsAndDisablesTheActuator(void)
{
    gearbox_fixture f;
    f.cfg.useFeedback = true;
    f.cfg.moveMs = 100U;
    f.in.positionValid = true;
    f.in.positionPermille = 0;

    f.pressUp();
    f.runUntil(ShiftPhase::Fault);

    ASSERT_FAULT(ShiftFault::PositionTimeout, f);
    TEST_ASSERT_FALSE(f.out.cutIgnition);
    TEST_ASSERT_FALSE(f.out.actuatorEnable);
}

static void test_feedback_invalidPositionNeverCountsAsArrived(void)
{
    gearbox_fixture f;
    f.cfg.useFeedback = true;
    f.cfg.moveMs = 100U;
    f.in.positionValid = false;
    f.in.positionPermille = static_cast<int16_t>(f.cfg.upTravel); // would match, but isn't trusted

    f.pressUp();
    f.runUntil(ShiftPhase::Fault);

    ASSERT_FAULT(ShiftFault::PositionTimeout, f);
}

static void test_feedback_returnWaitsForCentreThenTimesOut(void)
{
    gearbox_fixture f;
    f.cfg.useFeedback = true;
    f.cfg.cutOnUpshift = false;
    f.in.positionValid = true;
    f.in.positionPermille = static_cast<int16_t>(f.cfg.upTravel);

    f.pressUp();
    f.runUntil(ShiftPhase::Return);
    f.runFor(f.cfg.returnMs - 1U);
    ASSERT_PHASE(ShiftPhase::Return, f);

    // Never gets back to centre
    f.runFor(2U);
    ASSERT_PHASE(ShiftPhase::Fault, f);
    ASSERT_FAULT(ShiftFault::PositionTimeout, f);

    // And the happy path
    gearbox_fixture g;
    g.cfg.useFeedback = true;
    g.cfg.cutOnUpshift = false;
    g.in.positionValid = true;
    g.in.positionPermille = static_cast<int16_t>(g.cfg.upTravel);
    g.pressUp();
    g.runUntil(ShiftPhase::Return);
    g.in.positionPermille = static_cast<int16_t>(g.cfg.positionTolerance);
    g.step();
    ASSERT_PHASE(ShiftPhase::Lockout, g);
}

static void test_feedback_downshiftTargetsNegativeTravel(void)
{
    gearbox_fixture f;
    f.cfg.useFeedback = true;
    f.in.gear = 3U;
    f.in.positionValid = true;
    f.in.positionPermille = static_cast<int16_t>(-static_cast<int16_t>(f.cfg.downTravel));

    f.pressDown();
    f.step();

    ASSERT_PHASE(ShiftPhase::Hold, f);
}

// ---------------------------------------------------------------------------
// Gear verification
// ---------------------------------------------------------------------------

static void test_verify_succeedsWhenTheExpectedGearAppears(void)
{
    gearbox_fixture f;
    f.cfg.verifyMs = 150U;
    f.pressUp();
    f.runUntil(ShiftPhase::Verify);

    f.runFor(50U);
    ASSERT_PHASE(ShiftPhase::Verify, f);
    f.in.gear = 3U;
    f.step();

    ASSERT_PHASE(ShiftPhase::Lockout, f);
    TEST_ASSERT_EQUAL_UINT16(1, f.state.shiftCount);
}

static void test_verify_faultsWhenTheGearNeverArrives(void)
{
    gearbox_fixture f;
    f.cfg.verifyMs = 150U;
    f.pressUp();
    f.runUntil(ShiftPhase::Verify);
    const uint32_t verifyStart = f.sincePress();

    f.runUntil(ShiftPhase::Fault);

    TEST_ASSERT_EQUAL_UINT32(verifyStart + f.cfg.verifyMs, f.sincePress());
    ASSERT_FAULT(ShiftFault::GearNotReached, f);
    TEST_ASSERT_EQUAL_UINT16(0, f.state.shiftCount);
}

static void test_verify_wrongGearDoesNotCount(void)
{
    gearbox_fixture f;
    f.cfg.verifyMs = 150U;
    f.pressUp();
    f.runUntil(ShiftPhase::Verify);
    f.in.gear = 4U;
    f.runFor(10U);

    ASSERT_PHASE(ShiftPhase::Verify, f);
}

static void test_verify_skippedWhenTheStartingGearWasUnknown(void)
{
    gearbox_fixture f;
    f.cfg.verifyMs = 150U;
    f.in.gearValid = false;
    f.pressUp();
    f.runUntil(ShiftPhase::Return);
    f.runFor(f.cfg.returnMs);

    ASSERT_PHASE(ShiftPhase::Lockout, f);
}

static void test_verify_skippedWhenVerifyTimeIsZero(void)
{
    gearbox_fixture f;
    f.cfg.verifyMs = 0U;
    f.pressUp();
    f.runUntil(ShiftPhase::Return);
    f.runFor(f.cfg.returnMs);

    ASSERT_PHASE(ShiftPhase::Lockout, f);
}

// ---------------------------------------------------------------------------
// Invariants under random abuse
// ---------------------------------------------------------------------------

static uint32_t lcgState = 12345U;
static uint32_t lcgNext(void)
{
    lcgState = (lcgState * 1664525U) + 1013904223U;
    return lcgState >> 8U;
}

static void test_randomInputsNeverBreakTheSafetyInvariants(void)
{
    for (uint32_t seed = 1U; seed <= 40U; ++seed)
    {
        lcgState = seed;
        gearbox_fixture f;
        f.cfg.maxCutMs = static_cast<uint16_t>(1U + (lcgNext() % 200U));
        f.cfg.cutFuel = (lcgNext() & 1U) != 0U;
        f.cfg.cutOnDownshift = (lcgNext() & 1U) != 0U;
        f.cfg.useFeedback = (lcgNext() & 1U) != 0U;
        f.cfg.verifyMs = static_cast<uint16_t>(lcgNext() % 200U);
        f.cfg.downshiftMaxRpm = static_cast<uint16_t>(lcgNext() % 12000U);
        f.cfg.upTravel = static_cast<uint16_t>(lcgNext() % 1500U);
        f.cfg.downTravel = static_cast<uint16_t>(lcgNext() % 1500U);
        f.cfg.moveMs = static_cast<uint16_t>(lcgNext() % 200U);
        f.cfg.holdMs = static_cast<uint16_t>(lcgNext() % 200U);
        f.cfg.returnMs = static_cast<uint16_t>(lcgNext() % 200U);
        f.cfg.cutLeadMs = static_cast<uint16_t>(lcgNext() % 60U);
        f.cfg.maxShiftMs = static_cast<uint16_t>(lcgNext() % 800U);
        f.cfg.minIntervalMs = static_cast<uint16_t>(lcgNext() % 400U);
        f.cfg.faultLockoutMs = static_cast<uint16_t>(lcgNext() % 400U);

        uint32_t cutStartedAt = 0U;
        bool cutWasOn = false;
        uint16_t lastShiftCount = 0U;

        for (uint32_t i = 0U; i < 3000U; ++i)
        {
            const ShiftPhase before = f.state.phase;
            const uint32_t advance = 1U + (lcgNext() % 5U);
            f.in.nowMs += advance;
            f.in.upRequest = (lcgNext() % 40U) == 0U;
            f.in.downRequest = (lcgNext() % 40U) == 0U;
            f.in.rpm = static_cast<uint16_t>(lcgNext() % 14000U);
            f.in.gear = static_cast<uint8_t>(lcgNext() % 8U);
            f.in.gearValid = (lcgNext() % 4U) != 0U;
            f.in.positionValid = (lcgNext() % 4U) != 0U;
            f.in.positionPermille = static_cast<int16_t>(static_cast<int32_t>(lcgNext() % 2400U) - 1200);
            if ((lcgNext() % 200U) == 0U) { f.cfg.enabled = !f.cfg.enabled; }

            const bool bothRequested = f.in.upRequest && f.in.downRequest;
            f.out = gearboxStep(f.state, f.cfg, f.in);

            const ShiftPhase after = f.state.phase;

            // 1. The actuator only moves while moving or holding, and never beyond full scale
            if (f.out.actuatorPermille != 0)
            {
                TEST_ASSERT_TRUE(f.cfg.enabled);
                TEST_ASSERT_TRUE((after == ShiftPhase::Move) || (after == ShiftPhase::Hold));
            }
            TEST_ASSERT_TRUE(f.out.actuatorPermille <= GEARBOX_PERMILLE_FULL);
            TEST_ASSERT_TRUE(f.out.actuatorPermille >= -GEARBOX_PERMILLE_FULL);

            // 2. No cut outside an active shift, never in Fault, never when disabled
            const bool cutOn = f.out.cutIgnition || f.out.cutFuel;
            if (cutOn)
            {
                TEST_ASSERT_TRUE(f.cfg.enabled);
                TEST_ASSERT_TRUE((after == ShiftPhase::PreCut) || (after == ShiftPhase::Move) || (after == ShiftPhase::Hold));
            }

            // 3. The actuator is disabled in Fault (and when the module is disabled), enabled otherwise
            TEST_ASSERT_EQUAL(f.cfg.enabled && (after != ShiftPhase::Fault), f.out.actuatorEnable);

            // 4. A shift never starts from both paddles at once, nor from outside Idle
            if ((before != ShiftPhase::Idle) && f.cfg.enabled)
            {
                TEST_ASSERT_TRUE(after != ShiftPhase::PreCut || before == ShiftPhase::PreCut);
            }
            if (bothRequested && (before == ShiftPhase::Idle))
            {
                TEST_ASSERT_TRUE(after == ShiftPhase::Idle);
            }

            // 5. Continuous cut time is bounded by maxCutMs
            if (cutOn)
            {
                if (!cutWasOn) { cutStartedAt = f.in.nowMs; }
                TEST_ASSERT_TRUE(static_cast<uint32_t>(f.in.nowMs - cutStartedAt) < f.cfg.maxCutMs);
            }
            cutWasOn = cutOn;

            // 6. The shift counter only goes up (when enabled), by at most one per step
            if (f.cfg.enabled)
            {
                TEST_ASSERT_TRUE(f.state.shiftCount >= lastShiftCount);
                TEST_ASSERT_TRUE(f.state.shiftCount <= static_cast<uint16_t>(lastShiftCount + 1U));
            }
            lastShiftCount = f.state.shiftCount;
        }
    }
}

void testGearboxSequence(void)
{
    SET_UNITY_FILENAME() {
        RUN_TEST_P(test_disabled_ignoresRequestsAndOutputsNothing);
        RUN_TEST_P(test_disablingMidShift_dropsEverythingButKeepsShiftCount);

        RUN_TEST_P(test_upshift_followsTheWholeSequenceWithTheExpectedTiming);
        RUN_TEST_P(test_upshift_noCutBelowMinRpm);
        RUN_TEST_P(test_upshift_cutAtExactlyMinRpm);
        RUN_TEST_P(test_upshift_cutDisabledInConfig);
        RUN_TEST_P(test_upshift_fuelOnlyCut);
        RUN_TEST_P(test_upshift_bothCutsWhenBothEnabled);
        RUN_TEST_P(test_upshift_noCutWhenNeitherIgnitionNorFuelSelected);
        RUN_TEST_P(test_upshift_zeroLeadTimeSkipsPreCut);
        RUN_TEST_P(test_upshift_travelIsClampedToFullScale);
        RUN_TEST_P(test_upshift_fromNeutralIsAccepted);

        RUN_TEST_P(test_downshift_movesTheOtherWayAndDoesNotCutByDefault);
        RUN_TEST_P(test_downshift_cutWhenEnabled);
        RUN_TEST_P(test_downshift_firstGearRefusedByDefault);
        RUN_TEST_P(test_downshift_firstGearToNeutralWhenAllowed);
        RUN_TEST_P(test_downshift_fromNeutralRefused);

        RUN_TEST_P(test_upshift_topGearRefused);
        RUN_TEST_P(test_upshift_belowTopGearAccepted);
        RUN_TEST_P(test_unknownGear_upshiftAndDownshiftAreAccepted);
        RUN_TEST_P(test_downshift_overRevPredictionBoundary);
        RUN_TEST_P(test_downshift_overRevCheckOffWhenLimitIsZero);
        RUN_TEST_P(test_downshift_overRevUsesCurrentRpmWhenRatiosUnknown);
        RUN_TEST_P(test_downshift_overRevUsesCurrentRpmWhenGearUnknown);
        RUN_TEST_P(test_bothPaddlesTogetherDoNothing);
        RUN_TEST_P(test_requestsAreRefusedNotQueuedWhileBusyOrLockedOut);

        RUN_TEST_P(test_cutNeverExceedsMaxCutEvenIfTheSequenceIsSlower);
        RUN_TEST_P(test_zeroMaxCutNeverCuts);
        RUN_TEST_P(test_watchdogFaultsAReallySlowSequenceAndReleasesEverything);
        RUN_TEST_P(test_zeroWatchdogFaultsImmediately);
        RUN_TEST_P(test_faultRecoversAfterLockoutAndClearsOnNextShift);
        RUN_TEST_P(test_timingSurvivesTheMillisecondCounterWrapping);

        RUN_TEST_P(test_feedback_moveWaitsForThePositionNotTheClock);
        RUN_TEST_P(test_feedback_moveTimeoutFaultsAndDisablesTheActuator);
        RUN_TEST_P(test_feedback_invalidPositionNeverCountsAsArrived);
        RUN_TEST_P(test_feedback_returnWaitsForCentreThenTimesOut);
        RUN_TEST_P(test_feedback_downshiftTargetsNegativeTravel);

        RUN_TEST_P(test_verify_succeedsWhenTheExpectedGearAppears);
        RUN_TEST_P(test_verify_faultsWhenTheGearNeverArrives);
        RUN_TEST_P(test_verify_wrongGearDoesNotCount);
        RUN_TEST_P(test_verify_skippedWhenTheStartingGearWasUnknown);
        RUN_TEST_P(test_verify_skippedWhenVerifyTimeIsZero);

        RUN_TEST_P(test_randomInputsNeverBreakTheSafetyInvariants);
    }
}
