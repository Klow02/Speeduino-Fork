#pragma once

#include <unity.h>
#include "src/controllers/gearbox/gearboxStateMachine.h"

/**
 * Drives gearboxStep() in 1ms steps with a simulated clock.
 *
 * Defaults: enabled, 2nd gear of 5, 8000rpm, ratios rising with gear.
 */
struct gearbox_fixture
{
    gearbox_config_t cfg;
    gearbox_state_t state;
    gearbox_inputs_t in;
    gearbox_outputs_t out;
    uint32_t pressedAtMs = 0U;

    gearbox_fixture()
    {
        cfg.enabled = true;
        in.nowMs = 1000U;
        in.gearValid = true;
        in.gear = 2U;
        in.rpm = 8000U;
        const uint16_t ratios[GEARBOX_MAX_GEARS] = { 20U, 36U, 52U, 68U, 84U, 100U };
        for (uint8_t i = 0U; i < GEARBOX_MAX_GEARS; ++i)
        {
            in.ratio[i] = ratios[i];
        }
    }

    /** One step with no paddle activity, after advancing the clock */
    void step(uint32_t advanceMs = 1U)
    {
        in.nowMs += advanceMs;
        in.upRequest = false;
        in.downRequest = false;
        out = gearboxStep(state, cfg, in);
    }

    /** One step in which a paddle was pressed (no clock advance) */
    void press(bool up, bool down = false)
    {
        in.upRequest = up;
        in.downRequest = down;
        pressedAtMs = in.nowMs;
        out = gearboxStep(state, cfg, in);
        in.upRequest = false;
        in.downRequest = false;
    }

    void pressUp(void) { press(true, false); }
    void pressDown(void) { press(false, true); }

    /** Milliseconds since the last press */
    uint32_t sincePress(void) const { return in.nowMs - pressedAtMs; }

    /** Step until the phase is reached. Returns ms since the press, or UINT32_MAX if not reached within limitMs. */
    uint32_t runUntil(ShiftPhase phase, uint32_t limitMs = 5000U)
    {
        for (uint32_t i = 0U; i < limitMs; ++i)
        {
            if (state.phase == phase)
            {
                return sincePress();
            }
            step();
        }
        return (state.phase == phase) ? sincePress() : UINT32_MAX;
    }

    void runFor(uint32_t ms)
    {
        for (uint32_t i = 0U; i < ms; ++i)
        {
            step();
        }
    }
};
