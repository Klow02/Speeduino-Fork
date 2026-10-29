#pragma once
/**
 * @file gearboxHwArduino.h
 * @brief The two functions Speeduino's main loop calls for the paddle shift gearbox controller.
 */
#include <stdint.h>

/** Runs one pass of the gearbox control. Call once per main-loop pass. Does nothing while the gearbox is disabled (the default). */
void gearboxTask(void);

/**
 * Applies the shift cut to the ignition / fuel channel masks (only removes channels, never re-enables
 * any). Call after the rev limiter logic has set the masks. Does nothing while the gearbox is disabled.
 */
void gearboxApplyShiftCut(uint8_t& ignitionChannelsOn, uint8_t& fuelChannelsOn, uint8_t& ignitionChannelsPending);
