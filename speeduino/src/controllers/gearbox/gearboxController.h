#pragma once

/**
 * @file
 * @brief Paddle shift controller: glue between Speeduino (pins, clock, status) and the
 *        hardware independent sequencer in gearboxStateMachine.h
 */

#include "config_pages.h"
#include "statuses.h"
#include "src/pins/pinNumbers_t.h"
#include "gearboxStateMachine.h"

/** @brief Electrical configuration of the gearbox inputs */
struct gearbox_io_config_t
{
  bool paddlesActiveLow = true;  ///< Paddle switches pull the pin to ground when pressed
  bool neutralActiveLow = true;  ///< Neutral switch pulls the pin to ground in neutral
};

/**
 * @brief Actuator backend, called every step with the commanded state.
 *
 * Implement this for the servo / motor driver: map @p permille (+1000 full up, -1000 full down, 0 centre)
 * to a pulse width or position setpoint. When @p enable is false the driver must be de-energised.
 */
using gearboxActuator_t = void (*)(bool enable, int16_t permille);

/**
 * @brief The active configuration.
 *
 * TODO: this is not stored in a TunerStudio page yet, so it is only changeable in code.
 * It defaults to disabled.
 */
gearbox_config_t& gearboxConfig(void);
gearbox_io_config_t& gearboxIoConfig(void);

/** @brief Register the actuator backend. nullptr = none (cut and status only). */
void gearboxSetActuator(gearboxActuator_t actuator);

/** @brief Claim the pins. Disables the module if the paddle pins are not valid. */
void initialiseGearbox(statuses &current, const pinNumbers_t &pins);

/** @brief Run the gearbox control. Call every pass of the main loop. */
void gearboxControl(statuses &current, const config2 &page2);
