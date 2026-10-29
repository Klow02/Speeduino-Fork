/**
 * @file canTableGlue.cpp
 * @brief Implementation of canTableGlue.h. No direct hardware access: see canTableHwTeensy41.cpp.
 */
#include "canTableGlue.h"
#include <string.h>

// ------------------------------------------------------------------------------------------------
//  1. Sources
// ------------------------------------------------------------------------------------------------

/** psi -> kPa, rounded to the nearest whole kPa (1 psi = 6.895 kPa). */
static int32_t psiToKpa(int32_t psi)
{
  return (psi * 6895 + 500) / 1000;
}

void canTableFillSources(const canTableLive_t& live, canTableSources_t& out)
{
  out.value[CAN_SRC_NONE]              = 0;
  out.value[CAN_SRC_RPM]               = live.rpm;
  out.value[CAN_SRC_MAP_KPA]           = live.mapKpa;
  out.value[CAN_SRC_TPS_PERCENT]       = live.tpsPercent;
  out.value[CAN_SRC_CLT_C]             = live.cltC;
  out.value[CAN_SRC_IAT_C]             = live.iatC;
  out.value[CAN_SRC_BATTERY_100MV]     = live.battery100mV;
  out.value[CAN_SRC_SPEED_KPH]         = live.speedKph;
  out.value[CAN_SRC_GEAR]              = live.gear;
  out.value[CAN_SRC_OIL_PRESSURE_KPA]  = psiToKpa(live.oilPressurePsi);
  out.value[CAN_SRC_FUEL_PRESSURE_KPA] = psiToKpa(live.fuelPressurePsi);
  out.value[CAN_SRC_FUEL_TEMP_C]       = live.fuelTempC;
  // Lambda x 1000 = (AFR x 10) * 1000 / (stoich AFR x 10). Without a stoich value we cannot tell: send 0.
  out.value[CAN_SRC_LAMBDA_MILLI]      = (live.stoichX10 == 0U) ? 0 : (int32_t)(((uint32_t)live.afrX10 * 1000UL) / live.stoichX10);
  // CAN_SRC_USER1 ... USER8 are deliberately left alone (they belong to whoever fills them).
}

// ------------------------------------------------------------------------------------------------
//  Helpers
// ------------------------------------------------------------------------------------------------

/**
 * Fingerprint of the table (FNV-1a, 32 bit). Used only to notice "something changed", so a simple
 * hash is enough. The chance that a real change gives the same number is about 1 in 4 billion.
 */
static uint32_t hashTable(const canTableConfig_t& cfg)
{
  const uint8_t* p = (const uint8_t*)&cfg;
  uint32_t h = 2166136261UL;
  for (size_t i = 0; i < sizeof(cfg); ++i)
  {
    h ^= p[i];
    h *= 16777619UL;
  }
  return h;
}

/** The hardware functions used by the send callback. canTableProcess() gives the callback no context
 *  pointer, so the active HAL is kept here for the duration of one update call. */
static const canTableHal_t* activeHal = nullptr;

static bool sendThroughHal(uint8_t channel, uint16_t id, const uint8_t data[8])
{
  return (activeHal != nullptr) && activeHal->sendFrame(channel, id, data);
}

/** Switches every channel and termination pin off (state "table not usable"). */
static void releaseAll(canTableController_t& ctrl, const canTableHal_t& hal)
{
  for (uint8_t ch = 0; ch < CAN_TABLE_CHANNELS; ++ch)
  {
    if (ctrl.terminationPin[ch] != CAN_TABLE_NO_PIN)
    {
      hal.setTerminationPin(ctrl.terminationPin[ch], false);
      ctrl.terminationPin[ch] = CAN_TABLE_NO_PIN;
    }
    ctrl.channelUp[ch] = false;
  }
  ctrl.sending = false;
}

/** The table changed: check it, then start the channels it asks for. */
static void applyConfig(canTableController_t& ctrl, const canTableConfig_t& cfg, const canTableHal_t& hal)
{
  releaseAll(ctrl, hal);
  canTableResetState(ctrl.state); // after a change every message counts as "never sent" and goes out right away

  ctrl.lastError = canTableValidate(cfg, ctrl.errorIndex);
  if (ctrl.lastError != CAN_TABLE_OK)
  {
    return; // broken table: stay silent
  }

  for (uint8_t ch = 0; ch < CAN_TABLE_CHANNELS; ++ch)
  {
    const canTableChannel_t& c = cfg.channel[ch];
    if (c.enabled == 0U) { continue; }

    ctrl.channelUp[ch] = hal.beginChannel(ch, c.bitrateKbit);

    // Termination pin: only if one is configured AND it is not used by Speeduino for something else.
    // Driving a pin that is used for e.g. an injector or coil output could damage something, so such a
    // setting is ignored.
    if (ctrl.channelUp[ch] && c.terminationPin != CAN_TABLE_NO_PIN && hal.pinIsFree(c.terminationPin))
    {
      hal.setTerminationPin(c.terminationPin, true);
      ctrl.terminationPin[ch] = c.terminationPin;
    }
  }
  ctrl.sending = true;
}

// ------------------------------------------------------------------------------------------------
//  Controller
// ------------------------------------------------------------------------------------------------

void canTableControllerReset(canTableController_t& ctrl)
{
  memset(&ctrl, 0, sizeof(ctrl));
  canTableResetState(ctrl.state);
  ctrl.lastError = CAN_TABLE_OK;
  for (uint8_t ch = 0; ch < CAN_TABLE_CHANNELS; ++ch)
  {
    ctrl.terminationPin[ch] = CAN_TABLE_NO_PIN;
  }
}

void canTableControllerUpdate(canTableController_t& ctrl, const canTableConfig_t& cfg, const canTableLive_t& live,
                              const canTableHal_t& hal, uint32_t nowMs)
{
  // Subtraction of unsigned numbers stays correct when the millisecond counter wraps around.
  const bool checkDue = !ctrl.configKnown || ((uint32_t)(nowMs - ctrl.lastCheckMs) >= CAN_TABLE_CHANGE_CHECK_MS);
  if (checkDue)
  {
    ctrl.lastCheckMs = nowMs;
    const uint32_t h = hashTable(cfg);
    if (!ctrl.configKnown || h != ctrl.cfgHash)
    {
      ctrl.cfgHash = h;
      ctrl.configKnown = true;
      applyConfig(ctrl, cfg, hal);
    }
  }

  if (!ctrl.sending)
  {
    return;
  }

  canTableFillSources(live, ctrl.sources);
  activeHal = &hal;
  (void)canTableProcess(cfg, ctrl.state, ctrl.sources, nowMs, sendThroughHal);
  activeHal = nullptr;
}
