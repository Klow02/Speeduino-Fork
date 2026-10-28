/**
 * @file canTableHwTeensy41.cpp
 * @brief The real CAN hardware access of the CAN message table, for the Teensy 4.1 only.
 *
 * The Teensy 4.1 has three CAN controllers. This table uses two of them:
 *    channel 0 = CAN1  (pins 22 = TX, 23 = RX)  - the same controller the normal Speeduino CAN code
 *                                                 uses (Can0 in comms_CAN.cpp). Its bus speed is
 *                                                 therefore SHARED: setting a bitrate for channel 0
 *                                                 changes the speed of the standard Speeduino CAN too.
 *    channel 1 = CAN3  (pins 31 = TX, 30 = RX)  - used only by the table.
 *  CAN2 (pins 1/0) stays free. An external CAN transceiver chip is needed on every channel:
 *  the Teensy pins only speak 3.3 V logic, not the CAN bus levels.
 *
 *  This file contains no decisions, only "start the controller", "send this frame" and "set that
 *  pin". The decisions are in canTableGlue.cpp, which is tested on a PC.
 *
 *  NOT YET TESTED ON REAL HARDWARE (written without a Teensy at hand).
 */
#if defined(CORE_TEENSY41) && !defined(UNIT_TEST)

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "canTableGlue.h"
#include "../../../globals.h"
#include "../../../comms_CAN.h"

// CAN3 object. Declared here (file scope) like Can0 in comms_CAN.cpp, see
// https://github.com/tonton81/FlexCAN_T4/issues/67 . Receive queue is small: nothing is received yet.
static FlexCAN_T4<CAN3, RX_SIZE_16, TX_SIZE_16> can3;

static bool hwBeginChannel(uint8_t channel, uint16_t bitrateKbit)
{
  if (bitrateKbit == 0U)
  {
    return false;
  }
  if (channel == 0U)
  {
    // CAN1 is already started by initCAN() (comms_CAN.cpp). Only change its speed.
    CAN_setBitrate((uint32_t)bitrateKbit * 1000UL);
    return true;
  }
  if (channel == 1U)
  {
    can3.begin();
    can3.setBaudRate((uint32_t)bitrateKbit * 1000UL);
    return true;
  }
  return false;
}

static bool hwSendFrame(uint8_t channel, uint16_t id, const uint8_t data[8])
{
  if (channel == 0U)
  {
    return CAN_writeFrame(id, data);
  }
  if (channel == 1U)
  {
    CAN_message_t msg;
    msg.id = id;
    msg.len = 8;
    msg.flags.extended = 0; // standard 11-bit ID
    memcpy(msg.buf, data, 8);
    return can3.write(msg) > 0;
  }
  return false;
}

static bool hwPinIsFree(uint8_t pin)
{
  // The pin must exist on the board and must not be used by any Speeduino function.
  return (pin < BOARD_MAX_DIGITAL_PINS) && !pinIsUsed(pin);
}

static void hwSetTerminationPin(uint8_t pin, bool on)
{
  // Convention: HIGH = the 120 ohm termination resistor is connected into the bus.
  pinMode(pin, OUTPUT);
  digitalWrite(pin, on ? HIGH : LOW);
}

static const canTableHal_t teensyHal = { hwBeginChannel, hwSendFrame, hwPinIsFree, hwSetTerminationPin };

/** Controller memory of the one and only table. */
static canTableController_t controller;
static bool controllerReady = false;

void canTableTask(void)
{
  if (!controllerReady)
  {
    canTableControllerReset(controller);
    controllerReady = true;
  }
  canTableControllerUpdate(controller, configPage16.can, currentStatus, configPage2.stoich, teensyHal, millis());
}

#endif // CORE_TEENSY41
