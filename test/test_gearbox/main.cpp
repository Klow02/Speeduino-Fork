/**
 * Runs all tests of the paddle shift gearbox controller (src/controllers/gearbox).
 * PlatformIO builds this as a test program for a board (setup()/loop()); the logic has no hardware
 * access, so the same test files also compile for a PC (see ../test_cantable/README.txt).
 */
#include <Arduino.h>
#include <unity.h>

extern void testGearboxSequence(void);
extern void testGearboxDebounce(void);
extern void testGearboxController(void);
extern void testGearboxSettings(void);

#define UNITY_EXCLUDE_DETAILS

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    // NOTE: wait for >2 s if the board does not support a software reset via Serial.DTR/RTS
#if !defined(SIMULATOR)
    delay(2000);
#endif

    UNITY_BEGIN();
    testGearboxSequence();
    testGearboxDebounce();
    testGearboxController();
    testGearboxSettings();
    UNITY_END();
}

void loop()
{
    // nothing to do: the tests run once in setup()
}
