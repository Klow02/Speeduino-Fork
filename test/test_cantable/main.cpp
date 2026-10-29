/**
 * Runs all tests of the CAN message table (src/controllers/canTable).
 * Same layout as the other test folders: PlatformIO builds this as a test program for a board
 * (setup()/loop()). The table logic itself has no hardware access, so the very same test files can
 * also be compiled for a PC (see test/test_cantable/README.txt).
 */
#include <Arduino.h>
#include <unity.h>

extern void testCanTableValidate(void);
extern void testCanTableScaling(void);
extern void testCanTablePacking(void);
extern void testCanTableDdu9(void);
extern void testCanTableScheduler(void);
extern void testCanTableGlue(void);

#define UNITY_EXCLUDE_DETAILS

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    // NOTE: wait for >2 s if the board does not support a software reset via Serial.DTR/RTS
#if !defined(SIMULATOR)
    delay(2000);
#endif

    UNITY_BEGIN();
    testCanTableValidate();
    testCanTableScaling();
    testCanTablePacking();
    testCanTableDdu9();
    testCanTableScheduler();
    testCanTableGlue();
    UNITY_END();
}

void loop()
{
    // nothing to do: the tests run once in setup()
}
