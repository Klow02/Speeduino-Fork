#include "../test_harness_device.h"
#include "../test_harness_native.h"

void runAllTests(void)
{
    extern void testGearboxSequence(void);
    extern void testGearboxDebounce(void);
    extern void testGearboxController(void);

    testGearboxSequence();
    testGearboxDebounce();
    testGearboxController();
}

TEST_HARNESS(runAllTests)
