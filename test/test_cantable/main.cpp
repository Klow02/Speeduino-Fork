#include "../test_harness_device.h"
#include "../test_harness_native.h"

// Runs all tests of the CAN message table (src/controllers/canTable).
void runAllTests(void)
{
    extern void testCanTableValidate(void);
    extern void testCanTableScaling(void);
    extern void testCanTablePacking(void);
    extern void testCanTableDdu9(void);
    extern void testCanTableScheduler(void);
    extern void testCanTableGlue(void);

    testCanTableValidate();
    testCanTableScaling();
    testCanTablePacking();
    testCanTableDdu9();
    testCanTableScheduler();
    testCanTableGlue();
}

TEST_HARNESS(runAllTests)
