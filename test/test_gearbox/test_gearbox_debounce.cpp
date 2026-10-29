#include "gearbox_fixture.h"

static void test_debounce_pressShorterThanDebounceIsIgnored(void)
{
    debounce_t d;
    debouncePrime(d, false, 0U);

    TEST_ASSERT_FALSE(debounceRisingEdge(d, true, 100U, 20U));   // contact closes
    TEST_ASSERT_FALSE(debounceRisingEdge(d, true, 110U, 20U));   // still within debounce
    TEST_ASSERT_FALSE(debounceRisingEdge(d, false, 115U, 20U));  // bounces open
    TEST_ASSERT_FALSE(debounceRisingEdge(d, false, 200U, 20U));
}

static void test_debounce_edgeFiresExactlyOnceAfterDebounceTime(void)
{
    debounce_t d;
    debouncePrime(d, false, 0U);

    TEST_ASSERT_FALSE(debounceRisingEdge(d, true, 100U, 20U));
    TEST_ASSERT_FALSE(debounceRisingEdge(d, true, 119U, 20U));
    TEST_ASSERT_TRUE(debounceRisingEdge(d, true, 120U, 20U));
    TEST_ASSERT_FALSE(debounceRisingEdge(d, true, 121U, 20U));
    TEST_ASSERT_FALSE(debounceRisingEdge(d, true, 5000U, 20U));
}

static void test_debounce_bouncingContactRestartsTheTimer(void)
{
    debounce_t d;
    debouncePrime(d, false, 0U);

    TEST_ASSERT_FALSE(debounceRisingEdge(d, true, 100U, 20U));
    TEST_ASSERT_FALSE(debounceRisingEdge(d, false, 110U, 20U));
    TEST_ASSERT_FALSE(debounceRisingEdge(d, true, 115U, 20U));
    TEST_ASSERT_FALSE(debounceRisingEdge(d, true, 130U, 20U));  // only 15ms since the last change
    TEST_ASSERT_TRUE(debounceRisingEdge(d, true, 135U, 20U));
}

static void test_debounce_releaseThenPressFiresAgain(void)
{
    debounce_t d;
    debouncePrime(d, false, 0U);

    debounceRisingEdge(d, true, 100U, 20U);
    TEST_ASSERT_TRUE(debounceRisingEdge(d, true, 120U, 20U));

    TEST_ASSERT_FALSE(debounceRisingEdge(d, false, 200U, 20U));
    TEST_ASSERT_FALSE(debounceRisingEdge(d, false, 220U, 20U));  // release is not an edge of interest

    TEST_ASSERT_FALSE(debounceRisingEdge(d, true, 300U, 20U));
    TEST_ASSERT_TRUE(debounceRisingEdge(d, true, 320U, 20U));
}

static void test_debounce_paddleHeldAtStartupIsNotAPress(void)
{
    debounce_t d;
    debouncePrime(d, true, 0U);

    TEST_ASSERT_FALSE(debounceRisingEdge(d, true, 10U, 20U));
    TEST_ASSERT_FALSE(debounceRisingEdge(d, true, 1000U, 20U));
}

static void test_debounce_zeroDebounceFiresOnTheFirstPressedSample(void)
{
    debounce_t d;
    debouncePrime(d, false, 0U);

    TEST_ASSERT_TRUE(debounceRisingEdge(d, true, 5U, 0U));
}

static void test_debounce_survivesTheMillisecondCounterWrapping(void)
{
    debounce_t d;
    debouncePrime(d, false, 0xFFFFFFF0U);

    TEST_ASSERT_FALSE(debounceRisingEdge(d, true, 0xFFFFFFF8U, 20U));
    TEST_ASSERT_FALSE(debounceRisingEdge(d, true, 0x00000005U, 20U));  // 13ms later
    TEST_ASSERT_TRUE(debounceRisingEdge(d, true, 0x0000000CU, 20U));   // 20ms later
}

void testGearboxDebounce(void)
{
    {
        RUN_TEST(test_debounce_pressShorterThanDebounceIsIgnored);
        RUN_TEST(test_debounce_edgeFiresExactlyOnceAfterDebounceTime);
        RUN_TEST(test_debounce_bouncingContactRestartsTheTimer);
        RUN_TEST(test_debounce_releaseThenPressFiresAgain);
        RUN_TEST(test_debounce_paddleHeldAtStartupIsNotAPress);
        RUN_TEST(test_debounce_zeroDebounceFiresOnTheFirstPressedSample);
        RUN_TEST(test_debounce_survivesTheMillisecondCounterWrapping);
    }
}
