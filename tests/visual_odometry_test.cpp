
#include "CppUnitLite/TestHarness.h"

#include "soft_slam_io.h"
#include "feature.h"
#include "matching.h"

#include "config.h"

TEST(VO, 1)
{
}

int main()
{

    TestResult tr;
    TestRegistry::runAllTests(tr);

    return 0;
}
