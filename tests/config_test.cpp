
#include "CppUnitLite/TestHarness.h"

#include "config.h"

// TEST(Config, get)
// {
//     Config sth("kitti00.yaml");
//     double fx = sth.get<double>("Camera.fx");
//     double fy = sth.get<double>("Camera.fy");
//     double bf = sth.get<double>("Camera.bf");
//     double th = sth.get<double>("ThDepth");
//     CHECK_EQUAL(718.8560, fx);
//     CHECK_EQUAL(35, th);
// }
template <class T>
T transform(std::string input)
{
    return T(input);
}
TEST(Config, get)
{
    cout << (double)ConfigParameters["Camera.fx"];
    // cout << transform<double>("321") << endl;
    // Config sth("kitti00.yaml");
    // double fx = sth.get<double>("Camera.fx");
    // double fy = sth.get<double>("Camera.fy");
    // double bf = sth.get<double>("Camera.bf");
    // double th = sth.get<double>("ThDepth");
    // CHECK_EQUAL(719, fx);
    // CHECK_EQUAL(35, th);
}

int main()
{

    TestResult tr;
    TestRegistry::runAllTests(tr);

    cout << CV_MAJOR_VERSION << endl;
    cout << CV_MINOR_VERSION << endl;
    return 0;
}
