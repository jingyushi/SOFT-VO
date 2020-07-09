

#include "CppUnitLite/TestHarness.h"

#include "soft_slam_io.h"

using namespace std;

int main()
{

    TestResult tr;
    TestRegistry::runAllTests(tr);

    return 0;
}

TEST(Stack, creation)
{
    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;
    string pathDataset = "/home/zephyr/Data/KITTI/data_odometry_gray/dataset/sequences/00";
    LoadImages(string(pathDataset), vstrImageLeft, vstrImageRight, vTimestamps);

    std::string pathFirst = "/home/zephyr/Data/KITTI/data_odometry_gray/dataset/sequences/00/image_0/000000.png";
    std::string pathToCheck = vstrImageLeft[0];
    CHECK_EQUAL(pathFirst, pathToCheck);
}
