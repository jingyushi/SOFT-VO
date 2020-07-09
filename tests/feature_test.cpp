

#include "CppUnitLite/TestHarness.h"

#include "soft_slam_io.h"
#include "feature.h"

TEST(GenerateFeatureValues, NMS_fast)
{
    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;
    string pathDataset = "/home/zephyr/Data/KITTI/data_odometry_gray/dataset/sequences/00";
    LoadImages(string(pathDataset), vstrImageLeft, vstrImageRight, vTimestamps);

    cv::Mat imgLeft, imgRight;
    int ni = 0;
    imgLeft = cv::imread(vstrImageLeft[ni], 1);
    imgRight = cv::imread(vstrImageRight[ni], 1);

    if (imgLeft.empty())
    {
        cerr << endl
             << "Failed to load image at: "
             << string(vstrImageLeft[ni]) << endl;
    }

    cv::Mat blobKernel = (cv::Mat_<double>(5, 5) << -1, -1, -1, -1, -1, -1, 1, 1, 1, -1, -1, 1, 8, 1, -1, -1, 1, 1, 1, -1, -1, -1, -1, -1, -1);
    cv::Mat cornerKernel = (cv::Mat_<double>(5, 5) << -1, -1, 0, 1, 1, -1, -1, 0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, -1, -1, 1, 1, 0, -1, -1);
    blobKernel = blobKernel / 25.0;
    cornerKernel = cornerKernel / 25.0;

    vector<cv::Point> blobPointsMax;
    vector<cv::Point> blobPointsMin;
    vector<cv::Point> cornerPointsMax;
    vector<cv::Point> cornerPointsMin;

    cv::Mat blobImg;
    cv::filter2D(imgLeft, blobImg, CV_32F, blobKernel); //Generally, don't use CV_64F
    cv::cvtColor(blobImg, blobImg, CV_BGR2GRAY);
    cv::Mat cornerImg;
    cv::filter2D(imgLeft, cornerImg, CV_32F, cornerKernel);
    cv::cvtColor(cornerImg, cornerImg, CV_BGR2GRAY);

    auto points = NMS_fast(blobImg, cornerImg, 40);
    for (auto ps : points)
    {
        for (auto point : ps)
            cout << point << endl;
        cout << endl
             << endl
             << endl
             << endl;
    }
    // CHECK_EQUAL(35, softPoints[0].pattern(0));

    DrawFeaturePoints(imgLeft, points);
}

TEST(GenerateFeatureValues, GenerateFeatureObjects)
{
    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;
    string pathDataset = "/home/zephyr/Data/KITTI/data_odometry_gray/dataset/sequences/00";
    LoadImages(string(pathDataset), vstrImageLeft, vstrImageRight, vTimestamps);

    cv::Mat imgLeft, imgRight;
    int ni = 0;
    imgLeft = cv::imread(vstrImageLeft[ni], 1);
    imgRight = cv::imread(vstrImageRight[ni], 1);

    if (imgLeft.empty())
    {
        cerr << endl
             << "Failed to load image at: "
             << string(vstrImageLeft[ni]) << endl;
    }
    auto ppoints = GenerateFeatureObjects(imgLeft);

    DrawFeaturePoints(imgLeft, ppoints);
    cout << "You are expected to see two same pictures to pass the two tests!" << endl;
}
int main()
{

    TestResult tr;
    TestRegistry::runAllTests(tr);

    cout << CV_MAJOR_VERSION << endl;
    cout << CV_MINOR_VERSION << endl;
    return 0;
}
