

#include "CppUnitLite/TestHarness.h"

#include "feature.h"
#include "matching.h"
#include "soft_slam_io.h"

#include <chrono>

#include "config.h"

TEST(Camera, ReadCameraConfig)
{
    string path = "/home/zephyr/Data/KITTI/data_odometry_gray/dataset/sequences/00/calib.txt";
    vector<CameraConfig> sth = ReadCameraConfig<CameraConfig>(path);
    cout << sth[0] << endl
         << endl;
    CHECK_EQUAL(718, static_cast<int>(sth[0](0)));
}

TEST(Camera, ReadGroundTruth)
{
    string path = "/home/zephyr/Data/KITTI/data_odometry_poses/dataset/poses/00.txt";
    vector<PoseT> sth = ReadPoseGroundTruth<PoseT>(path);
    CHECK_EQUAL(1, sth[0](0));
}

vector<vector<SoftPoint>> SomePreCalculation()
{
    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;
    string pathDataset = "/home/zephyr/Data/KITTI/data_odometry_gray/dataset/sequences/00";
    LoadImages(string(pathDataset), vstrImageLeft, vstrImageRight, vTimestamps);

    cv::Mat imgLeftOld, imgRightOld, imgLeftNew, imgRightNew;
    int ni = 0;
    imgLeftOld = cv::imread(vstrImageLeft[ni], 1);
    imgRightOld = cv::imread(vstrImageRight[ni], 1);
    ni = 1;
    imgLeftNew = cv::imread(vstrImageLeft[ni], 1);
    imgRightNew = cv::imread(vstrImageRight[ni], 1);

    int binSize = (int)ConfigParameters["binSize"];
    int radiusMatch = (int)ConfigParameters["radiusMatch"];
    double matchDispTolerance = (double)ConfigParameters["matchDispTolerance"];
    int nccWindow = (int)ConfigParameters["nccWindow"];
    double nccTolerance = (double)ConfigParameters["nccTolerance"]; //should be close to 1
    double patternDiffTol = (double)ConfigParameters["patternDiffTol"];
    double dist_tol = (double)ConfigParameters["dist_tol"];
    int xBinNum = ceil(imgLeftOld.size().width / (double)binSize);
    int yBinNum = ceil(imgLeftOld.size().height / (double)binSize);

    MatchParameters matchParameters(imgLeftOld.size().width, imgLeftOld.size().height, binSize, radiusMatch, matchDispTolerance,
                                    nccWindow, nccTolerance, xBinNum, yBinNum, dist_tol, patternDiffTol);

    vector<vector<SoftPoint>> softPointsLeftOld = GenerateFeatureObjects(imgLeftOld);
    vector<vector<SoftPoint>> softPointsLeftNew = GenerateFeatureObjects(imgLeftNew);
    vector<vector<SoftPoint>> softPointsRightOld = GenerateFeatureObjects(imgRightOld);
    vector<vector<SoftPoint>> softPointsRightNew = GenerateFeatureObjects(imgRightNew);
    vector<vector<SoftPoint>> matchedPoints = CircularMatch(softPointsLeftOld,
                                                            softPointsLeftNew, softPointsRightOld,
                                                            softPointsRightNew, matchParameters);

    vector<cv::Mat> imgs;
    imgs.push_back(imgLeftOld);
    imgs.push_back(imgRightOld);
    imgs.push_back(imgRightNew);
    imgs.push_back(imgLeftNew);

    // cout << "******"
    //      << "Found initial matches: " << matchedPoints.size() << "******" << endl;

    NCC_Check(matchedPoints, imgLeftOld,
              imgRightOld, imgRightNew,
              imgLeftNew, matchParameters);

    return matchedPoints;
}

TEST(Camera, recoverRotation_and_un_scaled_t)
{

    //*******************************************************************

    string pathCamera = "/home/zephyr/Data/KITTI/data_odometry_gray/dataset/sequences/00/calib.txt";
    vector<CameraConfig> cameraProjection = ReadCameraConfig<CameraConfig>(pathCamera);
    double focal = cameraProjection[0](0);
    cv::Point2d pp(cameraProjection[0](2), cameraProjection[0](6));

    auto start = chrono::high_resolution_clock::now();

    vector<vector<SoftPoint>> matchedPoints = SomePreCalculation();

    //AFter manually checking the results, remove the sixth-point and the last two
    //!!!!!!!!!!!!
    matchedPoints[6] = matchedPoints[10];
    matchedPoints.pop_back();
    matchedPoints.pop_back();
    matchedPoints.pop_back();

    auto leftMatches = SoftP2CVP(matchedPoints);

    cv::Mat E, R, t, mask;
    E = cv::findEssentialMat(leftMatches[1], leftMatches[0], focal, pp, cv::RANSAC, 0.99, 0.5, mask);
    cv::recoverPose(E, leftMatches[1], leftMatches[0], R, t, focal, pp, mask);

    auto stop = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::microseconds>(stop - start);
    cout << "*****Time for two frames: " << duration.count() / (double)1000000 << " seconds" << endl
         << endl;

    cout << R << endl;
    cout << t << endl;
    // cout << E << endl;
    cout << mask << endl;

    CHECK_EQUAL(1, t.at<double>(0, 2) > 0.8 && t.at<double>(0, 2) < 1);
}

int main()
{
    TestResult tr;
    TestRegistry::runAllTests(tr);

    return 0;
}
