

#include "CppUnitLite/TestHarness.h"

#include "matching.h"

TEST(circular_match, NCC)
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

     vector<vector<SoftPoint>> softPointsLeftOld = GenerateFeatureObjects(imgLeftOld);
     vector<vector<SoftPoint>> softPointsLeftNew = GenerateFeatureObjects(imgLeftNew);
     vector<vector<SoftPoint>> softPointsRightOld = GenerateFeatureObjects(imgRightOld);
     vector<vector<SoftPoint>> softPointsRightNew = GenerateFeatureObjects(imgRightNew);

     int pointsNum1 = 0, pointsNum2 = 0, pointsNum3 = 0, pointsNum0 = 0;
     for (int i = 0; i < 4; i++)
     {
          pointsNum0 += softPointsLeftOld[i].size();
          pointsNum1 += softPointsRightOld[i].size();
          pointsNum2 += softPointsRightOld[i].size();
          pointsNum3 += softPointsLeftNew[i].size();
     }

     cout << "The number of initial feature points in the four frames: "
          << pointsNum0 << ", " << pointsNum1 << ", " << pointsNum2 << ", " << pointsNum3 << endl;

     int binSize = 50;
     int radiusMatch = 100;
     double matchDispTolerance = 5;
     int nccWindow = 5;
     double nccTolerance = 0.96; //should be close to 1
     double patternDiffTol = 24 * 40;

     int xBinNum = ceil(imgLeftOld.size().width / (double)binSize);
     int yBinNum = ceil(imgLeftOld.size().height / (double)binSize);
     double dist_tol = 100;

     MatchParameters matchParameters(imgLeftOld.size().width, imgLeftOld.size().height, binSize, radiusMatch, matchDispTolerance,
                                     nccWindow, nccTolerance, xBinNum, yBinNum, dist_tol, patternDiffTol);

     vector<vector<SoftPoint>> matchedPoints = CircularMatch(softPointsLeftOld,
                                                             softPointsLeftNew, softPointsRightOld,
                                                             softPointsRightNew, matchParameters);

     vector<cv::Mat> imgs;
     imgs.push_back(imgLeftOld);
     imgs.push_back(imgRightOld);
     imgs.push_back(imgRightNew);
     imgs.push_back(imgLeftNew);

     cout << "******"
          << "Found initial matches: " << matchedPoints.size() << "******" << endl;

     NCC_Check(matchedPoints, imgLeftOld,
               imgRightOld, imgRightNew,
               imgLeftNew, matchParameters);

     cout << "******"
          << " Matches after NCC: " << matchedPoints.size() << "******" << endl;
     DrawFeaturePointsForMatch(imgs, matchedPoints);
}

// TEST(circular_match, GenerateBinIndex)
// {
//     // Retrieve paths to images
//     vector<string> vstrImageLeft;
//     vector<string> vstrImageRight;
//     vector<double> vTimestamps;
//     string pathDataset = "/home/zephyr/Data/KITTI/data_odometry_gray/dataset/sequences/00";
//     LoadImages(string(pathDataset), vstrImageLeft, vstrImageRight, vTimestamps);

//     cv::Mat imgLeftOld, imgRightOld, imgLeftNew, imgRightNew;
//     int ni = 0;
//     imgLeftOld = cv::imread(vstrImageLeft[ni], 1);
//     imgRightOld = cv::imread(vstrImageRight[ni], 1);
//     ni = 1;
//     imgLeftNew = cv::imread(vstrImageLeft[ni], 1);
//     imgRightNew = cv::imread(vstrImageRight[ni], 1);
//     // ShortImgShow(imgLeftNew);

//     vector<vector<SoftPoint>> softPointsLeftOld = GenerateFeatureObjects(imgLeftOld);
//     vector<vector<SoftPoint>> softPointsLeftNew = GenerateFeatureObjects(imgLeftNew);
//     vector<vector<SoftPoint>> softPointsRightOld = GenerateFeatureObjects(imgRightOld);
//     vector<vector<SoftPoint>> softPointsRightNew = GenerateFeatureObjects(imgRightNew);

//     int binSize = 50;
//     int radiusMatch = 200;
//     double matchDispTolerance = 1;
//     int nccWindow = 21;
//     double nccTolerance = 0.3;

//     int xBinNum = ceil(imgLeftOld.size().width / (double)binSize);
//     int yBinNum = ceil(imgLeftOld.size().height / (double)binSize);
//     double dist_tol = 20;

//     MatchParameters matchParameters(imgLeftOld.size().width, imgLeftOld.size().height, binSize, radiusMatch, matchDispTolerance,
//                                     nccWindow, nccTolerance, xBinNum, yBinNum, dist_tol);

//     auto sth = GenerateBinIndex(softPointsLeftOld, matchParameters);
//     // cout << "The size of sth is :" << sth.size() << endl;
//     // softPointsLeftOld[3][sth[526][0]].show();
//     int actualIndex = softPointsLeftOld[3].size() - 1;
//     SoftPoint p = softPointsLeftOld[3][actualIndex];
//     int xBin = p.x / matchParameters.binSize;
//     int yBin = p.y / matchParameters.binSize;
//     bool ifPass = false;
//     for (int tryPoint : sth[3 * xBinNum * yBinNum + yBin * xBinNum + xBin])
//         if (softPointsLeftOld[3][tryPoint].x == p.x)
//         {
//             ifPass = true;
//             continue;
//         }

//     CHECK_EQUAL(true, ifPass);
// }

int main()
{

     TestResult tr;
     TestRegistry::runAllTests(tr);

     cout << CV_MAJOR_VERSION << endl;
     cout << CV_MINOR_VERSION << endl;
     return 0;
}
