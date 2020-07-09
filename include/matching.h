
#include <tuple>
#include "soft_slam_io.h"
#include "feature.h"

struct MatchParameters
{
    int width;
    int height;
    int binSize;
    int radiusMatch;
    double matchDispTolerance;
    int nccWindow;
    double nccTolerance;
    int xBinNum;
    int yBinNum;
    double dist_tol;
    double patternDiffTol;
    MatchParameters(int width,
                    int height,
                    int binSize,
                    int radiusMatch,
                    double matchDispTolerance,
                    int nccWindow,
                    double nccTolerance,
                    int xBinNum,
                    int yBinNum,
                    double dist_tol,
                    double patternDiffTol) : width(width), height(height), binSize(binSize),
                                             radiusMatch(radiusMatch), matchDispTolerance(matchDispTolerance),
                                             nccWindow(nccWindow), nccTolerance(nccTolerance), xBinNum(xBinNum),
                                             yBinNum(yBinNum), dist_tol(dist_tol), patternDiffTol(patternDiffTol) {}
};

vector<vector<int>> GenerateBinIndex(vector<vector<SoftPoint>> &softPoints, MatchParameters matchParameters);

tuple<int, int, int> SearchMatchPoints(SoftPoint &point, vector<SoftPoint> &softPoints,
                                       vector<vector<int>> &indexNextFrame, MatchParameters matchParameters,
                                       int classIndex);

vector<vector<SoftPoint>> CircularMatch(vector<vector<SoftPoint>> &softPointsLeftOld, vector<vector<SoftPoint>> &softPointsLeftNew,
                                        vector<vector<SoftPoint>> &softPointsRightOld, vector<vector<SoftPoint>> &softPointsRightNew,
                                        MatchParameters matchParameters);

bool CheckOneMatch(SoftPoint p1, SoftPoint p2, cv::Mat img1, cv::Mat img2, int nccWindow, double nccTolerance);

bool NCC_GatingOneMatchSet(vector<SoftPoint> &points,
                           cv::Mat &imgLeftOld, cv::Mat &imgRightOld,
                           cv::Mat &imgRightNew, cv::Mat &imgLeftNew, MatchParameters &matchParameters);

void NCC_Check(vector<vector<SoftPoint>> &pointsIn4,
               cv::Mat imgLeftOld, cv::Mat imgRightOld,
               cv::Mat imgRightNew, cv::Mat imgLeftNew, MatchParameters matchParameters);

void DrawFeaturePointsForMatch(vector<cv::Mat> imgs, vector<vector<SoftPoint>> &points4Class, int radius = 3);
