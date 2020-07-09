
#include "matching.h"

/**
 * @brief Run circular match to the feature points detected
 * @param: The soft points detected from the four frames
 * @param: MatchParameters Combination
 * @return A vector of the matched points, the first dimension is the number of matched pairs, 
 * in the second dimension, each vector<SoftPoint> there has only four points
 **/
vector<vector<SoftPoint>> CircularMatch(vector<vector<SoftPoint>> &softPointsLeftOld,
                                        vector<vector<SoftPoint>> &softPointsLeftNew,
                                        vector<vector<SoftPoint>> &softPointsRightOld,
                                        vector<vector<SoftPoint>> &softPointsRightNew,
                                        MatchParameters matchParameters)
{
    vector<vector<int>> indexRightOld = GenerateBinIndex(softPointsRightOld, matchParameters);
    vector<vector<int>> indexLeftNew = GenerateBinIndex(softPointsLeftNew, matchParameters);
    vector<vector<int>> indexRightNew = GenerateBinIndex(softPointsRightNew, matchParameters);

    int stereoDistTolerance = matchParameters.matchDispTolerance;
    vector<vector<SoftPoint>> matchedPoints;

    for (int classIndex = 0; classIndex < 4; classIndex++)
    {
        vector<SoftPoint> pointsTemp;

        for (auto pointCurrent : softPointsLeftOld[classIndex])
        {
            pointsTemp.clear();

            //LeftOld - RightOld
            int indexMatchRightOld = 0, indexInBinRightOld = 0, binIndexInTheVectorRightOld = 0;
            tie(indexMatchRightOld, indexInBinRightOld, binIndexInTheVectorRightOld) =
                SearchMatchPoints(pointCurrent, softPointsRightOld[classIndex],
                                  indexRightOld, matchParameters, classIndex);

            if (indexMatchRightOld != -1 && abs(pointCurrent.y -
                                                softPointsRightOld[classIndex][indexMatchRightOld].y) <
                                                stereoDistTolerance)
            {
                pointsTemp.push_back(pointCurrent);
                pointsTemp.push_back(softPointsRightOld[classIndex][indexMatchRightOld]);
            }
            else
                continue;

            //RightOld-RightNew
            int indexMatchRightNew = 0, indexInBinRightNew = 0, binIndexInTheVectorRightNew = 0;
            tie(indexMatchRightNew, indexInBinRightNew, binIndexInTheVectorRightNew) =
                SearchMatchPoints(softPointsRightOld[classIndex][indexMatchRightOld],
                                  softPointsRightNew[classIndex],
                                  indexRightNew, matchParameters, classIndex);
            if (indexMatchRightNew != -1)
            {
                pointsTemp.push_back(softPointsRightNew[classIndex][indexMatchRightNew]);
            }
            else
                continue;

            //RightNew-LeftNew
            int indexMatchLeftNew = 0, indexInBinLeftNew = 0, binIndexInTheVectorLeftNew = 0;
            tie(indexMatchLeftNew, indexInBinLeftNew, binIndexInTheVectorLeftNew) =
                SearchMatchPoints(softPointsRightNew[classIndex][indexMatchRightNew],
                                  softPointsLeftNew[classIndex], indexLeftNew, matchParameters, classIndex);
            // matchedPoint.show();
            if (indexMatchLeftNew != -1 && abs(softPointsRightNew[classIndex][indexMatchRightNew].y -
                                               softPointsLeftNew[classIndex][indexMatchLeftNew].y) <
                                               stereoDistTolerance)
            {
                pointsTemp.push_back(softPointsLeftNew[classIndex][indexMatchLeftNew]);
            }
            else
                continue;

            //LeftNew-LeftOld
            double dist = (softPointsLeftNew[classIndex][indexMatchLeftNew].pattern - pointCurrent.pattern).lpNorm<1>();
            double distLeftMatch = (softPointsLeftNew[classIndex][indexMatchLeftNew].x - pointCurrent.x) *
                                       (softPointsLeftNew[classIndex][indexMatchLeftNew].x - pointCurrent.x) +
                                   (softPointsLeftNew[classIndex][indexMatchLeftNew].y - pointCurrent.y) *
                                       (softPointsLeftNew[classIndex][indexMatchLeftNew].y - pointCurrent.y);
            if (dist < matchParameters.dist_tol && distLeftMatch < matchParameters.radiusMatch * 3)
            {
                matchedPoints.push_back(pointsTemp);

                //Erase the matched poitns from index bins
                //Note: don't erase points from softPoints vectors
                // SoftPoint pointTemp = softPointsRightOld[classIndex][indexMatchRightOld];
                indexRightOld[binIndexInTheVectorRightOld].erase(indexRightOld[binIndexInTheVectorRightOld].begin() +
                                                                 indexInBinRightOld);
                indexRightNew[binIndexInTheVectorRightNew].erase(indexRightNew[binIndexInTheVectorRightNew].begin() +
                                                                 indexInBinRightNew);
                indexLeftNew[binIndexInTheVectorLeftNew].erase(indexLeftNew[binIndexInTheVectorLeftNew].begin() +
                                                               indexInBinLeftNew);
            }
            else
                continue;
        }
    }
    return matchedPoints;
}

/**
 * The matching of points is based on bins. The image is divided into multiple bins, 
 *      and points of one class are only matched in its and adjacent bins
 * @param: the SoftPoints detected from the first step, composed of 4 classes
 * @param: MatchParameters
 * @output: The index of points in each class of each bin, the length: 4*xBinNum*yBinNum 
 **/
vector<vector<int>> GenerateBinIndex(vector<vector<SoftPoint>> &softPoints, MatchParameters matchParameters)
{
    vector<vector<int>> indexInBin;
    for (int i = 0; i < 4 * matchParameters.xBinNum * matchParameters.yBinNum; i++)
    {
        vector<int> temp;
        indexInBin.push_back(temp);
    }

    for (int classIndex = 0; classIndex < 4; classIndex++)
    {
        for (int k = 0; k < static_cast<int>(softPoints[classIndex].size()); k++)
        {
            int xBinTemp = softPoints[classIndex][k].x / matchParameters.binSize;
            int yBinTemp = softPoints[classIndex][k].y / matchParameters.binSize;
            //This part of comment code could be useful if you want to make sure the code works
            //            if (classIndex * matchParameters.xBinNum * matchParameters.yBinNum +
            //                yBinTemp * matchParameters.xBinNum + xBinTemp==161)
            //            {cout<<1<<endl;}
            indexInBin[classIndex * matchParameters.xBinNum * matchParameters.yBinNum +
                       yBinTemp * matchParameters.xBinNum + xBinTemp]
                .push_back(k);
            if (xBinTemp >= matchParameters.xBinNum || yBinTemp >= matchParameters.yBinNum)
                cout << "!!!!Error detected from GenerateBinIndex" << endl;

            //This part of comment code could be useful if you want to make sure the code works
            // if (classIndex == 0 && k == 9)
            //     cout << "The k" << k << endl;
            // cout << xBinTemp << ", " << yBinTemp << endl;
            // cout << classIndex * xBinNum * yBinNum + yBinTemp * xBinNum + xBinTemp << endl;
            // cout << indexInBin[classIndex * xBinNum * yBinNum + yBinTemp * xBinNum + xBinTemp][indexInBin[classIndex * xBinNum * yBinNum + yBinTemp * xBinNum + xBinTemp].size() - 1] << endl;
            // cout << "****" << indexInBin.size() << endl;
        }
    }
    return indexInBin;
}

/**
 *  The function search for matched points of the given point
 * @param: point, the point to match
 * @param: softPoints, all the points in the next frame
 * @param: indexNextFrame, the bins that contain the index of all the points
 * @param: MatchParameters
 * @param: classIndex
 * @return: a tuple that contains three index: 
 * The index of the point in the big softPoints vector(one class of one image). -1 means failed matching
 * The index of the point in the bin
 * The index of the bin that contains the point
 **/
tuple<int, int, int> SearchMatchPoints(SoftPoint &point, vector<SoftPoint> &softPoints,
                                       vector<vector<int>> &indexNextFrame, MatchParameters matchParameters,
                                       int classIndex)
{
    double patternDiffTol = matchParameters.patternDiffTol;

    int xMinBin = point.x - matchParameters.radiusMatch;
    int xMaxBin = point.x + matchParameters.radiusMatch;
    int yMinBin = point.y - matchParameters.radiusMatch;
    int yMaxBin = point.y + matchParameters.radiusMatch;
    xMinBin = min(max(xMinBin - 1, 0) / matchParameters.binSize, matchParameters.xBinNum);
    xMaxBin = min(min(xMaxBin - 1, matchParameters.width) / matchParameters.binSize, matchParameters.xBinNum);
    yMinBin = min(max(yMinBin - 1, 0) / matchParameters.binSize, matchParameters.yBinNum);
    yMaxBin = min(min(yMaxBin - 1, matchParameters.height) / matchParameters.binSize, matchParameters.yBinNum);

    double minDiff = 24 * 255;
    int minIndex = -1;
    int minIndexInBin = 0;
    int binIndexInTheVector = 0;
    for (int i = xMinBin; i <= xMaxBin; i++)
        for (int j = yMinBin; j <= yMaxBin; j++)
        {
            int indexInBin = 0;
            for (int indexTemp : indexNextFrame[classIndex * matchParameters.xBinNum * matchParameters.yBinNum +
                                                j * matchParameters.xBinNum + i])
            {
                if (indexTemp >= static_cast<int>(softPoints.size()))
                    cout << "!!!!!!Error in searchMatchPoints, the given index exceeds the size of the vector!"
                         << indexTemp << endl;
                // softPoints[indexTemp].show();
                double diff = (softPoints[indexTemp].pattern - point.pattern).lpNorm<1>();
                if (diff < minDiff)
                {
                    minIndex = indexTemp;
                    minDiff = diff;
                    minIndexInBin = indexInBin;
                    binIndexInTheVector = classIndex * matchParameters.xBinNum * matchParameters.yBinNum +
                                          j * matchParameters.xBinNum + i;
                }
                indexInBin++;
            }
        }

    //Detect if results found
    if (minDiff < patternDiffTol && ((softPoints[minIndex].x - point.x) * (softPoints[minIndex].x - point.x) + (softPoints[minIndex].y - point.y) * (softPoints[minIndex].y - point.y)) <
                                        matchParameters.radiusMatch * matchParameters.radiusMatch)
    {
        return make_tuple(minIndex, minIndexInBin, binIndexInTheVector);
    }
    else
    {
        return make_tuple(-1, -1, -1);
    }
}

/**
 * @brief: remove matches that has low NCC values. This function directly change the input match sequence
 * @param: input: matches, etc
 * @output:
 **/
void NCC_Check(vector<vector<SoftPoint>> &pointsIn4,
               cv::Mat imgLeftOld, cv::Mat imgRightOld,
               cv::Mat imgRightNew, cv::Mat imgLeftNew, MatchParameters matchParameters)
{
    if (imgLeftOld.type() == 16)
        cv::cvtColor(imgLeftOld, imgLeftOld, CV_BGR2GRAY);
    if (imgRightOld.type() == 16)
        cv::cvtColor(imgRightOld, imgRightOld, CV_BGR2GRAY);
    if (imgRightNew.type() == 16)
        cv::cvtColor(imgRightNew, imgRightNew, CV_BGR2GRAY);
    if (imgLeftNew.type() == 16)
        cv::cvtColor(imgLeftNew, imgLeftNew, CV_BGR2GRAY);

    for (int i = 0; i < static_cast<int>(pointsIn4.size()); i++)
    {
        if (!NCC_GatingOneMatchSet(pointsIn4[i], imgLeftOld, imgRightOld, imgRightNew, imgLeftNew, matchParameters))
        {
            //assign the element to remove the value of the last element, and then remove the last element
            pointsIn4[i] = pointsIn4.back();
            pointsIn4.pop_back();
        }
    }
}
/**
 * Check for one pair of match points
 **/
bool NCC_GatingOneMatchSet(vector<SoftPoint> &points,
                           cv::Mat &imgLeftOld, cv::Mat &imgRightOld,
                           cv::Mat &imgRightNew, cv::Mat &imgLeftNew, MatchParameters &matchParameters)
{

    int nccWindow = matchParameters.nccWindow;
    double nccTolerance = matchParameters.nccTolerance;

    if (!CheckOneMatch(points[0], points[1], imgLeftOld, imgRightOld, nccWindow, nccTolerance))
    {
        return false;
    }
    if (!CheckOneMatch(points[1], points[2], imgRightOld, imgRightNew, nccWindow, nccTolerance))
    {
        return false;
    }
    if (!CheckOneMatch(points[2], points[3], imgRightNew, imgLeftNew, nccWindow, nccTolerance))
    {
        return false;
    }
    if (!CheckOneMatch(points[3], points[0], imgLeftNew, imgLeftOld, nccWindow, nccTolerance))
    {
        return false;
    }
    return true;
}

/**
 * Check for two points in one pair of match points
 **/
bool CheckOneMatch(SoftPoint p1, SoftPoint p2, cv::Mat img1, cv::Mat img2, int nccWindow, double nccTolerance)
{
    if (p1.x - nccWindow < 0 || p2.x - nccWindow < 0 || p1.y - nccWindow < 0 || p2.y - nccWindow < 0 || p1.x + nccWindow > img1.cols - 1 ||
        p2.x + nccWindow > img2.cols - 1 || p1.y + nccWindow > img1.rows - 1 || p2.y + nccWindow > img2.rows - 1)
        return false;
    cv::Rect mask1(cv::Point(p1.x - nccWindow, p1.y - nccWindow),
                   cv::Point(p1.x + nccWindow + 1, p1.y + nccWindow + 1));
    cv::Mat template1 = img1(mask1);
    cv::Rect mask2(cv::Point(p2.x - nccWindow, p2.y - nccWindow),
                   cv::Point(p2.x + nccWindow + 1, p2.y + nccWindow + 1));
    cv::Mat template2 = img2(mask2);

    cv::Mat result;
    result.create(1, 1, CV_32F);

    cv::matchTemplate(template1, template2, result, CV_TM_CCORR_NORMED);
    // cout << "Result for one match: " << result << endl;
    if (result.at<float>(0, 0) > nccTolerance)
        return true;
    else
        return false;
}

/**
 * Draws the locations of the matched feature points in the four frames: LeftOld, RightOld, RightNew, LeftNew
 * @param: imgs, the four frames in the order above
 * @param: points4Class, the matched points in these four frames
 * @param: radius, the radius to draw the features on the frames
 **/
void DrawFeaturePointsForMatch(vector<cv::Mat> imgs, vector<vector<SoftPoint>> &points4Class, int radius)
{
    //    int colorDist=255/static_cast<int>(points4Class.size())/2;
    for (int j = 0; j < static_cast<int>(points4Class.size()); j++)
    {
        for (int i = 0; i < 4; i++)
        {
            cv::circle(imgs[i], cv::Point(points4Class[j][i].x, points4Class[j][i].y), radius, cv::Scalar(0, 0, 255));

            ShortImgShow(imgs[i], "i=" + to_string(i));
            points4Class[j][i].show();
        }
        cout << endl
             << endl;
    }
}
