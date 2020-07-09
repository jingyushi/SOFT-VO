
/*
 * Copyright 2019 <copyright holder> <email>
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */

#include "feature_selection.h"

typedef vector<vector<vector<vector<SoftPoint>>>> Bucket;

/**
 * @file feature_selection.cpp
 * @author bill
 * @version 0.1
 * @date 2019-10-21
 * @brief The implementation of feature_selection part
 * @details This part takes in the output feature matches from the tracking part and select among the matches according to some criterions
 **/

/**
 * Generate buckets within a given image, with respect to a given feature class
 * 
 * @param Img The given image
 * @param &InputMatches Tracked Matches, each of the match contains four SoftPoints
 * @param BucketSize The size of a single bucket
 * @return A Bucket contains all matches within it and of the class assigned
 **/
/*
Bucket GenerateBuckets(cv::Mat Img, vector<vector<SoftPoint>> &InputMatches, int BucketSize=50)
{
  int ImageHeight = Img.rows;
  int ImageWidth = Img.cols;
  int BucketNumberX = ImageHeight/BucketSize;
  int BucketNumberY = ImageWidth/BucketSize;
  
  Bucket Buckets;
  Buckets.resize(BucketNumberX);
}
*/

/**
 * Sort The input bucket (Bubble Sort)
 * 
 * @param &InputBucket The bucket to be sorted, containing a set of matching within this bucket of the same class
 * @param AgeThreshold The threshold of ages
 * @return A sorted bucket, the smaller the index, the higher the rank
 **/
bool SortBucket(vector<vector<SoftPoint>> &InputBucket, int AgeThreshold)
{
    vector<SoftPoint> Temp;
    //cout << "sorting start" << endl;
    for (uint ii = 0; ii < InputBucket.size(); ii++)
    {
        for (uint jj = 0; jj < InputBucket.size() - ii - 1; jj++)
        {
            if ((InputBucket[jj][3].age == InputBucket[jj + 1][3].age) // jj+1 out of bound
                || (InputBucket[jj][3].age > AgeThreshold) || (InputBucket[jj + 1][3].age > AgeThreshold))
            {
                //cout << "condition 1 satisfied, comparing strength" <<endl;
                if (InputBucket[jj][3].value < InputBucket[jj + 1][3].value) //compare the strength of the features
                {
                    Temp = InputBucket[jj];
                    InputBucket[jj] = InputBucket[jj + 1];
                    InputBucket[jj + 1] = Temp;
                    //cout << "order exchanged for " << jj <<"&"<<ii<<" matches"<<endl;
                }
            }
            else
            {
                //cout << "condition 2 satisfied, comparing strength" <<endl;
                if (InputBucket[jj][3].age < InputBucket[jj + 1][3].age)
                {
                    Temp = InputBucket[jj];
                    InputBucket[jj] = InputBucket[jj + 1];
                    InputBucket[jj + 1] = Temp;
                    //cout << "order exchanged for " << jj <<"&"<<ii<<" matches"<<endl;
                }
            }
        }
    }
    return true;
}

/** @brief Generate buckets within a given image, with respect to a given feature class

// @param Img The given image. In our case we use the current left image for selection
// @param &InputMatches Tracked Matches, each of the match contains four SoftPoints
// @param BucketSize The size of a single bucket
// @param MaxFeaturesNum The maxium number of features selected within a bucket
// @return A set of selected matches, each of which contains four SoftPoints
*/
vector<vector<SoftPoint>> SelectFeatures(cv::Mat Img, vector<vector<SoftPoint>> &InputMatches, int BucketSize, int MaxFeaturesNum)
{
    Bucket BucketsBlobMax;
    Bucket BucketsBlobMin;
    Bucket BucketsCornerMax;
    Bucket BucketsCornerMin;

    int AgeThreshold = 4;

    int ImageHeight = Img.rows;
    int ImageWidth = Img.cols;
    int BucketNumberX = ImageWidth / BucketSize;
    int BucketNumberY = ImageHeight / BucketSize;
    int StartingX = ImageWidth % BucketSize / 2;
    int StartingY = ImageHeight % BucketSize / 2; // In the case of size not being divisible by 50, pixels before the starting coordinate will be ignored
    /*cout << "ImageHeight: " <<ImageHeight <<endl;
  cout << "ImageWidth: " <<ImageWidth <<endl;
  cout << "BucketNumberX: " <<BucketNumberX <<endl;
  cout << "BucketNumberY: " <<BucketNumberY <<endl;
  cout << "StartingX: " <<StartingX <<endl;
  cout << "StartingY: " <<StartingY <<endl;*/
    //vector<vector<vector<int>>> BucketCoordinates(BucketNumberX,vector<int>(BucketNumberY,vector<int>(2,0)));
    //The coordinates of each bucket are stored in this 3d vector

    BucketsBlobMax.resize(BucketNumberX);
    BucketsBlobMin.resize(BucketNumberX);
    BucketsCornerMax.resize(BucketNumberX);
    BucketsCornerMin.resize(BucketNumberX);
    for (int ii = 0; ii < BucketNumberX; ii++)
    {
        BucketsBlobMax[ii].resize(BucketNumberY);
        BucketsBlobMin[ii].resize(BucketNumberY);
        BucketsCornerMax[ii].resize(BucketNumberY);
        BucketsCornerMin[ii].resize(BucketNumberY);
        /*for(int jj=0;jj<BucketNumberY;jj++)
    {
      BucketCoordinates[ii][jj][0] = StartingX+ii*50;//x coordinate
      BucketCoordinates[ii][jj][1] = StartingY+jj*50;//y coordinate      
    }*/
    }

    for (uint imatch = 0; imatch < InputMatches.size(); imatch++)
    {
        if (InputMatches[imatch][3].x < StartingX || InputMatches[imatch][3].y < StartingY || InputMatches[imatch][3].x >= (StartingX + BucketNumberX * BucketSize) || InputMatches[imatch][3].y >= (StartingY + BucketNumberY * BucketSize))
            /*if(InputMatches[imatch][3].y<StartingX
      ||InputMatches[imatch][3].x<StartingY
      ||InputMatches[imatch][3].y>=(StartingX+BucketNumberX*BucketSize)
      ||InputMatches[imatch][3].x>=(StartingY+BucketNumberY*BucketSize))*/
            continue;
        int BucketX = (int(InputMatches[imatch][3].x) - StartingX) / BucketSize;
        int BucketY = (int(InputMatches[imatch][3].y) - StartingY) / BucketSize;
        switch (InputMatches[imatch][3].classID)
        {
        case 0:
            BucketsBlobMax[BucketX][BucketY].push_back(InputMatches[imatch]);
            break;
        case 1:
            BucketsBlobMin[BucketX][BucketY].push_back(InputMatches[imatch]);
            break;
        case 2:
            BucketsCornerMax[BucketX][BucketY].push_back(InputMatches[imatch]);
            break;
        case 3:
            BucketsCornerMin[BucketX][BucketY].push_back(InputMatches[imatch]);
            break;
        default:
            continue;
        }
    }
    //return InputMatches;

    //cout <<"BucketsBlobMax size:"<< BucketsBlobMax[1][5].size()<<endl;
    vector<vector<SoftPoint>> SelectedMatches;
    for (int xx = 0; xx < BucketNumberX; xx++)
    {
        for (int yy = 0; yy < BucketNumberY; yy++)
        {
            //cout << "sorting BucketsBlobMax "<<xx<<" "<<yy<<endl;
            /*
      cout << "strength before sorting:";
      for(uint testii=0;testii<BucketsBlobMax[xx][yy].size();testii++)
      {
	cout<<BucketsBlobMax[xx][yy][testii][3].value;
	cout << " ";
      }
      cout << endl;*/
            SortBucket(BucketsBlobMax[xx][yy], AgeThreshold);
            /*cout << "strength after sorting:";
      for(uint testii=0;testii<BucketsBlobMax[xx][yy].size();testii++)
      {
	cout<<BucketsBlobMax[xx][yy][testii][3].value;
	 cout << " ";
      }
      cout << endl;
      */
            //cout << "sorting done" << endl;
            SortBucket(BucketsBlobMin[xx][yy], AgeThreshold);
            SortBucket(BucketsCornerMax[xx][yy], AgeThreshold);
            SortBucket(BucketsCornerMin[xx][yy], AgeThreshold);
            int SelectionCount = 0;
            for (uint ll = 0;; ll++)
            {
                if (ll < BucketsBlobMax[xx][yy].size())
                {
                    SelectedMatches.push_back(BucketsBlobMax[xx][yy][ll]);
                    SelectionCount++;
                    if (SelectionCount >= MaxFeaturesNum)
                        break;
                }
                if (ll < BucketsBlobMin[xx][yy].size())
                {
                    SelectedMatches.push_back(BucketsBlobMin[xx][yy][ll]);
                    SelectionCount++;
                    if (SelectionCount >= MaxFeaturesNum)
                        break;
                }
                if (ll < BucketsCornerMax[xx][yy].size())
                {
                    SelectedMatches.push_back(BucketsCornerMax[xx][yy][ll]);
                    SelectionCount++;
                    if (SelectionCount >= MaxFeaturesNum)
                        break;
                }
                if (ll < BucketsCornerMin[xx][yy].size())
                {
                    SelectedMatches.push_back(BucketsCornerMin[xx][yy][ll]);
                    SelectionCount++;
                    if (SelectionCount >= MaxFeaturesNum)
                        break;
                }
                if ((ll >= BucketsBlobMax[xx][yy].size()) && (ll >= BucketsBlobMin[xx][yy].size()) && (ll >= BucketsCornerMax[xx][yy].size()) && (ll >= BucketsCornerMin[xx][yy].size()))
                    break;
            }
        }
    }
    return SelectedMatches;
}

//divide the image into 50x50 buckets

//getting the features within the buckets (Only matched features from Left image at time t)

//from class0 to class3 select the strongest

//stop when selection number meets N

//save the matches according to the selections of features above (features from Left image at time t)

//return the new matches