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

#ifndef FEATURE_SELECTION_H
#define FEATURE_SELECTION_H

/**

 * @file feature_selection.h

 * @author bill

 * @version 0.1

 * @date 2019-10-21

 * @brief The headfile of feature_selection part

 * @details This part takes in the output feature matches from the tracking part and select among the matches according to some criterions

 **/

#include <vector>

#include "feature.h"
#include <Eigen/Eigen>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.hpp>

//typedef vector<vector<vector<vector<SoftPoint>>>> Bucket;

//Bucket GenerateBuckets(cv::Mat Img, vector<vector<SoftPoint>> &InputMatches, int BucketSize=50);

bool SortBucket(vector<vector<SoftPoint>> &InputBucket, int AgeThreshold);

vector<vector<SoftPoint>> SelectFeatures(cv::Mat Img, vector<vector<SoftPoint>> &InputMatches, int BucketSize, int MaxFeaturesNum);

#endif // FEATURE_SELECTION_H
