#ifndef SOFT_SLAM_IO_H
#define SOFT_SLAM_IO_H

#include <iostream>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>

#include <string>

typedef Eigen::Matrix<double, 12, 1> PoseT;
typedef Eigen::Matrix<double, 12, 1> CameraConfig;

using namespace std;
void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);

template <typename T>
vector<T> ReadPoseGroundTruth(string path)
{
    ifstream infile(path);
    vector<T> poseVec;

    string line;
    string token;
    string delimiter = " ";
    size_t pos = 0;

    int line_num = 0;
    int inner_index = 0;

    if (infile.is_open())
    {
        cout << "The PoseT file opened!" << endl;
        while (getline(infile, line))
        {
            T pose_temp;
            while ((pos = line.find(delimiter)) != string::npos)
            {
                token = line.substr(0, pos);
                float temp = atof(token.c_str()); // Transform PoseT from
                                                  // 1024-scale to 128-scale
                pose_temp(inner_index) = temp;
                line.erase(0, pos + delimiter.length());
                inner_index++;
            }

            poseVec.push_back(pose_temp);

            line_num++;
            inner_index = 0;
        }
    }
    else
    {
        throw std::invalid_argument("Can't open the PoseT file " +
                                    path);
    }

    cout << "Finished reading ground truth pose file; read "
         << poseVec.size() << " poses." << endl;
    return poseVec;
}

template <class T>
vector<T> ReadCameraConfig(string path)
{
    ifstream infile(path);
    vector<T> poseVec;

    string line;
    string token;
    string delimiter = " ";
    size_t pos = 0;

    int line_num = 0;
    int inner_index = 0;

    if (infile.is_open())
    {
        cout << "The file opened!" << endl;
        while (getline(infile, line))
        {
            if (line[0] == 'P')
            {
                line = line.erase(0, 4);
            }

            T pose_temp;
            while ((pos = line.find(delimiter)) != string::npos)
            {
                token = line.substr(0, pos);
                float temp = atof(token.c_str()); // Transform PoseT from
                                                  // 1024-scale to 128-scale
                pose_temp(inner_index) = temp;
                line.erase(0, pos + delimiter.length());
                inner_index++;
            }

            poseVec.push_back(pose_temp);

            line_num++;
            inner_index = 0;
        }
    }
    else
    {
        throw std::invalid_argument("Can't open the cameraConfig file " +
                                    path);
    }

    cout << "Finished reading cameraConfig file; read "
         << poseVec.size() << " configurations." << endl;
    return poseVec;
}

template <class T>
vector<cv::Mat> ReadProjectionMatrix(string path)
{
    ifstream infile(path);
    vector<T> poseVec;

    string line;
    string token;
    string delimiter = " ";
    size_t pos = 0;

    int line_num = 0;
    int inner_index = 0;

    if (infile.is_open())
    {
        cout << "The file opened!" << endl;
        while (getline(infile, line))
        {
            if (line[0] == 'P')
            {
                line = line.erase(0, 4);
            }

            T pose_temp;
            while ((pos = line.find(delimiter)) != string::npos)
            {
                token = line.substr(0, pos);
                float temp = atof(token.c_str()); // Transform PoseT from
                                                  // 1024-scale to 128-scale
                pose_temp(inner_index) = temp;
                line.erase(0, pos + delimiter.length());
                inner_index++;
            }

            poseVec.push_back(pose_temp);

            line_num++;
            inner_index = 0;
        }
    }
    else
    {
        throw std::invalid_argument("Can't open the cameraConfig file " +
                                    path);
    }

    cv::Mat leftProjectMatrix = cv::Mat::zeros(3, 4, CV_32F);
    cv::Mat rightProjectMatrix = cv::Mat::zeros(3, 4, CV_32F);

    for (int i = 0; i < 12; i++)
    {
        leftProjectMatrix.at<float>(i / 4, i % 4) = poseVec[0](i);
        rightProjectMatrix.at<float>(i / 4, i % 4) = poseVec[1](i);
    }
    vector<cv::Mat> leftAndRight;
    leftAndRight.push_back(leftProjectMatrix);
    leftAndRight.push_back(rightProjectMatrix);

    cout << "Finished reading cameraConfig file; read "
         << poseVec.size() << " configurations." << endl;
    return leftAndRight;
}

#endif