#ifndef FEATURE_H
#define FEATURE_H

#include <vector>

#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <Eigen/Dense>

#include "soft_slam_io.h"

using namespace std;

//following left->right up->down order for 24 points
const int NUM_MATCH = 24;
typedef Eigen::Matrix<double, 1, NUM_MATCH> MatchPattern;

MatchPattern InitialPattern();
MatchPattern InitialPattern(int val);
// void ShortImgShow(cv::Mat img, string name);

void ShortImgShow(cv::Mat img, string name = "windows");

struct SoftPoint
{
public:
    //DataMembers:
    double x;
    double y;
    int classID; //0-maxBlob, 1-minBlob, 2-maxCorner, 3-minCorner
    double value;
    int age;
    MatchPattern pattern;

    SoftPoint() : x(0), y(0), classID(0), value(0), age(0) { pattern = InitialPattern(); }
    SoftPoint(double x, double y) : x(x), y(y), classID(0), value(0), age(0) { pattern = InitialPattern(); }
    SoftPoint(double x, double y, double value) : x(x), y(y), classID(0), value(value), age(0) { pattern = InitialPattern(); }
    SoftPoint(double x, double y, double value, int classID) : x(x), y(y), classID(classID), value(value), age(0) { pattern = InitialPattern(); }
    SoftPoint(double x, double y, double value, int classID, MatchPattern &inputPattern) : x(x), y(y), classID(classID), value(value), age(0), pattern(inputPattern) {}
    SoftPoint &operator=(const SoftPoint &p)
    {
        // cout << "copy assignment is called" << endl;
        x = p.x;
        y = p.y;
        pattern = p.pattern;
        value = p.value;
        classID = p.classID;
        age = p.age;
        return *this;
    }
    SoftPoint(const SoftPoint &p)
    {
        x = p.x;
        y = p.y;
        pattern = p.pattern;
        value = p.value;
        classID = p.classID;
        age = p.age;
        // cout << "Copy constructor is called" << endl;
    }
    void show()
    {
        cout << "Location: " << x << ", " << y << ", "
             << "Value: " << value << ", "
             << "Class: " << classID << ", "
             << "age: " << age << endl
             << pattern << endl;
    }
};

// vector<SoftPoint> GenerateInitialDescriptor(vector<cv::Point2f> points, cv::Mat img, int classID);

vector<vector<SoftPoint>> GenerateFeatureObjects(cv::Mat &img, int nmsRange = 40);

vector<vector<SoftPoint>> GenerateSoftPoints(vector<vector<cv::Point2f>> &points, cv::Mat &blobImg, cv::Mat &cornerImg, cv::Mat sobelImg);

vector<SoftPoint> GenerateFeatureObjects1Class(vector<cv::Point2f> &points, cv::Mat &filterImg, cv::Mat &sobelImgInput, int classID);

vector<vector<cv::Point2f>> FindFeatureLocation(cv::Mat &blobImg, cv::Mat &cornerImg, double range = 30, string type = "max");

void DrawFeaturePoints(cv::Mat img, SoftPoint point, int radius = 5);
void DrawFeaturePoints(cv::Mat img, vector<vector<cv::Point2f>> points, int radius = 5);
void DrawFeaturePoints(cv::Mat img, vector<vector<SoftPoint>> points4Class, int radius = 5);

vector<vector<cv::Point2f>> NMS_fast(cv::Mat blobImg, cv::Mat cornerImg, double boxLength = 40);

bool CheckSurrounding(cv::Mat img, double val, string type, int i, int j, int boxLength);

vector<vector<cv::Point2f>> SoftP2CVP(vector<vector<SoftPoint>> softPoints4Class);

//void featureDetectionFast(cv::Mat image, std::vector<cv::Point2f> &points);
#endif
