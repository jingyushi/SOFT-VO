
#include "feature.h"

/**
 * initialize a pattern used for feature descriptor
 * @param NULL
 * @return An initial descriptor
 **/
MatchPattern InitialPattern()
{
    MatchPattern pattern;
    for (int i = 0; i < NUM_MATCH; i++)
        pattern(i) = 0;
    return pattern;
}

/**
 * initialize a pattern used for feature descriptor
 * @param val, the value to initialize the descriptor
 * @return An initial descriptor with equal values as val
 **/
MatchPattern InitialPattern(int val)
{
    MatchPattern pattern;
    for (int i = 0; i < NUM_MATCH; i++)
        pattern(i) = val;
    return pattern;
}
/**
 * A lazy version of cv::imshow
 * @param: cv::Mat to show, and its window name
 **/
void ShortImgShow(cv::Mat img, string name)
{
    cv::namedWindow(name, cv::WINDOW_AUTOSIZE);
    cv::imshow(name, img);
    cv::waitKey(0);
}

/**
 * Generate SoftPoint object for each class of points
 * @brief The code transforms the location set of feature points to the set of softPoints object given a class of points
 **/
vector<SoftPoint> GenerateFeatureObjects1Class(vector<cv::Point2f> &points, cv::Mat &filterImg, cv::Mat &sobelImgInput, int classID)
{
    cv::Mat sobelImg = sobelImgInput;
    vector<SoftPoint> softPoints;
    int rows = sobelImg.rows;
    int cols = sobelImg.cols;

    softPoints.reserve(static_cast<int>(points.size()));
    for (int index = 0; index < static_cast<int>(points.size()); index++)
    {
        MatchPattern pattern;
        if (points[index].x < 5 || points[index].y < 5 || points[index].x > cols - 5 || points[index].y > rows - 5)
        {
            pattern = InitialPattern(sobelImg.at<float>(points[index].y, points[index].x));
        }
        else
        {
            pattern << sobelImg.at<float>(points[index].y - 5, points[index].x - 1),
                sobelImg.at<float>(points[index].y - 5, points[index].x + 1),
                sobelImg.at<float>(points[index].y - 3, points[index].x - 5),
                sobelImg.at<float>(points[index].y - 3, points[index].x - 3),
                sobelImg.at<float>(points[index].y - 3, points[index].x - 1),
                sobelImg.at<float>(points[index].y - 3, points[index].x + 1),
                sobelImg.at<float>(points[index].y - 3, points[index].x + 3),
                sobelImg.at<float>(points[index].y - 3, points[index].x + 5),
                sobelImg.at<float>(points[index].y - 1, points[index].x - 3),
                sobelImg.at<float>(points[index].y - 1, points[index].x - 1),
                sobelImg.at<float>(points[index].y - 1, points[index].x + 1),
                sobelImg.at<float>(points[index].y - 1, points[index].x + 3),
                sobelImg.at<float>(points[index].y + 1, points[index].x - 3),
                sobelImg.at<float>(points[index].y + 1, points[index].x - 1),
                sobelImg.at<float>(points[index].y + 1, points[index].x + 1),
                sobelImg.at<float>(points[index].y + 1, points[index].x + 3),
                sobelImg.at<float>(points[index].y + 3, points[index].x - 5),
                sobelImg.at<float>(points[index].y + 3, points[index].x - 3),
                sobelImg.at<float>(points[index].y + 3, points[index].x - 1),
                sobelImg.at<float>(points[index].y + 3, points[index].x + 1),
                sobelImg.at<float>(points[index].y + 3, points[index].x + 3),
                sobelImg.at<float>(points[index].y - 3, points[index].x + 5),
                sobelImg.at<float>(points[index].y + 5, points[index].x - 1),
                sobelImg.at<float>(points[index].y + 5, points[index].x + 1);
        }
        if (pattern.norm() == 0)
            continue;
        SoftPoint pointTemp(points[index].x, points[index].y, filterImg.at<float>(points[index].y, points[index].x), classID, pattern);
        softPoints.push_back(pointTemp);
    }

    return softPoints;
}

/**
 * Generate SoftPoint objects for the detected points
 * @param: points: the pixel locations detected from the previous parts
 * @return: SoftPoint vectors, the first dimension is the class type, i.e., maxBlob[0], minBlob[1], maxCorner[2], minCorner[3]
 **/
vector<vector<SoftPoint>> GenerateSoftPoints(vector<vector<cv::Point2f>> &points, cv::Mat &blobImg, cv::Mat &cornerImg, cv::Mat sobelImg)
{

    vector<SoftPoint> blobMaxSoftPoints = GenerateFeatureObjects1Class(points[0], blobImg, sobelImg, 0);
    vector<SoftPoint> blobMinSoftPoints = GenerateFeatureObjects1Class(points[1], blobImg, sobelImg, 1);
    vector<SoftPoint> cornerMaxSoftPoints = GenerateFeatureObjects1Class(points[2], cornerImg, sobelImg, 2);
    vector<SoftPoint> cornerMinSoftPoints = GenerateFeatureObjects1Class(points[3], cornerImg, sobelImg, 3);

    // cout << "Feature objects generated" << endl;

    vector<vector<SoftPoint>> fourClassSoftPoint;
    fourClassSoftPoint.push_back(blobMaxSoftPoints);
    fourClassSoftPoint.push_back(blobMinSoftPoints);
    fourClassSoftPoint.push_back(cornerMaxSoftPoints);
    fourClassSoftPoint.push_back(cornerMinSoftPoints);

    return fourClassSoftPoint;
}
void featureDetectionFast(cv::Mat image, std::vector<cv::Point2f> &points)
{
    //uses FAST as for feature dection, modify parameters as necessary
    cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
    std::vector<cv::KeyPoint> keypoints;
    int fast_threshold = 20;
    bool nonmaxSuppression = true;
    cv::FAST(image, keypoints, fast_threshold, nonmaxSuppression);
    cv::KeyPoint::convert(keypoints, points, std::vector<int>());
}
/**
 * The function detects the feature points using blob/corner filter and generates SoftPoint objects of an image
 * @param: img, the image to detect feature poitns, and should be in RGB form, like CV_U8C3 rather than CV_U8C1
 * @return: The SoftPoints in four classes are stored as four dimensions, i.e., maxBlob[0], minBlob[1], maxCorner[2], minCorner[3]
 **/
vector<vector<SoftPoint>> GenerateFeatureObjects(cv::Mat &img, int nmsRange)
{
    cout << 1;
    // Generate blob/corner kernel
    cv::Mat blobKernel = (cv::Mat_<double>(5, 5) << -1, -1, -1, -1, -1, -1, 1, 1, 1, -1, -1, 1, 8, 1, -1, -1, 1, 1, 1, -1, -1, -1, -1, -1, -1);
    cv::Mat cornerKernel = (cv::Mat_<double>(5, 5) << -1, -1, 0, 1, 1, -1, -1, 0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, -1, -1, 1, 1, 0, -1, -1);
    blobKernel = blobKernel / 25.0;
    cornerKernel = cornerKernel / 25.0;

    // Generate the image after filtering
    cv::Mat blobImg;
    cv::filter2D(img, blobImg, CV_32F, blobKernel); //Generally, don't use CV_64F there
    cv::cvtColor(blobImg, blobImg, CV_BGR2GRAY);
    cv::Mat cornerImg;
    cv::filter2D(img, cornerImg, CV_32F, cornerKernel);
    cv::cvtColor(cornerImg, cornerImg, CV_BGR2GRAY);
    cv::Mat sobelImg;
    cv::Sobel(img, sobelImg, CV_32F, 1, 1, 3);
    cv::cvtColor(sobelImg, sobelImg, CV_BGR2GRAY);

    cout << 1;
    // Detect locations of the feature points
    vector<vector<cv::Point2f>> pointsLocationIn4class = NMS_fast(blobImg, cornerImg, nmsRange);

    // vector<cv::Point2f> pointsLocationIn1class;
    // featureDetectionFast(img, pointsLocationIn1class);
    // vector<vector<cv::Point2f>> pointsLocationIn4class;
    // vector<cv::Point2f> sth;
    // pointsLocationIn4class.push_back(pointsLocationIn1class);
    // pointsLocationIn4class.push_back(sth);
    // pointsLocationIn4class.push_back(sth);
    // pointsLocationIn4class.push_back(sth);

    //To test if the data type is right there, uncomment these parts
    /*
    cv::Rect roi(cv::Point(116 - 8, 61 - 8), cv::Point(116 + 8, 61 + 8));
    cout << "One part from sobel image!" << endl;
    cout << sobelImg(roi);
    cout << "End*********" << endl;
    cout << sobelImg.at<float>(61 - 8, 116 - 8) << endl;
    cout << sobelImg.at<float>(61 - 7, 116 - 8) << endl;
    */
    return GenerateSoftPoints(pointsLocationIn4class, blobImg, cornerImg, sobelImg);
}

/**
 * One part of NMS_fast, it checks if the chosen point in a box is the biggest/smallest in its surrounding boxes
 * @param: type, which could only be "max"/"min"
 * @param: i/j, the index of the current box
 * @param: boxLength, the same as the one in NMS_fast
 * @return: bool
 **/
bool CheckSurrounding(cv::Mat img, double val, string type, int i, int j, int boxLength)
{
    double maxValue = 0, minValue = 0;

    cv::Rect mask(cv::Point2f(j - boxLength, i - boxLength), cv::Point2f(j + boxLength, i + boxLength));
    cv::Mat box = img(mask);
    cv::minMaxLoc(box, &minValue, &maxValue);
    if (type == "max" && maxValue > val)
        return false;
    else if (type == "min" && minValue < val)
        return false;
    else if (type != "max" && type != "min")
        cout << "Input type error in the NMS_fast. The allowable parameters for type are only 'max' and 'min" << endl;
    return true;
}
/**
 * The function detects the feature locations based on blob and corner image, and run fast NMS to remove repeated points
 * @param: boxLength. It decides the maximum distance allowed between two points. The actual distance between all the points are bigger than boxLength
 * @return: Point locations in four the classes
 **/
vector<vector<cv::Point2f>> NMS_fast(cv::Mat blobImg, cv::Mat cornerImg, double boxLength)
{
    int width = blobImg.size().width;
    int height = blobImg.size().height;

    vector<vector<cv::Point2f>> cleanPoints;
    vector<cv::Point2f> vectorTemp;
    for (int i = 0; i < 4; i++)
        cleanPoints.push_back(vectorTemp);

    //Search over each box
    for (int i = boxLength; i < height - boxLength; i += boxLength)
    {
        for (int j = boxLength; j < width - boxLength; j += boxLength)
        {

            cv::Rect mask(cv::Point2f(j, i), cv::Point2f(j + boxLength, i + boxLength));
            cv::Mat blobBox = blobImg(mask);
            cv::Mat cornerBox = cornerImg(mask);

            double blobMinValue, blobMaxValue, cornerMinValue, cornerMaxValue;
            cv::Point blobMaxIndex, blobMinIndex, cornerMaxIndex, cornerMinIndex;
            cv::minMaxLoc(blobBox, &blobMinValue, &blobMaxValue, &blobMinIndex, &blobMaxIndex);
            cv::minMaxLoc(cornerBox, &cornerMinValue, &cornerMaxValue, &cornerMinIndex, &cornerMaxIndex);

            if (CheckSurrounding(blobImg, blobMaxValue, "max", i, j, boxLength))
            {
                blobMaxIndex.y += i;
                blobMaxIndex.x += j;
                cleanPoints[0].push_back(blobMaxIndex);
            }
            if (CheckSurrounding(blobImg, blobMinValue, "min", i, j, boxLength))
            {
                blobMinIndex.y += i;
                blobMinIndex.x += j;
                cleanPoints[1].push_back(blobMinIndex);
            }
            if (CheckSurrounding(cornerImg, cornerMaxValue, "max", i, j, boxLength))
            {
                cornerMaxIndex.y += i;
                cornerMaxIndex.x += j;
                cleanPoints[2].push_back(cornerMaxIndex);
            }
            if (CheckSurrounding(cornerImg, cornerMinValue, "min", i, j, boxLength))
            {
                cornerMinIndex.y += i;
                cornerMinIndex.x += j;
                cleanPoints[3].push_back(cornerMinIndex);
            }
        }
    }
    return cleanPoints;
}

/**
 * Draws all the feature poitns of 4 classes in an image
 **/
void DrawFeaturePoints(cv::Mat img, vector<vector<cv::Point2f>> points4Class, int radius)
{
    for (auto points : points4Class)
        for (auto point : points)
            cv::circle(img, cv::Point(point.x, point.y), radius, cv::Scalar(0, 0, 255));
    ShortImgShow(img);
}
/**
 * Draws all the feature poitns of 4 classes in an image
 **/
void DrawFeaturePoints(cv::Mat img, vector<vector<SoftPoint>> points4Class, int radius)
{
    for (auto points : points4Class)
        for (auto point : points)
            cv::circle(img, cv::Point(point.x, point.y), radius, cv::Scalar(0, 0, 255));
    ShortImgShow(img);
}

/**
 * Draw one point on an image and show it
 **/
void DrawFeaturePoints(cv::Mat img, SoftPoint point, int radius)
{

    cv::circle(img, cv::Point(point.x, point.y), radius, cv::Scalar(0, 0, 255));

    ShortImgShow(img);
}
/**
 * Transform SoftPoint objects to Opencv Point2
 **/
vector<vector<cv::Point2f>> SoftP2CVP(vector<vector<SoftPoint>> softPoints4Frame)
{
    vector<cv::Point2f> pointsLeftOld, pointsLeftNew, pointsRightOld, pointsRightNew;
    pointsLeftOld.reserve(softPoints4Frame.size());
    pointsLeftNew.reserve(softPoints4Frame.size());
    pointsRightOld.reserve(softPoints4Frame.size());
    pointsRightNew.reserve(softPoints4Frame.size());
    for (int i = 0; i < static_cast<int>(softPoints4Frame.size()); i++)
    {
        pointsLeftOld.push_back(cv::Point2f(softPoints4Frame[i][0].x, softPoints4Frame[i][0].y));
        pointsRightOld.push_back(cv::Point2f(softPoints4Frame[i][1].x, softPoints4Frame[i][1].y));
        pointsRightNew.push_back(cv::Point2f(softPoints4Frame[i][2].x, softPoints4Frame[i][2].y));
        pointsLeftNew.push_back(cv::Point2f(softPoints4Frame[i][3].x, softPoints4Frame[i][3].y));
    }

    vector<vector<cv::Point2f>> temp;
    temp.push_back(pointsLeftOld);
    temp.push_back(pointsRightOld);
    temp.push_back(pointsRightNew);
    temp.push_back(pointsLeftNew);
    return temp;
}