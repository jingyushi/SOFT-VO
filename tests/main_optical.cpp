
/**
 * This file is used for the stereo version of KITTI dataset
 * */

#include "soft_slam_io.h"
#include "features.h"
#include "matching.h"
#include "config.h"

#include "feature_selection.h"

void trackingFrame2Frame(cv::Mat &projMatrl, cv::Mat &projMatrr,
                         std::vector<cv::Point2f> &pointsLeft_t0,
                         std::vector<cv::Point2f> &pointsLeft_t1,
                         cv::Mat &points3D_t0,
                         cv::Mat &rotation,
                         cv::Mat &translation)
{

    // Calculate frame to frame transformation

    // -----------------------------------------------------------
    // Rotation(R) estimation using Nister's Five Points Algorithm
    // -----------------------------------------------------------
    double focal = projMatrl.at<float>(0, 0);
    cv::Point2d principle_point(projMatrl.at<float>(0, 2), projMatrl.at<float>(1, 2));

    //recovering the pose and the essential cv::matrix
    cv::Mat E, mask;
    cv::Mat translation_mono = cv::Mat::zeros(3, 1, CV_64F);
    E = cv::findEssentialMat(pointsLeft_t1, pointsLeft_t0, focal, principle_point, cv::RANSAC, 0.999, 1.0, mask);
    cv::recoverPose(E, pointsLeft_t1, pointsLeft_t0, rotation, translation_mono, focal, principle_point, mask);
    // cout << mask << endl;
    // std::cout << "recoverPose rotation: " << rotation << std::endl;

    // ------------------------------------------------
    // Translation (t) estimation by use solvePnPRansac
    // ------------------------------------------------
    cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64FC1);
    cv::Mat inliers;
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::Mat intrinsic_matrix = (cv::Mat_<float>(3, 3) << projMatrl.at<float>(0, 0), projMatrl.at<float>(0, 1), projMatrl.at<float>(0, 2),
                                projMatrl.at<float>(1, 0), projMatrl.at<float>(1, 1), projMatrl.at<float>(1, 2),
                                projMatrl.at<float>(1, 1), projMatrl.at<float>(1, 2), projMatrl.at<float>(1, 3));

    int iterationsCount = 500;     // number of Ransac iterations.
    float reprojectionError = 2.0; // maximum allowed distance to consider it an inlier.
    float confidence = 0.95;       // RANSAC successful confidence.
    bool useExtrinsicGuess = true;
    int flags = cv::SOLVEPNP_ITERATIVE;

    cv::solvePnPRansac(points3D_t0, pointsLeft_t1, intrinsic_matrix, distCoeffs, rvec, translation,
                       useExtrinsicGuess, iterationsCount, reprojectionError, confidence,
                       inliers, flags);

    translation = -translation;
    // std::cout << "inliers size: " << inliers.size() << std::endl;
}

void integrateOdometryStereo(int frame_i, cv::Mat &rigid_body_transformation, cv::Mat &frame_pose, const cv::Mat &rotation, const cv::Mat &translation_stereo)
{

    // std::cout << "rotation" << rotation << std::endl;
    // std::cout << "translation_stereo" << translation_stereo << std::endl;

    cv::Mat addup = (cv::Mat_<double>(1, 4) << 0, 0, 0, 1);

    cv::hconcat(rotation, translation_stereo, rigid_body_transformation);
    cv::vconcat(rigid_body_transformation, addup, rigid_body_transformation);

    // std::cout << "rigid_body_transformation" << rigid_body_transformation << std::endl;

    double scale = sqrt((translation_stereo.at<double>(0)) * (translation_stereo.at<double>(0)) + (translation_stereo.at<double>(1)) * (translation_stereo.at<double>(1)) + (translation_stereo.at<double>(2)) * (translation_stereo.at<double>(2)));

    // frame_pose = frame_pose * rigid_body_transformation;
    std::cout << "scale: " << scale << std::endl;

    // rigid_body_transformation = rigid_body_transformation.inv();
    // if ((scale>0.1)&&(translation_stereo.at<double>(2) > translation_stereo.at<double>(0)) && (translation_stereo.at<double>(2) > translation_stereo.at<double>(1)))
    if (scale > 0.05 && scale < 10)
    {
        // std::cout << "Rpose" << Rpose << std::endl;

        frame_pose = frame_pose * rigid_body_transformation;
    }
    else
    {
        std::cout << "[WARNING] scale below 0.1, or incorrect translation" << std::endl;
    }
}

void deleteUnmatchFeaturesCircle(std::vector<cv::Point2f> &points0, std::vector<cv::Point2f> &points1,
                                 std::vector<cv::Point2f> &points2, std::vector<cv::Point2f> &points3,
                                 std::vector<cv::Point2f> &points0_return,
                                 std::vector<uchar> &status0, std::vector<uchar> &status1,
                                 std::vector<uchar> &status2, std::vector<uchar> &status3)
{
    //getting rid of points for which the KLT tracking failed or those who have gone outside the frame

    int indexCorrection = 0;
    for (int i = 0; i < status3.size(); i++)
    {
        cv::Point2f pt0 = points0.at(i - indexCorrection);
        cv::Point2f pt1 = points1.at(i - indexCorrection);
        cv::Point2f pt2 = points2.at(i - indexCorrection);
        cv::Point2f pt3 = points3.at(i - indexCorrection);
        cv::Point2f pt0_r = points0_return.at(i - indexCorrection);

        if ((status3.at(i) == 0) || (pt3.x < 0) || (pt3.y < 0) ||
            (status2.at(i) == 0) || (pt2.x < 0) || (pt2.y < 0) ||
            (status1.at(i) == 0) || (pt1.x < 0) || (pt1.y < 0) ||
            (status0.at(i) == 0) || (pt0.x < 0) || (pt0.y < 0))
        {
            if ((pt0.x < 0) || (pt0.y < 0) || (pt1.x < 0) || (pt1.y < 0) || (pt2.x < 0) || (pt2.y < 0) || (pt3.x < 0) || (pt3.y < 0))
            {
                status3.at(i) = 0;
            }
            points0.erase(points0.begin() + (i - indexCorrection));
            points1.erase(points1.begin() + (i - indexCorrection));
            points2.erase(points2.begin() + (i - indexCorrection));
            points3.erase(points3.begin() + (i - indexCorrection));
            points0_return.erase(points0_return.begin() + (i - indexCorrection));

            indexCorrection++;
        }
    }
}

void checkValidMatch(std::vector<cv::Point2f> &points, std::vector<cv::Point2f> &points_return, std::vector<bool> &status, int threshold)
{
    int offset;
    for (uint i = 0; i < points.size(); i++)
    {
        offset = std::max(std::abs(points[i].x - points_return[i].x), std::abs(points[i].y - points_return[i].y));
        // std::cout << offset << ", ";

        if (offset > threshold)
        {
            status.push_back(false);
        }
        else
        {
            status.push_back(true);
        }
    }
}

void removeInvalidPoints(std::vector<cv::Point2f> &points, const std::vector<bool> &status)
{
    int index = 0;
    for (int i = 0; i < status.size(); i++)
    {
        if (status[i] == false)
        {
            points.erase(points.begin() + index);
        }
        else
        {
            index++;
        }
    }
}

vector<cv::Point2f> Soft2CVP(vector<SoftPoint> inputSoftSeq)
{
    vector<cv::Point2f> cvPoints;
    cvPoints.reserve(inputSoftSeq.size());
    for (uint i = 0; i < inputSoftSeq.size(); i++)
    {
        cvPoints.push_back(cv::Point2f(inputSoftSeq[i].x, inputSoftSeq[i].y));
    }
    return cvPoints;
}

int main(int argc, char **argv)
/**Arguments: 
 * argv[1]: sequence path
 * argv[2]: vocabulary
 * argv[3]: yaml
 **/
{
    //    if (argc != 2)
    //    {
    //        // cerr << endl
    //        //      << "Usage: ./stereo_kitti path_to_vocabulary path_to_settings path_to_sequence" << endl;
    //        cerr << endl
    //             << "Usage: ./main path_to_sequence" << endl;
    //        cout << "E.g.    /home/zephyr/Data/KITTI/data_odometry_gray/dataset/sequences/00" << endl;
    //
    //        return 1;
    //    }

    //*******************************************************************

    string pathCamera = "/home/zephyr/Data/KITTI/data_odometry_gray/dataset/sequences/00/calib.txt";
    vector<CameraConfig> cameraProjection = ReadCameraConfig<CameraConfig>(pathCamera);
    double focal = cameraProjection[0](0);
    cv::Point2d pp(cameraProjection[0](2), cameraProjection[0](6));

    vector<cv::Mat> projections = ReadProjectionMatrix<CameraConfig>(pathCamera);
    cv::Mat projectionMatrixLeft = projections[0];
    cv::Mat projectionMatrixRight = projections[1];

    // vector<cv::Mat> sth = ReadProjectionMatrix<CameraConfig>(pathCamera);

    // cout << "Projection matrix for left and right images: " << endl;
    // cout << sth[0] << endl
    //      << sth[1] << endl;

    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;
    string pathDataset = "/home/zephyr/Data/KITTI/data_odometry_gray/dataset/sequences/00";
    LoadImages(string(pathDataset), vstrImageLeft, vstrImageRight, vTimestamps);

    int nmsRange = (int)ConfigParameters["nmsRange"];
    int binSize = (int)ConfigParameters["binSize"];
    int radiusMatch = (int)ConfigParameters["radiusMatch"];
    double matchDispTolerance = (double)ConfigParameters["matchDispTolerance"];
    int nccWindow = (int)ConfigParameters["nccWindow"];
    double nccTolerance = (double)ConfigParameters["nccTolerance"]; //should be close to 1
    double patternDiffTol = (double)ConfigParameters["patternDiffTol"];
    double dist_tol = (double)ConfigParameters["dist_tol"];
    cv::Mat imgTemp;
    imgTemp = cv::imread(vstrImageLeft[0], 1);
    int xBinNum = ceil(imgTemp.size().width / (double)binSize);
    int yBinNum = ceil(imgTemp.size().height / (double)binSize);

    MatchParameters matchParameters(imgTemp.size().width, imgTemp.size().height, binSize, radiusMatch, matchDispTolerance,
                                    nccWindow, nccTolerance, xBinNum, yBinNum, dist_tol, patternDiffTol);

    cv::Mat bigMapToShow = cv::Mat::zeros(600, 600, CV_8UC3);
    char text[100];
    int fontFace = cv::FONT_HERSHEY_PLAIN;
    double fontScale = 1;
    int thickness = 1;
    cv::Point textOrg(10, 50);
    cv::Mat rotation = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat translation_stereo = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat rotationCurrent = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat translationCurrent = cv::Mat::zeros(3, 1, CV_64F);
    for (int indexFrame = 0; indexFrame < 900; indexFrame++) //read each frames
    {

        // ---------------- Read Frames ----------------
        cv::Mat imgLeftOld, imgRightOld, imgLeftNew, imgRightNew;
        // int ni = 0;
        imgLeftOld = cv::imread(vstrImageLeft[indexFrame], 1);
        imgRightOld = cv::imread(vstrImageRight[indexFrame], 1);

        imgLeftNew = cv::imread(vstrImageLeft[indexFrame + 1], 1);
        imgRightNew = cv::imread(vstrImageRight[indexFrame + 1], 1);
        // cout << 1 << endl;
        // ---------------- Feature detection ----------------
        vector<vector<SoftPoint>> softPointsLeftOld = GenerateFeatureObjects(imgLeftOld, nmsRange);
        // vector<vector<SoftPoint>> softPointsLeftNew = GenerateFeatureObjects(imgLeftNew, nmsRange);
        // vector<vector<SoftPoint>> softPointsRightOld = GenerateFeatureObjects(imgRightOld, nmsRange);
        // vector<vector<SoftPoint>> softPointsRightNew = GenerateFeatureObjects(imgRightNew, nmsRange);
        vector<cv::Point2f> cvPointsLeftNew;
        vector<cv::Point2f> cvPointsRightOld;
        vector<cv::Point2f> cvPointsRightNew;
        vector<cv::Point2f> cvPointsLeftOld = Soft2CVP(softPointsLeftOld[0]);
        vector<cv::Point2f> cvPointsReturn;

        // ---------------- Feature matching ----------------
        std::vector<uchar> status0;
        std::vector<uchar> status1;
        std::vector<uchar> status2;
        std::vector<uchar> status3;
        std::vector<float> err;
        cv::Size winSize = cv::Size(21, 21);
        cv::TermCriteria termcrit = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);
        vector<cv::Point2f> pointsResult;
        for (int i = 0; i < 4; i++)
        {
            calcOpticalFlowPyrLK(imgLeftOld, imgRightOld, cvPointsLeftOld, cvPointsRightOld,
                                 status0, err, winSize, 3, termcrit, 0, 0.001);
            calcOpticalFlowPyrLK(imgRightOld, imgRightNew, cvPointsRightOld, cvPointsRightNew,
                                 status1, err, winSize, 3, termcrit, 0, 0.001);
            calcOpticalFlowPyrLK(imgRightNew, imgLeftNew, cvPointsRightNew, cvPointsLeftNew,
                                 status2, err, winSize, 3, termcrit, 0, 0.001);
            calcOpticalFlowPyrLK(imgLeftNew, imgLeftOld, cvPointsLeftNew, cvPointsReturn,
                                 status3, err, winSize, 3, termcrit, 0, 0.001);
        }
        deleteUnmatchFeaturesCircle(cvPointsLeftOld, cvPointsRightOld, cvPointsRightNew, cvPointsLeftNew, cvPointsReturn,
                                    status0, status1, status2, status3);

        std::vector<bool> status;
        checkValidMatch(cvPointsLeftOld, cvPointsReturn, status, 0);

        removeInvalidPoints(cvPointsLeftOld, status);
        removeInvalidPoints(cvPointsRightNew, status);
        removeInvalidPoints(cvPointsLeftNew, status);
        removeInvalidPoints(cvPointsRightOld, status);

        // circularMatchingStan(imgLeftOld, imgRightOld, imgLeftNew, imgRightNew,
        // softPointsLeftOld, softPointsRightOld, softPointsLeftNew, softPointsRightNew,
        // pointsResult, )
        // vector<cv::Mat> imgs;
        // imgs.push_back(imgLeftOld);
        // imgs.push_back(imgRightOld);
        // imgs.push_back(imgRightNew);
        // imgs.push_back(imgLeftNew);

        // NCC_Check(matchedPoints, imgLeftOld,
        //           imgRightOld, imgRightNew,
        //           imgLeftNew, matchParameters);

        // matchedPoints = SelectFeatures(imgLeftNew, matchedPoints, 50, 4);

        // ---------------- Finding rotation and translation ----------------

        //vector<vector<cv::Point2f>> matchesInCV = SoftP2CVP(matchedPoints);

        //cv::Mat E, R, t, mask;
        //E = cv::findEssentialMat(matchesInCV[3], matchesInCV[0], focal, pp, cv::RANSAC, 0.99, 0.5, mask);
        //cv::recoverPose(E, matchesInCV[3], matchesInCV[0], R, t, focal, pp, mask);
        // cout << cvPointsLeftOld << endl;

        // Triangulate points
        cv::Mat points4DOld;
        // cv::triangulatePoints(projectionMatrixLeft, projectionMatrixRight, matchesInCV[0], matchesInCV[1], points4DOld);
        // cv::Mat points3DOld;
        // cv::convertPointsFromHomogeneous(points4DOld.reshape(4, points4DOld.size[1]), points3DOld);
        // // cout << "Points after triangulation: " << points3DOld << endl;
        // trackingFrame2Frame(projectionMatrixLeft, projectionMatrixRight, matchesInCV[0], matchesInCV[3],
        //                     points3DOld, rotation, translation_stereo);
        cv::triangulatePoints(projectionMatrixLeft, projectionMatrixRight, cvPointsLeftOld, cvPointsRightOld, points4DOld);
        cv::Mat points3DOld;
        cv::convertPointsFromHomogeneous(points4DOld.t(), points3DOld);
        // cout << points3DOld << endl;
        // cv::convertPointsFromHomogeneous(points4DOld.t(), points3DOld);
        // cout << "Points after triangulation: " << points3DOld << endl;

        trackingFrame2Frame(projectionMatrixLeft, projectionMatrixRight, cvPointsLeftOld, cvPointsLeftNew,
                            points3DOld, rotation, translation_stereo);

        translationCurrent = rotationCurrent * translation_stereo + translationCurrent;
        rotationCurrent = rotationCurrent * rotation;
        cout << "Translation vector: " << translation_stereo << endl;
        cout << "*******" << endl;
        cout << "Translation: " << translationCurrent << endl;
        cout << "*******" << endl;
        cout << "The number of matched pairs found: " << cvPointsLeftOld.size() << endl;
        cout << "*******" << endl;
        cout << "Rotation: " << rotationCurrent << endl;
        cout << "*******" << endl;
        // cv::Vec3f rotation_euler = rotationMatrixToEulerAngles(rotation);

        // clock_t toc = clock();
        // fps = float(frame_id - init_frame_id) / (toc - tic) * CLOCKS_PER_SEC;

        // std::cout << "Pose" << pose.t() << std::endl;
        // std::cout << "FPS: " << fps << std::endl;

        // ---------------- Show the map built from the current results ----------------

        // cout << "rotation for the current frame: " << rotationCurrent << endl;
        // cout << rotationCurrent << endl;

        int x = int(translationCurrent.at<double>(0)) + 300;
        int y = int(translationCurrent.at<double>(2)) + 100;
        cv::circle(bigMapToShow, cv::Point(x, y), 1, CV_RGB(255, 0, 0), 2);

        cv::rectangle(bigMapToShow, cv::Point(10, 30), cv::Point(550, 50), CV_RGB(0, 0, 0), CV_FILLED);
        sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", translationCurrent.at<double>(0),
                translationCurrent.at<double>(1), translationCurrent.at<double>(2));
        cv::putText(bigMapToShow, text, textOrg, fontFace, fontScale, cv::Scalar::all(255), thickness, 8);

        int r = imgLeftOld.rows;
        int c = imgLeftOld.cols;

        cv::Mat cat;
        cv::vconcat(imgLeftOld, imgLeftNew, cat);
        for (uint i = 0; i < cvPointsLeftOld.size(); i++)
        {
            int currx1, curry1, currx2, curry2;
            currx1 = cvPointsLeftOld[i].x;
            currx2 = cvPointsLeftNew[i].x;
            curry1 = cvPointsLeftOld[i].y;
            curry2 = cvPointsLeftNew[i].y + r;
            cv::circle(cat, cvPointsLeftOld[i], 3, cv::Scalar(0, 255, 0), -1, -1);
            cv::circle(cat, cv::Point2f(currx2, curry2), 3, cv::Scalar(0, 255, 0), -1, -1);
            if (i % 1 == 0)
            {
                cv::line(cat, cvPointsLeftOld[i], cv::Point2f(currx2, curry2), cv::Scalar(255, 0, 0), 1);
            }
            cv::namedWindow("concat", cv::WINDOW_AUTOSIZE);
            cv::imshow("concat", cat);
        }
        cv::waitKey(0);
        // cv::imshow("Road facing camera", currImage_c);
        cv::namedWindow("Trajectory", cv::WINDOW_AUTOSIZE);
        cv::imshow("Trajectory", bigMapToShow);

        cv::waitKey(1);
        cout << "One iteration ends" << endl;
    }

    return 0;
}
