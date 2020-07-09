
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
    cout << mask << endl;
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

    cv::solvePnPRansac(points3D_t0, pointsLeft_t0, intrinsic_matrix, distCoeffs, rvec, translation,
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
    for (int indexFrame = 0; indexFrame < 10; indexFrame++) //read each frames
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
        vector<vector<SoftPoint>> softPointsLeftNew = GenerateFeatureObjects(imgLeftNew, nmsRange);
        vector<vector<SoftPoint>> softPointsRightOld = GenerateFeatureObjects(imgRightOld, nmsRange);
        vector<vector<SoftPoint>> softPointsRightNew = GenerateFeatureObjects(imgRightNew, nmsRange);

        // ---------------- Feature matching ----------------
        vector<vector<SoftPoint>> matchedPoints = CircularMatch(softPointsLeftOld,
                                                                softPointsLeftNew, softPointsRightOld,
                                                                softPointsRightNew, matchParameters);
        vector<cv::Mat> imgs;
        imgs.push_back(imgLeftOld);
        imgs.push_back(imgRightOld);
        imgs.push_back(imgRightNew);
        imgs.push_back(imgLeftNew);

        NCC_Check(matchedPoints, imgLeftOld,
                  imgRightOld, imgRightNew,
                  imgLeftNew, matchParameters);

        // matchedPoints = SelectFeatures(imgLeftNew, matchedPoints, 50, 4);

        // ---------------- Finding rotation and translation ----------------

        vector<vector<cv::Point2f>> matchesInCV = SoftP2CVP(matchedPoints);

        //cv::Mat E, R, t, mask;
        //E = cv::findEssentialMat(matchesInCV[3], matchesInCV[0], focal, pp, cv::RANSAC, 0.99, 0.5, mask);
        //cv::recoverPose(E, matchesInCV[3], matchesInCV[0], R, t, focal, pp, mask);

        // Triangulate points
        cv::Mat points4DOld;
        cv::triangulatePoints(projectionMatrixLeft, projectionMatrixRight, matchesInCV[0], matchesInCV[1], points4DOld);
        cv::Mat points3DOld;
        cv::convertPointsFromHomogeneous(points4DOld.reshape(4, points4DOld.size[1]), points3DOld);
        // cout << "Points after triangulation: " << points3DOld << endl;
        trackingFrame2Frame(projectionMatrixLeft, projectionMatrixRight, matchesInCV[0], matchesInCV[3],
                            points3DOld, rotation, translation_stereo);
        cout << "Translation vector: " << translation_stereo << endl;
        cout << "*******" << endl;
        cout << "The number of matched pairs found: " << matchedPoints.size() << endl;
        cout << "*******" << endl;
        // cv::Vec3f rotation_euler = rotationMatrixToEulerAngles(rotation);

        // clock_t toc = clock();
        // fps = float(frame_id - init_frame_id) / (toc - tic) * CLOCKS_PER_SEC;

        // std::cout << "Pose" << pose.t() << std::endl;
        // std::cout << "FPS: " << fps << std::endl;

        // ---------------- Show the map built from the current results ----------------

        translationCurrent = rotationCurrent * translation_stereo + translationCurrent;
        rotationCurrent = rotationCurrent * rotation;
        cout << rotationCurrent << endl;

        int x = int(translationCurrent.at<double>(0)) + 300;
        int y = int(translationCurrent.at<double>(2)) + 500;
        cv::circle(bigMapToShow, cv::Point(x, y), 1, CV_RGB(255, 0, 0), 2);

        cv::rectangle(bigMapToShow, cv::Point(10, 30), cv::Point(550, 50), CV_RGB(0, 0, 0), CV_FILLED);
        sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", translationCurrent.at<double>(0),
                translationCurrent.at<double>(1), translationCurrent.at<double>(2));
        cv::putText(bigMapToShow, text, textOrg, fontFace, fontScale, cv::Scalar::all(255), thickness, 8);

        // cv::imshow("Road facing camera", currImage_c);
        cv::imshow("Trajectory", bigMapToShow);

        cv::waitKey(1);
        cout << "One iteration ends" << endl;
    }

    return 0;
}