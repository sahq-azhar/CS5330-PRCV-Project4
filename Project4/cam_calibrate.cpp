/*
Syed Azhar Hussain Quadri

CS 5330----Project 4
*/

#include "cam_calibrate.h"
#include "csv_util.h"

/*
 Given a cv::Mat of the image frame, cv::Mat for the output and vector of points
This function that takes in an image frame, an output matrix, and a vector of points. 
The function detects the corners present in a checkerboard grid in the image frame, 
draws them on the output matrix, and populates the vector with the coordinates of the detected corners.
 */
bool extractCorners(cv::Mat& src, cv::Mat& dst, std::vector<cv::Point2f>& corners, bool drawCorners)
{
    dst = src.clone();

    bool found = cv::findChessboardCorners(src, cv::Size(9, 6), corners);

    //std::cout << "No. of corners detected:- " << corners.size() << std::endl;
    //std::cout << "Co-ordinate of top left corner:- " << corners[0].x << " " << corners[0].y << std::endl;

    cv::Mat gray;
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);

    if (found == true) {
        cv::cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.1));
    }

    if (drawCorners) {
        cv::drawChessboardCorners(dst, cv::Size(6, 9), corners, found);
    }

    return(found);
}



/*
 This function that takes in a vector of points representing the image pixel coordinates of detected corners. 
 The function converts these points to their corresponding world coordinates for a checkerboard target, and populates the resulting vector with these world coordinates. 
 The function also populates additional vectors to store multiple sets of points and corners, which will be used in calibration. 
 */
int selectCalibration(std::vector<cv::Point2f>& corners, std::vector<std::vector<cv::Point2f>>& corners_list, std::vector<cv::Vec3f>& points, std::vector<std::vector<cv::Vec3f>>& points_list)
{
    const int cols = 9;
    const int num_corners = corners.size();

    points.reserve(num_corners);
    corners_list.reserve(1);
    points_list.reserve(1);

    for (int k = 0; k < num_corners; k++)
    {
        float i = (float)(k % cols);
        float j = (float)(-1 * k / cols);
        cv::Vec3f point(i, j, 0);
        points.push_back(point);
    }

    corners_list.push_back(corners);
    points_list.push_back(points);

    return 0;
}



/*
This function takes in vectors containing point sets and corner sets, as well as an initial camera matrix. 
The function then performs a calibration and calculates the calibrated camera matrix and distortion coefficients.
*/
float calibrateCamera(std::vector<std::vector<cv::Vec3f>>& points_list, std::vector<std::vector<cv::Point2f>>& corners_list, cv::Mat& camera_matrix, cv::Mat& dist_coeff)
{
    std::vector<cv::Mat> rot, trans;

    float error = cv::calibrateCamera(points_list,
        corners_list,
        cv::Size(1280, 720),
        camera_matrix,
        dist_coeff,
        rot,
        trans,
        cv::CALIB_FIX_ASPECT_RATIO,
        cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, DBL_EPSILON));

    return(error);
}


/*
 This function takes a camera matrix and distortion coefficients as input and saves the calibration information into a CSV file. 
 The CSV file can be retrieved later to calculate the camera pose.
 */
int saveCalibration(cv::Mat& camera_matrix, cv::Mat& dist_coeff)
{
    std::string fileName = "intrinsic_data.csv";
    char* fileName_char = new char[fileName.length() + 1];
    strcpy(fileName_char, fileName.c_str());

    std::string columnName = "camera_matrix";
    char* labelName_char = new char[columnName.length() + 1];
    strcpy(labelName_char, columnName.c_str());

    std::vector<float> camVector;
    for (int i = 0; i < camera_matrix.rows; i++) {
        for (int j = 0; j < camera_matrix.cols; j++) {
            float f_val = (float)camera_matrix.at<double>(i, j);
            camVector.push_back(f_val);
        }
    }
    append_image_data_csv(fileName_char, labelName_char, camVector);

    columnName = "distortion_coeff";
    char* label_char = new char[columnName.length() + 1];
    strcpy(label_char, columnName.c_str());

    std::vector<float> distVector;
    for (int i = 0; i < dist_coeff.rows; i++) {
        for (int j = 0; j < dist_coeff.cols; j++) {
            float f_val = (float)dist_coeff.at<double>(i, j);
            distVector.push_back(f_val);
        }
    }
    append_image_data_csv(fileName_char, label_char, distVector);

    return(0);
}
