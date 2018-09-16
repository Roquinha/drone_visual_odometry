#ifndef DRONE_VISUAL_ODOMETRY
#define DRONE_VISUAL_ODOMETRY

/*  Visual odometry library */

#include <iostream>
#include <math.h>
#include <thread>
#include <string.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"

//#include "SurfMatching/SurfMatching.h"
#include "opencv2/cudafeatures2d.hpp"
#include "opencv2/xfeatures2d/cuda.hpp"

#include "opencv2/opencv_modules.hpp"

#include "PoseEstimation.h"
#include "ParallelProc.h"
#include "KalmanFilter.h"
#include "PosePlotting.h"


using namespace cv;
using namespace std;
using namespace cv::cuda;


class VisualOdometry
{
public:
	VisualOdometry(bool debug = false);
	~VisualOdometry();
	// Initialize the algorithm with the first frame
	void init_frame(Mat& frame_1);
	// Compute the odometry with the new and the last frames
	void compute_next(Mat& new_frame);
	// Get current position
	double get_x();
	double get_y();
	double get_z();
	double get_yaw();
	// Update distance to the wall
	void set_wall_distance(double distance);

	// Enable / Disable debug
	inline void set_debug(bool val) {debug_ = val;}

private:
	// Calibration parameters
	Mat matCamCalibCV;
	Mat invmatCamCalib;
	Mat distCoeffs;

	// Log files
	ofstream fileResult_H;
	ofstream fileResult_POSE;
	ofstream fileDebug;

	// Last frame (grayscale and undistorted)
	Mat last_frame, last_frame_c;
	vector <KeyPoint> last_keypoints;
	GpuMat last_descriptorsGPU;

	// Debug image
	Mat IMG_OUT;

	// Distance to wall (z)
	double wall_distance;
	// Distance covered (x+y)
	double TotalDistance;

	// Homography mats
	Mat InitialHomographyWorld;
	Mat MatRTInitial;

	// Camera Yaw (accumulated)
	double CamYaw;
	// Position
	double Xacc;
	double Yacc;
	double Zacc;
	double OldXacc;
	double OldYacc;
	double OldZacc;
	
	// Debug flag
	bool debug_;

	// Drawing parameters
	//Mat traj = Mat::zeros(600, 600, CV_8UC3);
	//char text[100];
	//Point textOrg(10, 50);
	//int fontFace = FONT_HERSHEY_PLAIN;
	//double fontScale = 1;
	//int thickness = 1;

	// Thread vector
	vector<thread> threads;

	// Private methods
	double round_to_digits(double value, int digits);
	void join_threads();
};

#endif