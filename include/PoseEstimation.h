#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/xfeatures2d.hpp>
//#include<opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <ctype.h>
#include <algorithm> // for copy
#include <iterator>  // for ostream_iterator
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>
#include <thread>
//#include <ppl.h>
//#include <tbb/tbb.h>

using namespace cv;
using namespace cv::xfeatures2d;
using namespace std;
//using namespace concurrency;
//using namespace tbb;


const float inlier_threshold = 2.5f; // Distance threshold to identify inliers
const float match_ratio = 0.8f;   // Nearest neighbor matching ratio
const double alpha = 0.5; double beta;

void featureTracking(Mat img_1, Mat img_2, vector<Point2f>& points1, vector<Point2f>& points2, vector<uchar>& status)	{

	//this function automatically gets rid of points for which tracking fails
	vector<float> err;
	Size winSize = Size(21, 21);
	TermCriteria termcrit = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 0.01);

	calcOpticalFlowPyrLK(img_1, img_2, points1, points2, status, err, winSize, 3, termcrit, 0, 0.001);

	//getting rid of points for which the KLT tracking failed or those who have gone outside the frame
	int indexCorrection = 0;
	for (int i = 0; i < status.size(); i++)
	{
		Point2f pt = points2.at(i - indexCorrection);
		if ((status.at(i) == 0) || (pt.x < 0) || (pt.y < 0))	{
			if ((pt.x < 0) || (pt.y < 0))	{
				status.at(i) = 0;
			}
			points1.erase(points1.begin() + (i - indexCorrection));
			points2.erase(points2.begin() + (i - indexCorrection));
			indexCorrection++;
		}
	}
}

void featureDetection(Mat img_1, Mat img_2, Mat *img_out, Mat *frametoframeHomographt, int *Nomathes)	{   //uses FAST as of now, modify parameters as necessary
	vector<KeyPoint> keypoints_1, keypoints_2;
	Mat descriptors_1, descriptors_2;
	Mat img_1_gr, img_2_gr;
	Mat homography;
	Ptr<SIFT> SIFTdetector = SIFT::create(0, 4, 0.04, 9, 1.4);
	Ptr<AKAZE> akaze = AKAZE::create();

	// Working with grayscale
	/*	parallel_invoke(
	[&] {cvtColor(img_1, img_1_gr, COLOR_BGR2GRAY);
	SIFTdetector->detectAndCompute(img_1_gr, noArray(), keypoints_1, descriptors_1, false);
	},
	[&] {cvtColor(img_2, img_2_gr, COLOR_BGR2GRAY);
	SIFTdetector->detectAndCompute(img_2_gr, noArray(), keypoints_2, descriptors_2, false);
	}
	);
	*/
	thread t1([&] {
		cvtColor(img_1, img_1_gr, COLOR_BGR2GRAY);
		SIFTdetector->detectAndCompute(img_1_gr, noArray(), keypoints_1, descriptors_1, false);
	});
	thread t2([&] {
		cvtColor(img_2, img_2_gr, COLOR_BGR2GRAY);
		SIFTdetector->detectAndCompute(img_2_gr, noArray(), keypoints_2, descriptors_2, false);
	});
	t1.join();
	t2.join();
	/*cvtColor(img_1, img_1_gr, COLOR_BGR2GRAY);
	cvtColor(img_2, img_2_gr, COLOR_BGR2GRAY);
	akaze->detectAndCompute(img_1_gr, noArray(), keypoints_1, descriptors_1);
	akaze->detectAndCompute(img_2_gr, noArray(), keypoints_2, descriptors_2);*/

	BFMatcher bruteforcematcher(NORM_L2);
	vector<DMatch> matches;

	bruteforcematcher.match(descriptors_1, descriptors_2, matches, noArray());

	double max_dist = 0; double min_dist = 100;
	vector< DMatch > good_matches;

	//-- Quick calculation of max and min distances between keypoints
	for (int i = 0; i < descriptors_1.rows; i++)
	{
		double dist = matches[i].distance;
		if (dist < min_dist) min_dist = dist;
		if (dist > max_dist) max_dist = dist;

		/*if (matches[i].distance <= max(2 * min_dist, 0.02))
		{
		good_matches.push_back(matches[i]);
		}*/
	}

	/*parallel_for(0, descriptors_1.rows, [&](int i){

	double dist = matches[i].distance;
	if (dist < min_dist) min_dist = dist;
	if (dist > max_dist) max_dist = dist;

	if (matches[i].distance <= max(2 * min_dist, 0.02))
	{
	good_matches.push_back(matches[i]);
	}
	});*/

	for (int i = 0; i < descriptors_1.rows; i++)
	{
		if (matches[i].distance <= max(2 * min_dist, 0.02))
		{
			good_matches.push_back(matches[i]);
		}
	}

	//-- Localize the object
	vector<Point2f> pntImage1, pntImage2;

	for (int i = 0; i < good_matches.size(); i++)
	{
		//-- Get the keypoints from the good matches
		pntImage1.push_back(keypoints_1[good_matches[i].queryIdx].pt);
		pntImage2.push_back(keypoints_2[good_matches[i].trainIdx].pt);
	}

	*frametoframeHomographt = findHomography(pntImage1, pntImage2, RANSAC, 6, noArray(), 1500, 0.6);
	*Nomathes = good_matches.size();
	beta = 1 - alpha;
	addWeighted(img_1, 0.5, img_2, 0.5, 0.0, *img_out);

	for (int i = 0; i < pntImage1.size(); i++)
	{
		circle(*img_out, pntImage1[i], 2, Scalar(0, 255, 0), 2);
		circle(*img_out, pntImage2[i], 1, Scalar(0, 0, 255), 1);
		line(*img_out, pntImage1[i], pntImage2[i], Scalar(255, 0, 0), 2, 4);
	}
}

void featureDetectionFAST(Mat img_1, vector<Point2f>& points1)	{   //uses FAST as of now, modify parameters as necessary
	vector<KeyPoint> keypoints_1;
	int fast_threshold = 20;
	bool nonmaxSuppression = true;
	FAST(img_1, keypoints_1, fast_threshold, nonmaxSuppression);

	/*Ptr<SIFT> SIFTdetector = SIFT::create(0, 4, 0.04, 9, 1.4);
	Ptr<AKAZE> akaze = AKAZE::create();
	SIFTdetector->detect(img_1, keypoints_1, noArray());*/
	KeyPoint::convert(keypoints_1, points1, vector<int>());
}

void featureMatching2(Mat img_1, vector<KeyPoint> keypoints_1, Mat descriptors_1, Mat img_2, vector<KeyPoint> keypoints_2, Mat descriptors_2, Mat *frametoframeHomographt, int *Nomathes, Mat *img_out)
{
	BFMatcher bruteforcematcher(NORM_L2);
	//vector<DMatch> matches;
	std::vector<std::vector<cv::DMatch>> matches;
	//bruteforcematcher.match(Descriptor1, Descriptor2, matches, noArray());
	bruteforcematcher.knnMatch(descriptors_1, descriptors_2, matches, 2, noArray(), false);

	double max_dist = 0; double min_dist = 100;
	vector< DMatch > good_matches;

	//-- Quick calculation of max and min distances between keypoints
	//for (int i = 0; i < Descriptor1.rows; i++)
	//{
	//	double dist = matches[i][0].distance;
	//	if (dist < min_dist) min_dist = dist;
	//	if (dist > max_dist) max_dist = dist;

	//	/*if (matches[i].distance <= max(2 * min_dist, 0.02))
	//	{
	//	good_matches.push_back(matches[i]);
	//	}*/
	//}
	for (int i = 0; i < matches.size(); ++i)
	{
		const float ratio = 0.44; // As in Lowe's paper; can be tuned
		if (matches[i][0].distance < ratio * matches[i][1].distance)
		{
			good_matches.push_back(matches[i][0]);
		}
	}

	/*parallel_for(0, descriptors_1.rows, [&](int i){

	double dist = matches[i].distance;
	if (dist < min_dist) min_dist = dist;
	if (dist > max_dist) max_dist = dist;

	if (matches[i].distance <= max(2 * min_dist, 0.02))
	{
	good_matches.push_back(matches[i]);
	}
	});*/

	/*for (int i = 0; i < Descriptor1.rows; i++)
	{
	if (matches[i].distance <= max(2 * min_dist, 0.02))
	{
	good_matches.push_back(matches[i]);
	}
	}*/

	//-- Localize the object
	vector<Point2f> pntImage1, pntImage2;

	for (int i = 0; i < good_matches.size(); i++)
	{
		//-- Get the keypoints from the good matches
		pntImage1.push_back(keypoints_1[good_matches[i].queryIdx].pt);
		pntImage2.push_back(keypoints_2[good_matches[i].trainIdx].pt);
	}

	*frametoframeHomographt = findHomography(pntImage1, pntImage2, RANSAC, 3, noArray(), 2000, 0.995);
	cout << *frametoframeHomographt << endl;
	*Nomathes = good_matches.size();
	img_1.copySize(*img_out);
	beta = 1 - alpha;
	addWeighted(img_1, alpha, img_2, beta, 0.0, *img_out, -1);

	for (int i = 0; i < pntImage1.size(); i++)
	{
		circle(*img_out, pntImage1[i], 2, Scalar(0, 255, 0), 2);
		circle(*img_out, pntImage2[i], 1, Scalar(0, 0, 255), 1);
		line(*img_out, pntImage1[i], pntImage2[i], Scalar(255, 0, 0), 2, 4);
	}
}

void featureMatching(vector<KeyPoint> keypoints_1, vector<KeyPoint> keypoints_2, Mat descriptor_1, Mat descriptor_2, int imgwidth, int imgheigh, vector<Point2f>& points_1, vector<Point2f>& points_2)	{   //uses FAST as of now, modify parameters as necessary
	BFMatcher bruteforcematcher(NORM_L2);
	//vector<DMatch> matches;
	std::vector<std::vector<cv::DMatch>> matches;
	//bruteforcematcher.match(Descriptor1, Descriptor2, matches, noArray());
	bruteforcematcher.knnMatch(descriptor_1, descriptor_2, matches, 2, noArray(), false);
	double max_dist = 0; double min_dist = 100;
	vector< DMatch > good_matches;

	//-- Quick calculation of max and min distances between keypoints
	//for (int i = 0; i < Descriptor1.rows; i++)
	//{
	//	double dist = matches[i][0].distance;
	//	if (dist < min_dist) min_dist = dist;
	//	if (dist > max_dist) max_dist = dist;

	//	/*if (matches[i].distance <= max(2 * min_dist, 0.02))
	//	{
	//	good_matches.push_back(matches[i]);
	//	}*/
	//}

	//only 25% of maximum of possible distance
	//double tresholdDist = 0.25 * sqrt(double(imgheigh*imgheigh + imgwidth*imgwidth));


	//for (size_t i = 0; i < matches.size(); ++i)
	//{
	//	for (int j = 0; j < matches[i].size(); j++)
	//	{
	//		Point2f from = keypoints_1[matches[i][j].queryIdx].pt;
	//		Point2f to = keypoints_2[matches[i][j].trainIdx].pt;

	//		//calculate local distance for each possible match
	//		double dist = sqrt((from.x - to.x) * (from.x - to.x) + (from.y - to.y) * (from.y - to.y));

	//		//save as best match if local distance is in specified area and on same height
	//		if (dist < tresholdDist && abs(from.y - to.y) < 5)
	//		{
	//			good_matches.push_back(matches[i][j]);
	//			j = matches[i].size();
	//		}
	//	}
	//}

	for (int i = 0; i < matches.size(); ++i)
	{
		const float ratio = 0.44; // As in Lowe's paper; can be tuned
		if (matches[i][0].distance < ratio * matches[i][1].distance)
		{
			good_matches.push_back(matches[i][0]);
		}
	}


	/*parallel_for(0, descriptors_1.rows, [&](int i){

	double dist = matches[i].distance;
	if (dist < min_dist) min_dist = dist;
	if (dist > max_dist) max_dist = dist;

	if (matches[i].distance <= max(2 * min_dist, 0.02))
	{
	good_matches.push_back(matches[i]);
	}
	});*/

	/*for (int i = 0; i < Descriptor1.rows; i++)
	{
	if (matches[i].distance <= max(2 * min_dist, 0.02))
	{
	good_matches.push_back(matches[i]);
	}
	}*/

	//-- Localize the object
	for (int i = 0; i < good_matches.size(); i++)
	{
		//-- Get the keypoints from the good matches
		points_1.push_back(keypoints_1[good_matches[i].queryIdx].pt);
		points_2.push_back(keypoints_2[good_matches[i].trainIdx].pt);
	}
}