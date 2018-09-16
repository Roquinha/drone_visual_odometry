#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/xfeatures2d.hpp>
//#include<opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
//#include <boost/thread/thread.hpp>

#include <iostream>
#include <ctype.h>
#include <algorithm> // for copy
#include <iterator>  // for ostream_iterator
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>
//#include <ppl.h>
//#include <tbb/tbb.h>

using namespace cv;
using namespace cv::xfeatures2d;
using namespace std;
//using namespace concurrency;
//using namespace tbb;

#define NBUCKETS 8

class Parallel_DetectorFeature : public ParallelLoopBody
{
private:
	Mat img;
	vector<KeyPoint> *retVal;
	Ptr<SIFT> DetectorFeature;

public:
	Parallel_DetectorFeature(Mat inputImgage, vector<KeyPoint> *keypoints_roi, Ptr<SIFT> Detector)
		: img(inputImgage), retVal(keypoints_roi), DetectorFeature(Detector){}

	virtual void operator()(const Range& range) const
	{
		//cout << "range.start " << range.start << " range.end " << range.end << endl;
		for (int i = range.start; i < range.end; i++){
			int fil = i / NBUCKETS;
			int col = i - NBUCKETS*fil;
			Mat ROI_img(img, Rect(col*(img.cols / NBUCKETS), fil*(img.rows / NBUCKETS), img.cols / NBUCKETS, img.rows / NBUCKETS));
			DetectorFeature->detect(ROI_img, retVal[i]);
		}

	}
};

void ParallelDetectFeatures(Mat inputImgage, vector<KeyPoint > &keypoints, int mumfeaures, int rows, int cols)
{
	//int nOctaveLayers = 3;
	//int nfeatures=0, int nOctaveLayers=3, double contrastThreshold=0.04, double edgeThreshold=10, double sigma=1.6
	//Ptr<SIFT> DetectorFeature = SIFT::create(mumfeaures / rows / cols, nOctaveLayers, 0.0003, 9.0, 1.6);

	int nOctaveLayers = 4;
	//int nfeatures=0, int nOctaveLayers=3, double contrastThreshold=0.04, double edgeThreshold=10, double sigma=1.6

	Ptr<SIFT> DetectorFeature = SIFT::create(mumfeaures / rows / cols/*0*/, nOctaveLayers, 0.04, 9.0, 1.4);

	int dimx = inputImgage.cols / cols;
	int dimy = inputImgage.rows / rows;
	vector<KeyPoint > keypoints_roi[NBUCKETS*NBUCKETS];

	parallel_for_(Range(0, NBUCKETS*NBUCKETS), Parallel_DetectorFeature(inputImgage, keypoints_roi, DetectorFeature));

	for (int i = 0; i < rows; ++i){
		for (int j = 0; j < cols; ++j){
			//cout << "viendo subventana " << i*8+j;
			vector<KeyPoint > points = keypoints_roi[i*NBUCKETS + j];
			//cout << "num feat " << points.size() << endl;
			for (int k = 0; k < points.size(); ++k){
				KeyPoint Pto = points[k];
				Pto.pt.x = points[k].pt.x + j*dimx;
				Pto.pt.y = points[k].pt.y + i*dimy;
				keypoints.push_back(Pto);
				//cout << k << " " << keypoints.size() << " " << keypoints[k].pt.x << " " << keypoints[k].pt.y << endl;
			}
		}
	}
}


#define NUM_THREADS 1

class Parallel_DescriptorFeature : public ParallelLoopBody
{

private:
	Mat img;
	vector<KeyPoint > *retVal;
	vector<KeyPoint > key_point;
	Ptr<SIFT> DetectorFeature;
	Mat *desc_img;
	int num_elem;
	int num_elem_tot;

public:
	Parallel_DescriptorFeature(Mat inputImgage, vector<KeyPoint> keypoint, vector<KeyPoint> *keypoints_roi, Ptr<SIFT> Detector, Mat *descriptors_roi, int elem_num, int tot_elem_num)
		: img(inputImgage), key_point(keypoint), retVal(keypoints_roi), DetectorFeature(Detector), desc_img(descriptors_roi), num_elem(elem_num), num_elem_tot(tot_elem_num) {}

	virtual void operator()(const Range& range) const
	{
		//cout << "range.start " << range.start << " range.end " << range.end << endl;
		for (int i = range.start; i < range.end; i++){
			if (i < NUM_THREADS - 1) for (int j = 0; j < num_elem; ++j)retVal[i].push_back(key_point[i*num_elem + j]);
			else   for (int j = (NUM_THREADS - 1)*num_elem; j < num_elem_tot; ++j) retVal[NUM_THREADS - 1].push_back(key_point[j]);

			DetectorFeature->compute(img, retVal[i], desc_img[i]);
		}
	}
};


void ParallelComuteDescriptors(Mat inputImgage, vector<KeyPoint > keypoints, Ptr<SIFT> DetectorFeature, Mat &descriptors)
{
	vector<KeyPoint > keypoints_roi[NUM_THREADS];
	Mat descriptors_roi[NUM_THREADS];
	int num_keypoints = keypoints.size() / NUM_THREADS + 1;

	parallel_for_(Range(0, NUM_THREADS), Parallel_DescriptorFeature(inputImgage, keypoints, keypoints_roi, DetectorFeature, descriptors_roi, num_keypoints, keypoints.size()));
	for (int t = 0; t < NUM_THREADS; ++t) descriptors.push_back(descriptors_roi[t]);

	/*
	Mat descriptors_reshape(keypoints.size(),128,);
	for(int t=0;t<NUM_THREADS;++t){
	cv::Mat ROI_img(descriptors_reshape,cv::Rect(0,num_keypoints*t,128,keypoints_roi[t].size()));
	descriptors_roi[t].copyTo(ROI_img);
	}
	descriptors=descriptors_reshape;
	*/
}