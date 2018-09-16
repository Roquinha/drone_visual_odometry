#ifndef SURFMATCHING
#define SURFMATCHING

/*  Visual odometry library */

#include <iostream>
#include <math.h>
#include <thread>
#include <string.h>
#include <stdio.h>
#include <fstream>

#include <ros/ros.h>
#include <ros/package.h>
#include "opencv2/opencv.hpp"
//#include <opencv2/highgui.hpp>


using namespace cv;
using namespace std;

class SurfMatching
{
//Constructor y prototipos de los métodos
public:
	//Constructor:
	SurfMatching();
	//Destructor
	~SurfMatching();

	//Métodos usados
	Mat findHomographySurf(Mat& frame1, Mat& frame2);

//Atributos de la clase
private:

	// Last frame (grayscale and undistorted)
	Mat last_frame, last_frame_c;
	vector <KeyPoint> last_keypoints;
	Mat last_descriptor;
};

#endif
