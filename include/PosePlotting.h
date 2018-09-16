#include <opencv2/opencv.hpp>
//#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/highgui/highgui.hpp"

using namespace cv;
using namespace std;

// plotting variables
Mat plotArea = Mat(Size(800, 800), CV_8UC3, Scalar(255, 255, 255));
int maxRows = plotArea.rows;
Scalar lineColor = Scalar(255, 0, 0);
int cellSize = 1;
int shiftInXAxis = 400;
int shiftInYAxis = 400;

// plot path
void plotPath(Mat plotArea, int maxRows, float old_global_X_pos, float old_global_Y_pos, float global_X_pos, float global_Y_pos)
{
	// get line starting point
	Point point1((old_global_X_pos * cellSize + cellSize / 2) + shiftInXAxis * cellSize,
		maxRows - (old_global_Y_pos * cellSize + cellSize / 2) - shiftInYAxis * cellSize);
	// get line ending point
	Point point2((global_X_pos * cellSize + cellSize / 2) + shiftInXAxis * cellSize,
		maxRows - (global_Y_pos * cellSize + cellSize / 2) - shiftInYAxis * cellSize);

	// plot line
	line(plotArea, point1, point2, lineColor, 2);

	// show plot area with the points
	imshow("Plot", plotArea);

	// refresh every 1 ms
	waitKey(1);
}


// plotting (2) variables

// scale factors
float minimum_scale = -1;
float maximum_scale = 1;

vector<Point2f> pointsVector;
Point2f pointInfo;

// plot path in a dynamic area
void dynamicPlotPath(Mat localPlotArea, int maxRows, float global_X_pos, float global_Y_pos, float TotalDistance)
{
	// add the data to a point vector
	pointInfo.x = global_X_pos;
	pointInfo.y = (-1)*global_Y_pos;
	pointsVector.push_back(pointInfo);

	// check if scalling is needed for the X-axis
	if (global_X_pos < minimum_scale || global_X_pos > maximum_scale)
	{
		minimum_scale = -abs(global_X_pos) - 0.5;
		maximum_scale = abs(global_X_pos) + 0.5;
	}

	// check if scalling is needed for the Y-axis
	if (global_Y_pos < minimum_scale || global_Y_pos > maximum_scale)
	{
		minimum_scale = -abs(global_Y_pos) - 0.5;
		maximum_scale = abs(global_Y_pos) + 0.5;
	}

	for (int i = 0; i < pointsVector.size(); i++)
	{
		// define point center
		//Point point_center(maxRows * (global_X_pos - minimum_scale) / (maximum_scale - minimum_scale),
		//	maxRows - maxRows * (global_Y_pos - minimum_scale) / (maximum_scale - minimum_scale));

		Point point_center(maxRows * (pointsVector[i].x - minimum_scale) / (maximum_scale - minimum_scale),
			maxRows - maxRows * (pointsVector[i].y - minimum_scale) / (maximum_scale - minimum_scale));

		// plot the circle in the position
		circle(localPlotArea, point_center, 2, Scalar(0, 0, 255), 2);
	}

	float scaleValue = maximum_scale - minimum_scale;

	//stringstream ss(stringstream::in | stringstream::out);
	stringstream ss1(stringstream::in | stringstream::out);
	stringstream ss2(stringstream::in | stringstream::out);

	//ss << scaleValue;
	ss1 << TotalDistance << " m";
	ss2 << "Point(" << global_X_pos << ", " << global_Y_pos << ")";

	//string scaleText = ss.str();
	string DistText = ss1.str();
	string PointText = ss2.str();

	//putText(plotArea, scaleText, Point(0, maxRows - 20), FONT_HERSHEY_PLAIN, 4, Scalar(255), 4);
	putText(plotArea, DistText, Point(0, maxRows - 50), FONT_HERSHEY_PLAIN, 2, Scalar(0, 255, 0), 4);
	putText(plotArea, PointText, Point(0, maxRows - 20), FONT_HERSHEY_PLAIN, 2, Scalar(0, 255, 0), 4);


	// show plot area with the points
	imshow("Plot", localPlotArea);

	// refresh every 1 ms
	waitKey(1);

	plotArea = Mat(localPlotArea.size(), CV_8UC3, Scalar(0, 0, 0));
}


// plotting lines
// scale factors
float minimum_scale1 = -1;
float maximum_scale1 = 1;

vector<Point2f> pointsVector1, oldpointsVector1;
Point2f pointInfo1, oldpointInfo1;

// plot path in a dynamic area
void dynamicPlotPath1(Mat localPlotArea, int maxRows, float global_X_pos, float global_Y_pos, float oldglobal_X_pos, float oldglobal_Y_pos)
{
	// add the data to a point vector
	pointInfo1.x = global_X_pos;
	pointInfo1.y = (-1)*global_Y_pos;
	pointsVector1.push_back(pointInfo1);

	oldpointInfo1.x = global_X_pos;
	oldpointInfo1.y = (-1)*global_Y_pos;
	oldpointsVector1.push_back(oldpointInfo1);

	// check if scalling is needed for the X-axis
	if (global_X_pos < minimum_scale || global_X_pos > maximum_scale)
	{
		minimum_scale = -abs(global_X_pos) - 0.5;
		maximum_scale = abs(global_X_pos) + 0.5;
	}

	// check if scalling is needed for the Y-axis
	if (global_Y_pos < minimum_scale || global_Y_pos > maximum_scale)
	{
		minimum_scale = -abs(global_Y_pos) - 0.5;
		maximum_scale = abs(global_Y_pos) + 0.5;
	}

	for (int i = 0; i < pointsVector1.size(); i++)
	{
		// define point center
		//Point point_center(maxRows * (global_X_pos - minimum_scale) / (maximum_scale - minimum_scale),
		//	maxRows - maxRows * (global_Y_pos - minimum_scale) / (maximum_scale - minimum_scale));

		Point point_center(maxRows * (pointsVector1[i].x - minimum_scale) / (maximum_scale - minimum_scale),
			maxRows - maxRows * (pointsVector1[i].y - minimum_scale) / (maximum_scale - minimum_scale));

		Point oldpoint_center(maxRows * (oldpointsVector1[i].x - minimum_scale) / (maximum_scale - minimum_scale),
			maxRows - maxRows * (oldpointsVector1[i].y - minimum_scale) / (maximum_scale - minimum_scale));

		// plot the circle in the position
		circle(localPlotArea, point_center, 2, Scalar(0, 0, 255), 2);
		circle(localPlotArea, oldpoint_center, 2, Scalar(0, 0, 255), 2);
		// plot line
		line(localPlotArea, point_center, oldpoint_center, Scalar(0, 0, 255), 2);
	}

	float scaleValue = maximum_scale - minimum_scale;
	stringstream ss(stringstream::in | stringstream::out);
	ss << scaleValue;
	string scaleText = ss.str();
	putText(plotArea, scaleText, Point(0, maxRows - 20), FONT_HERSHEY_PLAIN, 4, Scalar(0), 4);

	// show plot area with the points
	imshow("Plot", localPlotArea);

	// refresh every 1 ms
	waitKey(1);

	plotArea = Mat(localPlotArea.size(), CV_8UC3, Scalar(255, 255, 255));
}