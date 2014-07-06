#include "stdafx.cpp"
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/calib3d/calib3d.hpp"

using namespace std;
using namespace cv;

void _ReadMatFromFile(Mat* Matrix, const string filename, const string MatName)
{
	FileStorage fs;
	fs.open(filename, FileStorage::READ);
	fs[MatName] >> *Matrix;
}

void _PrintMatToFile(Mat Matrix, const string filename, const string MatName)
{
	FileStorage fs(filename, FileStorage::WRITE);
	fs << MatName << Matrix;
}

void worldXYZToPixel(Point2f* ImageOrigin,Point3f WorldOrigin,InputArray objectPoints,InputArray imagePoints,Mat Intrinsic,Mat Distortion)
{
	vector<Point3f> inputpoints;
	inputpoints.push_back(WorldOrigin);

	vector<Point2f> outputpoints;
	Mat rvec, tvec;
	solvePnP(objectPoints, imagePoints, Intrinsic, Distortion, rvec, tvec);
	projectPoints(inputpoints, rvec, tvec, Intrinsic, Distortion, outputpoints);
	*ImageOrigin = outputpoints.front();
	return;
}

void test()
{
	Mat camera_matrix;
	const string filename("C:\\Users\\gsubramani\\Documents\\visual studio 2013\\Projects\\CameraCalibration\\CameraCalibration\\camera.yml");
	_ReadMatFromFile(&camera_matrix, filename,"camera_matrix");
	_PrintMatToFile(camera_matrix,
		"C:\\Users\\gsubramani\\Documents\\visual studio 2013\\Projects\\CameraCalibration\\CameraCalibration\\test.yml","MatrixCalibValues");
}

static void calcChessboardCoord(Size boardSize, float squareSize, vector<Point3f>& corners)
{
	corners.resize(0);
	for (int i = 0; i < boardSize.height; i++)
	for (int j = 0; j < boardSize.width; j++)
		corners.push_back(Point3f(float(j*squareSize), float(i*squareSize), 0));
}



void plotWorldOrigin(int a,int b,int c)
{
	Mat camera_matrix;
	const string filename("C:\\Users\\gsubramani\\Documents\\visual studio 2013\\Projects\\CameraCalibration\\CameraCalibration\\camera.yml");
	_ReadMatFromFile(&camera_matrix, filename, "camera_matrix");
	////////////////////////////////////////////////////////////// 
	/// Use Checkerboard pattern to determine world coordinates///
	////////////////////////////////////////////////////////////// 
	VideoCapture cap(1);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
	namedWindow("ZeroWindow", CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"
	Mat image;
	cap.read(image);
	Size checkerboard_size;
	checkerboard_size.width = 8;
	checkerboard_size.height = 5;
	vector<Point2f> imagePoints;
	bool success = findChessboardCorners(image,	checkerboard_size,imagePoints, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
	////////////////////////////////////////////////////////////// 
	/// Create Object point array ///	
	////////////////////////////////////////////////////////////// 
	vector<Point3f> objectPoints;
	calcChessboardCoord(checkerboard_size,1,objectPoints);
	////////////////////////////////////////////////////////////// 
	Mat dummy;
	Point2f ImageOrigin;
	Point3f WorldOrigin;
	WorldOrigin.x = a; WorldOrigin.y = b; WorldOrigin.z = c;
	worldXYZToPixel(&ImageOrigin, WorldOrigin,objectPoints, imagePoints, camera_matrix, dummy);
	Scalar color = Scalar(0, 255, 0);
	//color.ones().mul(100);
	circle(image, ImageOrigin, 5, color, 1, 8,0);
	imshow("ZeroWindow", image);

	Mat sil1, sil0;
	sil1 = imread("SilhCamera1.jpg");
	sil0 = imread("SilhCamera0.jpg");
	circle(sil0, ImageOrigin, 5, color, 1, 8, 0);
	circle(sil1, ImageOrigin, 5, color, 1, 8, 0);
	imshow("SilhCamera1", sil1);
	imshow("SilhCamera0", sil0);
	cvWaitKey(0);
}

