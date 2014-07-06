#include "stdafx.cpp"
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include<math.h>
using namespace cv;
using namespace std;

Mat Find_Largest_Component(Mat image)
{
	//First find the contours 
	image = image.clone();
	vector<vector <Point> > contours;
	vector<Vec4i> hier;
	findContours(image, contours, hier, CV_RETR_LIST,
		CV_CHAIN_APPROX_NONE);
	int number_of_contours = (int)contours.size();
	int max_contour_index = 0;
	int selected_contour_index = 0;
	int i = 0;
	for (int max_area = 0,second_max_ind = 0; i < number_of_contours; i++)
	{
		//vector<Point> current_contour = contours[i];
		vector<Point> current_contour = contours[i];
		double area = contourArea(current_contour, false);
		if (max_area < area)
		{
			second_max_ind = max_contour_index;
			max_contour_index = i;
			max_area = area;
			selected_contour_index = max_contour_index;
		}
	}
	vector<Point> contour = contours[selected_contour_index];
	vector<Point> LargestRegion;
	Scalar color(255);
	Mat output_image = Mat::zeros(image.size(), CV_8UC1);
	//Mat output_image = image.clone();// = Mat(LargestRegion);
	//drawContours(output_image, contours, selected_contour_index, color, CV_FILLED, 8, noArray(), 0, Point());
	drawContours(output_image, contours, selected_contour_index, color, CV_FILLED, 8);
	namedWindow("LargestComponent", CV_WINDOW_AUTOSIZE); //create a window called "Coutours"
	imshow("LargestComponent", output_image);
	cvWaitKey(0);
	destroyWindow("LargestComponent");
	return output_image;
	//return LargestRegion;
}
