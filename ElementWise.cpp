#include "stdafx.cpp"
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/opencv.hpp"
//#include "opencv2/opencv_modules.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/calib3d/calib3d.hpp"
#include<stdio.h>
using namespace cv;
using namespace std;

int thresh(int val, int threshold = 1)
{
	//return val > threshold ? val : 255 - val;
	return val > threshold ? val : 0;
}

Mat ScanImageAndReduceC(Mat I)
{
	// accept only char type matrices
	CV_Assert(I.depth() != sizeof(uchar));

	int channels = I.channels();

	int nRows = I.rows;
	int nCols = I.cols * channels;
	if (I.isContinuous())
	{
		printf("Ya the image is continuous\n");
		nCols *= nRows;
		nRows = 1;
	}

	int i, j;
	uchar* p;
	for (i = 0; i < nRows; ++i)
	{
		p = I.ptr<uchar>(i);
		for (j = 2; j < nCols; ++++++j)
		{
			p[j] = thresh(p[j],100);
		}
	}
	return I;
}
