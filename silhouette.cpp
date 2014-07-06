#include "stdafx.cpp"
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/opencv.hpp"
//#include "opencv2/opencv_modules.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include<math.h>
using namespace cv;

struct trackbar_data
{
	Mat background;
	Mat image;
	Mat silh;
};


Mat silhoutte_extract(Mat background, Mat image,float threshold)
{
	// compare each element with the background pixel value magnitude intensity
	// and return only the pixels with a large background difference
	// background and and image depth
	// image background subrtraction window select
	Mat silh = image.clone();
	CV_Assert(background.depth() == image.depth());
	// accept only char type matrices
	CV_Assert(background.depth() != sizeof(uchar));
	if (background.depth() != sizeof(uchar))
	{
		printf("not a char image\n");
		printf("the size of depth is %d\n", background.type());
	
	}
	int channels = background.channels();

	int nRows = background.rows;
	int nCols = background.cols * channels;
	if (background.isContinuous())
	{
		printf("Ya the image is continuous\n");
		nCols *= nRows;
		nRows = 1;
	}

	int i, j;
	uchar* silh_ptr;
	uchar* back_ptr;
	uchar* im_ptr;
	for (i = 0; i < nRows; ++i)
	{
		silh_ptr = silh.ptr<uchar>(i);
		back_ptr = background.ptr<uchar>(i);
		im_ptr = image.ptr<uchar>(i);
		
		for (j = 0; j < nCols; ++++++j) // skip the other channels to reach next pixel 
		{
			double dot_prod = back_ptr[j] * im_ptr[j] + back_ptr[j + 1] * im_ptr[j + 1] + back_ptr[j + 2] * im_ptr[j + 2];
			dot_prod = dot_prod / (255.0 * 255.0);
			double mag = (back_ptr[j] * back_ptr[j] / ((double)255.0 * 255.0)
				+ back_ptr[j + 1] * back_ptr[j + 1] / ((double)255.0 * 255.0)
				+ back_ptr[j + 2] * back_ptr[j + 2] / ((double)255 * 255.0));
			mag = mag*(im_ptr[j] * im_ptr[j] / ((double)255.0 * 255)
				+ im_ptr[j + 1] * im_ptr[j + 1] / ((double)255.0 * 255)
				+ im_ptr[j + 2] * im_ptr[j + 2] / ((double)255.0 * 255));

			mag = sqrt(mag);
			dot_prod = (mag == 0) ? 0 : dot_prod / mag; // chech for divide by zero
			
			//printf(" %lf %lf\n", mag, dot_prod);

			silh_ptr[j] = dot_prod > threshold? 0 : im_ptr[j];
			silh_ptr[j + 1] = dot_prod > threshold? 0 : im_ptr[j + 1];
			silh_ptr[j + 2] = dot_prod > threshold ? 0 : im_ptr[j + 2];
			//p[j] = thresh(p[j], 100);
		}
	}
	return silh;
}
////////////////////////////////////////////////
void on_trackbar(int position, void* data)
{
	Mat background = ((trackbar_data*)data)->background;
	Mat image = ((trackbar_data*)data)->image;
	((trackbar_data*)data)->silh = silhoutte_extract(background, image, (position + 900) / 1000.0);
	imshow("Background_Removal", ((trackbar_data*)data)->silh);
}
////////////////////////////////////////////////
void on_thresholding_trackbar(int position, void* data)
{
	threshold(((trackbar_data*)data)->image, ((trackbar_data*)data)->silh, position, 255, THRESH_BINARY);
	imshow("Thresholding", ((trackbar_data*)data)->silh);
}
////////////////////////////////////////////////
void on_Close(int position, void* data)
{
	if (position == 0) return;
	int morph_size = position;
	Mat element =
			getStructuringElement(MORPH_ELLIPSE,
			Size(2 * morph_size + 1, 2 * morph_size + 1),
			Point(morph_size, morph_size));
	morphologyEx(((trackbar_data*)data)->image, ((trackbar_data*)data)->silh, MORPH_CLOSE, element);
//	for (int i = 1; i < position; i++)
//	{
//		morphologyEx(((trackbar_data*)data)->silh, ((trackbar_data*)data)->silh, MORPH_CLOSE, element);
//	}
	imshow("Close_opp", ((trackbar_data*)data)->silh);
}
////////////////////////////////////////////////
Mat silhoutte_calibrate(const Mat background, const Mat image)
{
	int alpha_slider = 90;	
	int alpha_max = 100;
	trackbar_data data;
	data.background = background;
	data.image = image;
	data.silh = silhoutte_extract(background, image, 90);
	printf("\n+++++++++++Background removal stage++++++++++\n");
	namedWindow("Background_Removal", CV_WINDOW_AUTOSIZE);
	imshow("Background_Removal", data.silh);
	on_trackbar(992, &data);
	createTrackbar("Background_Removal", "Background_Removal", &alpha_slider, alpha_max, on_trackbar, &data);
	cvWaitKey(0);
	destroyWindow("Background_Removal");
	printf("\n+++++++++++Tresholding Stage++++++++++\n");
	cvtColor(data.silh,data.silh, CV_RGB2GRAY);
	printf("\nBW conversion\n");
	int thresholding_silder = 50;
	int thresholding_silder_max = 255;
	namedWindow("Thresholding", CV_WINDOW_AUTOSIZE);
	imshow("Thresholding", data.silh);
	data.image = data.silh.clone(); // basically new image to protect the input image
	on_thresholding_trackbar(50, &data);
	createTrackbar("Thresholding", "Thresholding", &thresholding_silder, thresholding_silder_max, on_thresholding_trackbar, &data);
	cvWaitKey(0);
	destroyWindow("Thresholding");
	printf("\n+++++++++++Gray Scale Dialation++++++++++\n");

	int close_iter = 5;
	int close_iter_max = 10;
	namedWindow("Close_opp", CV_WINDOW_AUTOSIZE);
	Mat swap_var;
	swap_var = data.image;
	data.image = data.silh;
	data.silh = swap_var;
	int morph_size = 5;
	Mat element = 
		getStructuringElement(MORPH_ELLIPSE, 
								Size(2 * morph_size + 1, 2 * morph_size + 1), 
								Point(morph_size, morph_size));
	morphologyEx(data.image, data.silh, MORPH_CLOSE, element);
	imshow("Close_opp", data.silh);
	on_Close(5, &data);
	createTrackbar("Close_opp", "Close_opp", &close_iter, close_iter_max, on_Close, &data);
	cvWaitKey(0);
	destroyWindow("Close_opp");
	data.silh = Find_Largest_Component(data.silh);
	return data.silh;
}

