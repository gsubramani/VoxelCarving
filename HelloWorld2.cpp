// HelloWorld2.cpp : Defines the entry point for the console application.
//
#ifdef _WIN64
#include "stdafx.h"
#endif

#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/calib3d/calib3d.hpp"
#include "ComponentSelection.cpp"

#include "Camera_calb.cpp"
#include "ElementWise.cpp"
#include "silhouette.cpp"
#include "VoxelCarving.cpp"
#include "CameraGlobalPos.cpp"

///////////////////////////////////////////////
//s#include <dshow.h>
//#include <Ks.h>             // Required by KsMedia.h
//#include <KsMedia.h>        // For KSPROPERTY_CAMERACONTROL_FLAGS_*
///////////////////////////////////////////////
using namespace cv;
using namespace std;

int main()
{
	Mat bk0 = imread("BGCamera0.jpg");
	Mat bk1 = imread("BGCamera1.jpg");

	Mat fk0 = imread("ObjectCamera0.jpg");
	Mat fk1 = imread("ObjectCamera1.jpg");

	Mat sil0, sil1;
	//sil0 = silhoutte_calibrate(bk0, fk0); // cam0	
	//sil1 = silhoutte_calibrate(bk1, fk1); // cam1
	//imwrite("SilhCamera1.jpg", sil1);
	//imwrite("SilhCamera0.jpg", sil0);

	//return 1;

	sil1 = imread("SilhCamera1.jpg");
	sil0 = imread("SilhCamera0.jpg");
	namedWindow("window", CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"
	//imshow("window",sil0);
	//cvWaitKey(0);
	Mat Intrinsic0, Intrinsic1;
	const string filename("camera.yml");
	ReadMatFromFile(&Intrinsic0, filename, "camera_matrix");
	PrintMatToFile(Intrinsic0, "temp.yml", "temp");
	
	Intrinsic1 = Intrinsic0;
	
	Mat dummy;
	VoxelCamera Vcam0 = VoxelCamera(bk0, sil0, Intrinsic0,dummy);
	VoxelCamera Vcam1 = VoxelCamera(bk1, sil1, Intrinsic1,dummy);
	
	PrintMatToFile(Vcam0.rvec, "temp.yml", "vcatemp");
	Voxel vox(100, 100, 100,0.05);
	vox.AddCamera(Vcam0);
	vox.AddCamera(Vcam1);
	vox.generateLookup();
	//LookUpTable lt(2, 100, 100, 100,0.1);
	//lt.generateLookup(vox.cameras);
	vox.ComputeVoxelValues();
	vox.viewVoxel();
	//vox.printVoxelPoints("voxelpts.dat");
	//	AddCamera(Mat r, Mat t, Mat Intr, Mat Dist, Mat view)
	return 0;
}

int main22(int argc, char* argv[])
{
	//plotWorldOrigin(0, 0, 0);
		//return 1;
	VideoCapture cap0(0),cap1(1); // open the video camera no. 0
	cap0.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
	cap0.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
	cap1.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
	cap1.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
	//namedWindow("Cam0", CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"
	//namedWindow("Cam1", CV_WINDOW_AUTOSIZE);
	Mat Cam0, Cam1;
	
	printf("set the background up");
	while (1)
	{
		cap0.read(Cam0); // read a new frame from video
		cap1.read(Cam1); // read a new frame from video
		imshow("Cam0", Cam0); //show the frame in "MyVideo" window
		imshow("Cam1", Cam1); //show the frame in "MyVideo" window
		if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
		{
			cout << "esc key is pressed by user" << endl;
			break;
		}
	}

	//plotWorldOrigin(0, 0, 2);
	//plotWorldOrigin(0, 0, 1);

	//imwrite("BGCamera0.jpg",Cam0);
	//imwrite("BGCamera1.jpg", Cam1);
	//printf("set the forground up\n");
	//cvWaitKey(0);
	//printf("here");
	//cap0.read(Cam0); // read a new frame from video
	//cap1.read(Cam1); // read a new frame from video
	//imwrite("ObjectCamera1.jpg", Cam1);
	//imwrite("ObjectCamera0.jpg", Cam0);
	//return 1;

	Mat bk0 = imread("BGCamera0.jpg");
	Mat bk1 = imread("BGCamera1.jpg");

	Mat fk0 = imread("ObjectCamera0.jpg");
	Mat fk1 = imread("ObjectCamera1.jpg");

	Mat sil0, sil1;
	//sil0 = silhoutte_calibrate(bk0, fk0); // cam0	
	//sil1 = silhoutte_calibrate(bk1, fk1); // cam1
	//imwrite("SilhCamera1.jpg", sil1);
	//imwrite("SilhCamera0.jpg", sil0);

	//return 1;

	sil1 = imread("SilhCamera1.jpg");
	sil0 = imread("SilhCamera0.jpg");
	Mat Intrinsic0, Intrinsic1;
	const string filename("C:\\Users\\gsubramani\\Documents\\visual studio 2013\\Projects\\CameraCalibration\\CameraCalibration\\camera.yml");
	ReadMatFromFile(&Intrinsic0, filename, "camera_matrix");
	PrintMatToFile(Intrinsic0, "temp.yml", "temp");
	
	Intrinsic1 = Intrinsic0;
	
	Mat dummy;
	VoxelCamera Vcam0 = VoxelCamera(bk0, sil0, Intrinsic0,dummy);
	VoxelCamera Vcam1 = VoxelCamera(bk1, sil1, Intrinsic1,dummy);
	
	PrintMatToFile(Vcam0.rvec, "temp.yml", "vcatemp");
	Voxel vox(100, 100, 100,.22);
	vox.AddCamera(Vcam0);
	vox.AddCamera(Vcam1);
	vox.generateLookup();
	//LookUpTable lt(2, 100, 100, 100,0.1);
	//lt.generateLookup(vox.cameras);
	vox.ComputeVoxelValues();
	//vox.printVoxelPoints("voxelpts.dat");
	//	AddCamera(Mat r, Mat t, Mat Intr, Mat Dist, Mat view)
	return 0;
}
/////////////////////////////////////////////////////////////////
