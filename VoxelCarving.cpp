#include "stdafx.cpp"

#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/viz/vizcore.hpp>


using namespace cv;
using namespace std;

///////////////////////////////////////////
///////////////////////////////////////////

void ReadMatFromFile(Mat* Matrix, const string filename, const string MatName)
{
	FileStorage fs;
	fs.open(filename, FileStorage::READ);
	fs[MatName] >> *Matrix;
}

void PrintMatToFile(Mat Matrix, const string filename, const string MatName)
{
	FileStorage fs(filename, FileStorage::WRITE);
	fs << MatName << Matrix;
}
////////////////////////////////////////////
class VoxelCamera
{
public:
	Mat rvec;
	Mat tvec;
	Mat Intrinsic;
	Mat Distortion;
	static unsigned char number;
	unsigned char ID;
	Mat image;
	VoxelCamera(){}
	VoxelCamera(Mat CalibImage, Mat forground, Mat Intr, Mat Dist)
	{
		VoxelCamera cam;
		ID = cam.number;
		number++;
		bool success = getExtrinsicfromBoard(rvec, tvec, CalibImage, 8, 5, Intr);
		Intrinsic = Intr;
		Distortion = Dist;
		image = forground;
		if (success != true)
		{
		
			printf("\n o---ohhhh there is a problem \n");
		}
	}
	VoxelCamera(Mat r, Mat t, Mat Intr, Mat Dist, Mat view)
	{
		VoxelCamera cam;
		rvec = r;
		tvec = t;
		Intrinsic = Intr;
		Distortion = Dist;
		ID = cam.number;
		number++;
		image = view;
	}
private:
	bool getExtrinsicfromBoard(Mat& rvec, Mat& tvec, Mat boardImg, int width, int height, Mat IntrinsicParam)
	{
		Size checkerboard_size;
		checkerboard_size.width = width;
		checkerboard_size.height = height;
		vector<Point2f> imagePoints;
		bool success = findChessboardCorners(boardImg, checkerboard_size, imagePoints, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
		vector<Point3f> objectPoints;
		calcChessboardCoord(checkerboard_size, 1, objectPoints);
		Mat dummy;
		success = solvePnP(objectPoints, imagePoints, IntrinsicParam, dummy, rvec, tvec);
		PrintMatToFile(rvec, "temp.yml", "rvec");
		PrintMatToFile(tvec, "temp.yml", "tvec");
		return success;
	}
	static void calcChessboardCoord(Size boardSize, float squareSize, vector<Point3f>& corners)
	{
		corners.resize(0);
		for (int i = 0; i < boardSize.height; i++)
		for (int j = 0; j < boardSize.width; j++)
			corners.push_back(Point3f(float(j*squareSize), float(i*squareSize), 0));
	}

};
unsigned char VoxelCamera::number = 0;


class Voxel{
public:
	int width;	// x
	int height; // y
	int depth;	// z
	float unit; // converts unit width to unit distance 
	vector<VoxelCamera> cameras;
	string unitType;
	uchar* voxelPoints;
	vector< vector< Point_< float > > > Table;
	cv::Point_<uchar>* PixelValue;
	vector<Point3f> PointCloud;
	Voxel(int width, int height, int depth,float ut)
	{
		Voxel::width = width; Voxel::height = height; Voxel::depth = depth;
		voxelPoints = new uchar[width*height*depth];
		PixelValue = new Point_<uchar>[width*height*depth];
		unit = ut;
	}
	
	void AddCamera(Mat r,Mat t,Mat Intr,Mat Dist,Mat view)
	{
		VoxelCamera cam;
		cameras.push_back(VoxelCamera(r, t, Intr, Dist, view));
	}

	void AddCamera(VoxelCamera cam)
	{
		cameras.push_back(cam);
	}
	void generateLookup()
	{
		Point3_<float> WorldPoints_arr;
		vector< Point3_< float> > WorldPoints;
		for (int i = 0; i < width; i++)
		for (int j = 0; j < height; j++)
		for (int k = 0; k < depth; k++)
		{
			(WorldPoints_arr).x = ((float)i)*unit;
			(WorldPoints_arr).y = ((float)j)*unit;
			(WorldPoints_arr).z = ((float)k)*unit;
			WorldPoints.push_back(WorldPoints_arr);
		}
		for (int c = 0; c < cameras.size(); c++)
		{
			vector< Point_<float> > imagePoints;
			projectPoints(WorldPoints, cameras[c].rvec, cameras[c].tvec, cameras[c].Intrinsic, cameras[c].Distortion, imagePoints);
			Table.push_back(imagePoints);
		}

		
		//Table = LookUpTable(cameras.size(),width, height, depth, unit);
		//Table.generateLookup(cameras);
	}
	inline Point_<float> getPointFromLookup(uchar c, uchar w, uchar h, uchar d)
	{
		return (Table[c])[w*height*depth + h*depth + d];
	}

	void ComputeVoxelValues()
	{
		FILE *pfile;
		pfile = fopen("gogooo.txt", "w");
		if (cameras.size() == 0)
		{
			printf("\nAdd cameras to the scene!!\n");
			return;
		}
		for (int xdim = 0; xdim < width; xdim++)
		{
			for (int ydim = 0; ydim < height; ydim++)
			{
				for (int zdim = 0; zdim < depth; zdim++)
				{
					uchar Voxelvalue = 0;
					for (uchar cam = 0; cam <cameras.size(); cam++)
					{
						Size s = cameras[cam].image.size();
						Point_<float> pt = getPointFromLookup(cam, xdim, ydim, zdim);
						if (s.width>(int)pt.x	&&	s.height>(int)pt.y	&& pt.x>0 && pt.y > 0)
						{
							uchar a = cameras[cam].image.at<uchar>((int)pt.y, (int)pt.x * (cameras[cam].image.channels()));
							if (a > 100)
							{
								Voxelvalue++;
								//fprintf(pfile,"%d %d %d %d, ", xdim, ydim, zdim,
								//(int)cameras[cam].image.at<uchar>((int)pt.y, (int)pt.x));
								//fprintf(pfile, "%d %d %d %d %d %d, ", xdim, ydim, zdim, (int)pt.x, (int)pt.y,a);
							}
							/*if (cameras[cam].image.at<uchar>((int)pt.y, (int)pt.x) == 1)
							{
							Voxelvalue = Voxelvalue * 1;
							}
							else
							{
							Voxelvalue = 0;
							}*/
						}
					}
					//voxelPoints[depth*height*xdim + depth*ydim + zdim] = Voxelvalue;
					if (Voxelvalue == 2)
					{
					//	fprintf(pfile, "%d,%d,%d\n", xdim, ydim, zdim);
						PointCloud.push_back(Point3f(xdim,ydim,zdim));
					}
				}
			}
		}
		
	}
	void viewVoxel()
	{
		viz::Viz3d myWindow("VoxelPoints");
		viz::WCloud cloudwidget(PointCloud);
		cloudwidget.setRenderingProperty(viz::LINE_WIDTH,4.0);
		myWindow.showWidget("CloudWidget",cloudwidget);
		myWindow.spin();
		return;
	}
	void printVoxelPoints(char* filename)
	{
		FILE *pfile;
		pfile = fopen(filename, "w");
		for (int xdim = 0; xdim < width; xdim++)
		{
			for (int ydim = 0; ydim < height; ydim++)
			{
				for (int zdim = 0; zdim < depth; zdim++)
				{
					fprintf(pfile,"%d %d %d %d\n", xdim,ydim,zdim,
						(int)voxelPoints[depth*height*xdim + depth*ydim + zdim]
						);
				}
			}
		}
	}
	
	~Voxel()
	{
		delete[] PixelValue;
		delete[] voxelPoints;
	}
	
};
