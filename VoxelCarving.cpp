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
//Reading Matrix from a file
void ReadMatFromFile(Mat* Matrix, const string filename, const string MatName)
{
	FileStorage fs; 
	fs.open(filename, FileStorage::READ);
	fs[MatName] >> *Matrix;
}
//Store Matrix in a file
void PrintMatToFile(Mat Matrix, const string filename, const string MatName)
{
	FileStorage fs(filename, FileStorage::WRITE);
	fs << MatName << Matrix;
}
////////////////////////////////////////////
// Class contains Camera settings and parameters
class VoxelCamera
{
public:
	Mat rvec; // rotation of camera
	Mat tvec; // translation of camera
	Mat Intrinsic; // Intrinsic matrix of camera
	Mat Distortion; // Distortion matrix of camera
	static unsigned char number; // total numbers of cameras
	unsigned char ID; // camera ID
	Mat image;
	VoxelCamera(){}
	//constructor to create a camera object
	VoxelCamera(Mat CalibImage, Mat forground, Mat Intr, Mat Dist) 
	{
		VoxelCamera cam;
		ID = cam.number; // camera ID
		number++; // increment the total number of cameras
		bool success = getExtrinsicfromBoard(rvec, tvec, CalibImage, 8, 5, Intr); // get camera position 
		Intrinsic = Intr;
		Distortion = Dist;
		image = forground; // store forground image
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
	//Extract Extrinsic properties from checkerboard
	bool getExtrinsicfromBoard(Mat& rvec, Mat& tvec, Mat boardImg, int width, int height, Mat IntrinsicParam)
	{
		//checkerboard dimensions
		Size checkerboard_size;
		checkerboard_size.width = width;
		checkerboard_size.height = height;
		//vector of image point pixel locations corresponding to checkerboard positions
		vector<Point2f> imagePoints;
		bool success = findChessboardCorners(boardImg, checkerboard_size, imagePoints, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
		//vector of checkerboard coordinates which correspond to checkerboard positions in 3D
		vector<Point3f> objectPoints;
		//Determine positions of checkerboard positions in the real world
		calcChessboardCoord(checkerboard_size, 1, objectPoints);
		Mat dummy;
		// Determine rotation and translation matrices for camera
		success = solvePnP(objectPoints, imagePoints, IntrinsicParam, dummy, rvec, tvec);
		PrintMatToFile(rvec, "temp.yml", "rvec");
		PrintMatToFile(tvec, "temp.yml", "tvec");
		return success;
	}
	//Determine positions of checkerboard positions in the real world
	// squareSize -> determines the actual real world board corner length
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
	//Specify the dimensions of the 3D voxel grid 
	int width;  
	int height; 
	int depth; 	
	float unit; // converts unit width to unit distance 
	vector<VoxelCamera> cameras; //cameras used for the voxel grid
	string unitType; // The actual unit name like meteres etc
	uchar* voxelPoints; //array of voxel points 
	vector< vector< Point_< float > > > Table; //Vector of an array of voxel points
	cv::Point_<uchar>* PixelValue; 
	vector<Point3f> PointCloud; // vector of point cloud points
	//initializing the voxel grid
	Voxel(int width, int height, int depth,float ut)
	{
		Voxel::width = width; Voxel::height = height; Voxel::depth = depth;
		voxelPoints = new uchar[width*height*depth];
		PixelValue = new Point_<uchar>[width*height*depth];
		unit = ut;
	}
	// Add camera a camera with camera properties 
	void AddCamera(Mat r,Mat t,Mat Intr,Mat Dist,Mat view)
	{
		VoxelCamera cam;
		cameras.push_back(VoxelCamera(r, t, Intr, Dist, view));
	}

	void AddCamera(VoxelCamera cam)
	{
		cameras.push_back(cam);
	}
	// Generate lookup table which links each voxel point to a corresponding pixel in each camera image
	void generateLookup()
	{
		Point3_<float> WorldPoints_arr;
		vector< Point3_< float> > WorldPoints;
		//loop through each voxel in voxel grid
		for (int i = 0; i < width; i++)
		for (int j = 0; j < height; j++)
		for (int k = 0; k < depth; k++)
		{
			(WorldPoints_arr).x = ((float)i)*unit;
			(WorldPoints_arr).y = ((float)j)*unit;
			(WorldPoints_arr).z = ((float)k)*unit;
			WorldPoints.push_back(WorldPoints_arr);
		}
		//for each camera do this
		for (int c = 0; c < cameras.size(); c++)
		{
			vector< Point_<float> > imagePoints;
			//projects world points on to the image and stores these projections in imagePoints
			projectPoints(WorldPoints, cameras[c].rvec, cameras[c].tvec, cameras[c].Intrinsic, cameras[c].Distortion, imagePoints);
			Table.push_back(imagePoints);
		}

		
		//Table = LookUpTable(cameras.size(),width, height, depth, unit);
		//Table.generateLookup(cameras);
	}
	// lookup a point from the table generated by generateLookup
	inline Point_<float> getPointFromLookup(uchar c, uchar w, uchar h, uchar d)
	{
		return (Table[c])[w*height*depth + h*depth + d];
	}
	// This function determines if the object is present at the voxel location 
	void ComputeVoxelValues()
	{
		//pfile helps send stuff to matlab as a text file for post processing
		FILE *pfile;
		pfile = fopen("gogooo.txt", "w");
		if (cameras.size() == 0)
		{
			printf("\nAdd cameras to the scene!!\n");
			return;
		}
		//loop through each voxel
		for (int xdim = 0; xdim < width; xdim++)
		{
			for (int ydim = 0; ydim < height; ydim++)
			{
				for (int zdim = 0; zdim < depth; zdim++)
				{
					//Voxelvalue is a flag to keep track of which cameras actually detect an object
					uchar Voxelvalue = 0;
					//For each camera
					for (uchar cam = 0; cam <cameras.size(); cam++)
					{
						Size s = cameras[cam].image.size();
						//check if located point is within the voxel grid
						Point_<float> pt = getPointFromLookup(cam, xdim, ydim, zdim);
						if (s.width>(int)pt.x	&&	s.height>(int)pt.y	&& pt.x>0 && pt.y > 0)
						{
							uchar a = cameras[cam].image.at<uchar>((int)pt.y, (int)pt.x * (cameras[cam].image.channels()));
							if (a > 100)
							{
								//increment this flag to determine if there are necessary number of occurances in the images
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
					//Add this voxel to the point cloud as a point that contains an object location
					if (Voxelvalue == 2)
					{
					//	fprintf(pfile, "%d,%d,%d\n", xdim, ydim, zdim);
						PointCloud.push_back(Point3f(xdim,ydim,zdim));
					}
				}
			}
		}
		
	}
	//Visualization functions using Viz3D 
	void viewVoxel()
	{
		viz::Viz3d myWindow("VoxelPoints");
		viz::WCloud cloudwidget(PointCloud);
		cloudwidget.setRenderingProperty(viz::LINE_WIDTH,4.0);
		myWindow.showWidget("CloudWidget",cloudwidget);
		myWindow.spin();
		return;
	}
	//Dumping points into a file for later use
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
	// garbage collection
	~Voxel()
	{
		delete[] PixelValue;
		delete[] voxelPoints;
	}
	
};
