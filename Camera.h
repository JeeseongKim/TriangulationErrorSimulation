#ifndef CAMERA_
#define CAMERA_

#include <iostream>
//#include <Eigen/Dense>
#include <cmath>
//#include <Eigen/Core>
//#include <Eigen/StdVector>
#include <cstdlib>
#include <ctime>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

class Camera
{

public:

	Camera();
	~Camera();

	//Matrix<double, 3, 3> Intrinsic;
	//Matrix<double, 3, 4> Extrinsic;

	double pixel_x, pixel_y;
	vector<pair<double, double>> pixl_result;

	/*cv::Mat getT(double th1, double th2, double d1, double d2, double d3);*/
	cv::Mat getT(double xyx_th1, double xyx_th2, double xyx_th3, double transx, double transy, double transz);
	cv::Mat getK();
	cv::Mat getKT(cv::Mat K, cv::Mat T);

	//void Expt1(Vector4d &pt1);
	//void InExpt1(Matrix<double, 3, 3> &In, Matrix<double, 3, 4> &Ex, Vector4d &pt1);

	//void InExpt1(Vector4d &pt1);

	//Vector3d exptres;
	//Vector3d inexptres;
	//Matrix<double, 3, 4> InEx;

	vector<pair<int, cv::Mat>> pixl_cur; //카메라로 찍은 landmark pixel 저장
	vector<pair<int, cv::Mat>> pixl_mat; //카메라로 찍은 landmark pixel 저장

	cv::Mat KT;
	cv::Mat p1;
	cv::Mat p2;
	cv::Mat p3;

	void save_pixel(double x1, double y1);
	void show_pixl();
};




#endif