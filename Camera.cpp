#include "Camera.h"

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
#include <vector>

using namespace std;

#define PI 3.14159265359

Camera::Camera()
{

}

Camera::~Camera()
{

}

cv::Mat Camera::getT(double xyx_th1, double xyx_th2, double xyx_th3, double transx, double transy, double transz)
{
	//cv::Mat T = cv::Mat::zeros(3, 4, CV_64F);

	//T.at<double>(0, 0) = cos(xyx_th2);
	//T.at<double>(1, 0) = sin(xyx_th2)*sin(xyx_th3);
	//T.at<double>(2, 0) = cos(xyx_th3)*sin(xyx_th2);

	//T.at<double>(0, 1) = sin(xyx_th1)*sin(xyx_th2);
	//T.at<double>(1, 1) = (cos(xyx_th1)*cos(xyx_th3)) - (cos(xyx_th2)*sin(xyx_th1)*sin(xyx_th3));
	//T.at<double>(2, 1) = (-cos(xyx_th1)*sin(xyx_th3)) - (cos(xyx_th2)*cos(xyx_th3)*sin(xyx_th1));

	//T.at<double>(0, 2) = -cos(xyx_th1)*sin(xyx_th2);
	//T.at<double>(1, 2) = (cos(xyx_th3)*sin(xyx_th1)) + (cos(xyx_th1)*cos(xyx_th2)*sin(xyx_th3));
	//T.at<double>(2, 2) = (cos(xyx_th1)*cos(xyx_th2)*cos(xyx_th3)) - (sin(xyx_th1)*sin(xyx_th3));

	//T.at<double>(0, 3) = transx;
	//T.at<double>(1, 3) = transy;
	//T.at<double>(2, 3) = transz;

	//return T;

	cv::Mat T = cv::Mat::zeros(4, 4, CV_64F);
	cv::Mat R = cv::Mat::zeros(3, 3, CV_64F);

	R.at<double>(0, 0) = cos(xyx_th2);
	R.at<double>(1, 0) = sin(xyx_th1)*sin(xyx_th2);
	R.at<double>(2, 0) = -cos(xyx_th1)*sin(xyx_th2);

	R.at<double>(0, 1) = sin(xyx_th2)*sin(xyx_th3);
	R.at<double>(1, 1) = (cos(xyx_th1)*cos(xyx_th3)) - (cos(xyx_th2)*sin(xyx_th1)*sin(xyx_th3));
	R.at<double>(2, 1) = (cos(xyx_th3)*sin(xyx_th1)) - (cos(xyx_th1)*cos(xyx_th2)*sin(xyx_th3));

	R.at<double>(0, 2) = cos(xyx_th3)*sin(xyx_th2);
	R.at<double>(1, 2) = -(cos(xyx_th1)*sin(xyx_th3)) - (cos(xyx_th2)*cos(xyx_th3)*sin(xyx_th1));
	R.at<double>(2, 2) = (cos(xyx_th1)*cos(xyx_th2)*cos(xyx_th3)) - (sin(xyx_th1)*sin(xyx_th3));

	R = R.t();
	R.copyTo(T.rowRange(0, 3).colRange(0, 3));

	T.at<double>(0, 3) = transx;
	T.at<double>(1, 3) = transy;
	T.at<double>(2, 3) = transz;

	T.at<double>(3, 0) = 0.0;
	T.at<double>(3, 1) = 0.0;
	T.at<double>(3, 2) = 0.0;
	T.at<double>(3, 3) = 1.0;

	

	return T;

	//double th1 = xyx_th1;
	//double th2 = xyx_th2;
	//double d1 = transx;
	//double d2 = transy;
	//double d3 = transz;
	//
	//cv::Mat tc2w = cv::Mat::zeros(3, 1, CV_64F); // tc2w 는 world 기준 카메라 좌표
	//tc2w.at<double>(0, 0) = d3;
	//tc2w.at<double>(1, 0) = d1;
	//tc2w.at<double>(2, 0) = d2;

	//float fCtemp = cos(th2*PI / 180 + PI / 2.0);
	//float fStemp = sin(th2*PI / 180 + PI / 2.0);

	//float fCTilting = cos(th1 * PI / 180);
	//float fSTilting = sin(th1 * PI / 180);

	//cv::Mat Rw2c = cv::Mat::zeros(cv::Size(3, 3), CV_64F);

	//Rw2c.at<double>(0, 0) = fCtemp;				Rw2c.at<double>(0, 1) = fStemp;					Rw2c.at<double>(0, 2) = 0;
	//Rw2c.at<double>(1, 0) = fSTilting * fStemp;	Rw2c.at<double>(1, 1) = -fSTilting * fCtemp;	Rw2c.at<double>(1, 2) = fCTilting;
	//Rw2c.at<double>(2, 0) = fCTilting * fStemp;	Rw2c.at<double>(2, 1) = -fCTilting * fCtemp;	Rw2c.at<double>(2, 2) = -fSTilting;

	//cv::Mat tw2c = -Rw2c * tc2w; 

	//cv::Mat T = cv::Mat::zeros(cv::Size(4, 4), CV_64F);
	//Rw2c.copyTo(T.rowRange(0, 3).colRange(0, 3));	
	//tw2c.copyTo(T.rowRange(0, 3).col(3));
	//T.row(3).col(3) = 1;

	//return T;

}

cv::Mat Camera::getK()
{
	double fx = 638.923602;
	double fy = 638.832943;
	double cx = 327.234846;
	double cy = 231.953842;

	cv::Mat K = cv::Mat::zeros(3, 3, CV_64F);

	K.at<double>(0, 0) = fx;
	K.at<double>(1, 0) = 0;
	K.at<double>(2, 0) = 0;

	K.at<double>(0, 1) = 0;
	K.at<double>(1, 1) = fy;
	K.at<double>(2, 1) = 0;

	K.at<double>(0, 2) = cx;
	K.at<double>(1, 2) = cy;
	K.at<double>(2, 2) = 1;

	return K;
}

cv::Mat Camera::getKT(cv::Mat K, cv::Mat T)
{
	cv::Mat KT;
	KT = K * T;

	return KT;
}

//void Camera::InExpt1(Vector4d &pt1)
//{
//	InEx = Intrinsic*Extrinsic;
//	exptres = Extrinsic*pt1;
//	inexptres = Intrinsic*Extrinsic*pt1 / exptres(2);
//
//	pair<int, int> a;
//	a.first = inexptres(0);
//	a.second = inexptres(1);
//
//	pixl_result.push_back(a);
//}
//
//
//void Camera::show_pixl()
//{
//	for (int i = 0; i < pixl_result.size(); i++)
//	{
//		cout << pixl_result[i].first << ", " << pixl_result[i].second << endl;
//	}
//
//}
