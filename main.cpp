#include "Camera.h"
#define PI 3.14159265359

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

using namespace std;

//triangulation 함수 생성
cv::Mat triangulation(pair<int, cv::Mat> pixl_cur, pair<int, cv::Mat> pixl_mat, cv::Mat K_cur, cv::Mat K_mat, cv::Mat Tcur, cv::Mat Tmat, int i)
{
	int width = 640;
	int height = 480;

	//for (int i = 0; i < pixl_cur.size(); i++)
	//{
	//	if ((pixl_cur[i].second.at<double>(0, 0) > width) || (pixl_cur[i].second.at<double>(1, 0) > height))
	//		continue;
	//	if ((pixl_mat[i].second.at<double>(0, 0) > width) || (pixl_mat[i].second.at<double>(1, 0) > height))
	//		continue;

	//	pair<int, cv::Mat> tmp_cur;
	//	tmp_cur.first = i;
	//	tmp_cur.second = pixl_cur[i].second;
	//	pixl_cur_in.push_back(tmp_cur);

	//	pair<int, cv::Mat> tmp_mat;
	//	tmp_mat.first = i;
	//	tmp_mat.second = pixl_mat[i].second;
	//	pixl_mat_in.push_back(tmp_mat);
	//}

	cv::Mat A = cv::Mat::zeros(4, 4, CV_64F);
	vector<pair<cv::Mat, pair<double, double>>> tri_pt;

	pair<cv::Mat, pair<double, double>> tmp1;
	tmp1.first = K_cur * Tcur.rowRange(0,3);
	tmp1.second.first = pixl_cur.second.at<double>(0, 0);
	tmp1.second.second = pixl_cur.second.at<double>(1, 0);
	tri_pt.push_back(tmp1);

	pair<cv::Mat, pair<double, double>> tmp2;
	tmp2.first = K_mat * Tmat.rowRange(0, 3);
	tmp2.second.first = pixl_mat.second.at<double>(0, 0);
	tmp2.second.second = pixl_mat.second.at<double>(1, 0);
	tri_pt.push_back(tmp2);

	if ((tmp1.second.first > 0) && (tmp1.second.second > 0) && (tmp2.second.first > 0) && (tmp2.second.second > 0))
	{
		if ((tmp1.second.first < width) && (tmp1.second.second < height) && (tmp2.second.first < width) && (tmp2.second.second < height))
		{
			cv::Mat AA;
			AA.create(4, 4, CV_64F);
			for (int j = 0; j < tri_pt.size(); j++)
			{
				int x = (int)(width - tri_pt.at(j).second.first);
				int y = (int)(height - tri_pt.at(j).second.second);

				for (int k = 0; k < 4; k++)
				{
					AA.at<double>(2 * j, k) = x*tri_pt.at(j).first.at<double>(2, k) - tri_pt.at(j).first.at<double>(0, k);
					AA.at<double>(2 * j + 1, k) = y*tri_pt.at(j).first.at<double>(2, k) - tri_pt.at(j).first.at<double>(1, k);
				}
			}

			cv::Mat w, u, vt;
			cv::SVDecomp(AA, w, u, vt);

			cv::Mat tmp = cv::Mat::zeros(3, 1, CV_64FC1);
			tmp.at<double>(0, 0) = vt.at<double>(3, 0) / vt.at<double>(3, 3);
			tmp.at<double>(1, 0) = vt.at<double>(3, 1) / vt.at<double>(3, 3);
			tmp.at<double>(2, 0) = vt.at<double>(3, 2) / vt.at<double>(3, 3);

			tmp = tmp.t();

			return tmp;
		}

		else
		{
			cv::Mat nomat = cv::Mat::zeros(3, 1, CV_64F);
			nomat.at<double>(0, 0) = -1000.0;
			nomat.at<double>(1, 0) = -1000.0;
			nomat.at<double>(2, 0) = -1000.0;

			nomat = nomat.t();
			return nomat;
		}
	}

	else
	{
		cv::Mat nomat = cv::Mat::zeros(3, 1, CV_64F);
		nomat.at<double>(0, 0) = -1000.0;
		nomat.at<double>(1, 0) = -1000.0;
		nomat.at<double>(2, 0) = -1000.0;
		nomat = nomat.t();
		return nomat;
	}

}


int main(void)
{
	Camera* cam_cur = new Camera();
	Camera* cam_mat = new Camera();
	
	//Make Camera Pose
	cv::Mat K_cur = cam_cur->getK();
	cv::Mat K_mat = cam_mat->getK();

	//double cxyx_th1 = 90.0 * PI / 180; 	double cxyx_th2 = 90.0 * PI / 180; 	double cxyx_th3 = 2.0 * PI / 180;
	//double mxyx_th1 = 90.3 * PI / 180; 	double mxyx_th2 = 90.1 * PI / 180; 	double mxyx_th3 = 2.15 * PI / 180;

	//double cxyx_th1 = 1.0;// 6.0;
	//double cxyx_th2 = 1.0;//-2.0;
	//double cxyx_th3 = 1.0;

	//double mxyx_th1 = 1.0;//6.0;
	//double mxyx_th2 = 1.0;//-2.0;
	//double mxyx_th3 = 1.0;

	double cxyx_th1 = 90.2 * PI / 180;
	double cxyx_th2 = 90.1 * PI / 180;
	double cxyx_th3 = -10.15 * PI/180;

	double mxyx_th1 = 90.1 * PI / 180;
	double mxyx_th2 = 90.3 * PI / 180;
	double mxyx_th3 = -10.2 * PI / 180;
	//double mxyx_th3 = 0*PI / 180;

	//cv::Mat Tcur = cam_cur->getT(cxyx_th1, cxyx_th2, cxyx_th3, -0.036, 0.29699, 0.46388); //camera 기준
	//cv::Mat Tmat = cam_mat->getT(mxyx_th1, mxyx_th2, mxyx_th3, 0.01, 0.34699, 0.56388);

	cv::Mat Tcur = cam_cur->getT(cxyx_th1, cxyx_th2, cxyx_th3, 0.1, 0.1, 1.1); //camera 기준.
	cv::Mat Tmat = cam_mat->getT(mxyx_th1, mxyx_th2, mxyx_th3, 0.8, 0.5, 1.3);

	//For test world -> camera
	//cv::Mat Tcur = cam_cur->getT(90 * PI / 180, 90 * PI / 180, 0, 0, 0, 0); //camera 기준
	//cv::Mat test = cv::Mat::zeros(4, 1, CV_64F);
	//test.at<double>(0, 0) = 1.0;
	//test.at<double>(1, 0) = 2.0;
	//test.at<double>(2, 0) = 3.0;
	//test.at<double>(3, 0) = 1.0;
	//cv::Mat testans = Tcur.rowRange(0, 3)*test;

	FILE *pFile = fopen("Landmarks_GT.txt", "w");
	//Make Ground Truth Landmark
	std::vector<pair<int,cv::Mat>> gt_landmark;
	int idx = 0;
	int k = 0;
	
	for (double i = 0.0; i < 20.0; i=i+1.0)
	{
		for (double j = 0.0; j < 5.0; j=j+1.0)
		{
			pair<int, cv::Mat> tmp;
			tmp.second = cv::Mat::zeros(1, 3, CV_64F);
			tmp.second.at<double>(0, 0) = (5.0 * (i + 1.0));
			tmp.first = idx;
			//tmp.second.at<double>(0, 1) = (-6.0 + 2.0 * ((double)j + 1.0));
			//tmp.second.at<double>(0, 2) = (double)j;
			tmp.second.at<double>(0, 1) = (-6.0 + 2.0 * (j + 1.0));
			tmp.second.at<double>(0, 2) = (j + 1.0);

			//cout << idx << endl;
			gt_landmark.push_back(tmp);
			idx++;

			//cout << gt_landmark[k].second.at<int>(0, 0) << "  " << gt_landmark[k].second.at<int>(0, 1) << "  " << gt_landmark[k].second.at<int>(0, 2) << endl;
			fprintf(pFile, "%lf\t %lf\t %lf\n", gt_landmark[k].second.at<double>(0, 0), gt_landmark[k].second.at<double>(0, 1), gt_landmark[k].second.at<double>(0, 2));
			k++;
		}
	}
	fclose(pFile);

	//Take a picture
	for (int i = 0; i < gt_landmark.size(); i++)
	{
		cv::Mat landmark = cv::Mat::zeros(4, 1, CV_64F);
		landmark.at<double>(0, 0) = gt_landmark.at(i).second.at<double>(0, 0);
		landmark.at<double>(1, 0) = gt_landmark.at(i).second.at<double>(0, 1);
		landmark.at<double>(2, 0) = gt_landmark.at(i).second.at<double>(0, 2);
		landmark.at<double>(3, 0) = 1.0;

		cv::Mat pixl_tmp;
		pixl_tmp = K_cur * Tcur.rowRange(0,3) * landmark;

		pixl_tmp.at<double>(0, 0) = pixl_tmp.at<double>(0, 0) / pixl_tmp.at<double>(2, 0);
		pixl_tmp.at<double>(1, 0) = pixl_tmp.at<double>(1, 0) / pixl_tmp.at<double>(2, 0);
		pixl_tmp.at<double>(2, 0) = 1.0;

		pixl_tmp.at<double>(0, 0) = 640.0 - pixl_tmp.at<double>(0, 0);
		pixl_tmp.at<double>(1, 0) = 480.0 - pixl_tmp.at<double>(1, 0);


		cv::Mat pixl_tmp_mat;
		pixl_tmp_mat = K_mat * Tmat.rowRange(0, 3) * landmark;

		pixl_tmp_mat.at<double>(0, 0) = pixl_tmp_mat.at<double>(0, 0) / pixl_tmp_mat.at<double>(2, 0);
		pixl_tmp_mat.at<double>(1, 0) = pixl_tmp_mat.at<double>(1, 0) / pixl_tmp_mat.at<double>(2, 0);
		pixl_tmp_mat.at<double>(2, 0) = 1.0;

		pixl_tmp_mat.at<double>(0, 0) = 640.0 - pixl_tmp_mat.at<double>(0, 0);
		pixl_tmp_mat.at<double>(1, 0) = 480.0 - pixl_tmp_mat.at<double>(1, 0);

		pair<int, cv::Mat> cur_ptmp;
		cur_ptmp.first = i;
		cur_ptmp.second = pixl_tmp;

		pair<int, cv::Mat> mat_ptmp;
		mat_ptmp.first = i;
		mat_ptmp.second = pixl_tmp_mat;

		cam_cur->pixl_cur.push_back(cur_ptmp);
		cam_mat->pixl_mat.push_back(mat_ptmp);
	}

	//vector<pair<int, cv::Mat>>estimated_landmarks;
	//double all_error;
	//all_error = 0;

	//for (int i = 0; i < 100; i++)
	//{
	//	double error;
	//	error = tri(cam_cur->pixl_cur[i], cam_mat->pixl_mat[i], cam_cur, cam_mat, K_cur, K_mat, Tcur, Tmat, gt_landmark.at(i));

	//	if (error > 0)
	//	{
	//		all_error = all_error + error;
	//	}
	//		
	//}

	//cout << all_error << endl;

	vector<pair<int, cv::Mat>> triangulation_result;
	
	FILE *pFile_ = fopen("Triangulation_.txt", "w");
	for (int i = 0; i < 100; i++)
	{
		cv::Mat ans;
		ans = triangulation(cam_cur->pixl_cur[i], cam_mat->pixl_mat[i], K_cur, K_mat, Tcur, Tmat, i);
		cout << ans.at<double>(0, 0) << "  " << ans.at<double>(0,1) << "  " << ans.at<double>(0,2) << endl;
		fprintf(pFile_, "%lf\t %lf\t %lf\n", ans.at<double>(0, 0) ,ans.at<double>(0, 1) , ans.at<double>(0, 2));
	}
	fclose(pFile_);
	return 0;
}

