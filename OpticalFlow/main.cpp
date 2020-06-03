/// <summary>
/// Mains this instance.
/// </summary>
/// <returns></returns>
/// --------------------------------------------------------------------
/// 创建日期：2018/10/29  11:05
/// 创建者：热夏
/// 邮箱：wangxianwenup@outlook.com
/// 所用设备：Windows10 64bit + VisualStudio 2017
/// 更改日期：
/// 概述：利用光流法进行运动目标检测
/// --------------------------------------------------------------------

#include<opencv2/video/video.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/core/core.hpp>
#include<iostream>
#include<cstdio>

using namespace std;
using namespace cv;

//--------声明全局函数------------
void tracking(Mat &frame,Mat &output);
bool addNewPoints();
bool acceptTrackedPoint(int i);

//--------声明全局变量------------
string window_name = "optical flow tracking 光流检测";
Mat gray;		//当前图片
Mat gray_prev;	//预测图片

vector<Point2f> points[2];		//point0为特征点的原来位置，point1为特征点的新位置
vector<Point2f> initial;		//初始化跟踪点的位置
vector<Point2f> features;		//检测的特征
int maxCount = 500;				//检测的最大特征数
double qLevel = 0.01;			//特征检测的等级
double minDest = 10.0;			//两特征点之间最小的距离
vector<uchar> status;			//跟踪特征的状态，特征的流发现为1，否则为0
vector<float> err;

//--------help函数 打印程序的说明-----------
static void help()
{
	//输出OpenCV版本
	cout << "\n\n\t\t\t用光流法检测运动目标\n"
		
		<< "\n\n\t\t\t   当前使用的OpenCV版本为：" << CV_VERSION
		<< "\n\n  ----------------------------------------------------------------------------";
}




//-------main函数 程序入口-------------------
int main()
{
	Mat frame;
	Mat result;

	VideoCapture capture(0);
   // VideoCapture capture("1.avi");
	help();

	if (capture.isOpened())
	{
		while (true)
		{
			capture >> frame;
			if (!frame.empty())             //不为空
			{
				tracking(frame, result);     //跟踪

			}
			else
			{
				printf("No Capture frame , Break");
				break;
			}

			int c = waitKey(50);
			if (27 == (char)c)
			{
				break;
			}



		}
	}

	return 0;
}
//--------tracking函数  跟踪运动目标------------------
//	frame：输入的视频帧         output:有跟踪结果的视频帧

void tracking(Mat &frame, Mat &output)
{
	cvtColor(frame, gray, CV_BGR2GRAY);
	frame.copyTo(output);

	//添加特征点
	if (addNewPoints())
	{
		goodFeaturesToTrack(gray, features, maxCount, qLevel, minDest);
		points[0].insert(points[0].end(), features.begin(), features.end());
		initial.insert(initial.end(), features.begin(), features.end());
	}
	if (gray_prev.empty())
	{
		gray.copyTo(gray_prev);
	}
	//l-k流光法运动估计
	calcOpticalFlowPyrLK(gray_prev, gray, points[0], points[1], status, err);
	//去掉一些不好的特征点
	int k = 0;
	for (size_t i = 0; i < points[1].size(); i++)
	{
		if (acceptTrackedPoint(i))
		{
			initial[k] = initial[i];
			points[1][k++] = points[1][i];

		}

	}
	points[1].resize(k);
	initial.resize(k);
	//显示特征点和运动轨迹
	for (size_t i = 0; i < points[1].size(); i++)
	{
		line(output, initial[i], points[1][i], Scalar(0, 0, 255));
		circle(output, points[1][i], 3, Scalar(0, 255, 0), -1);

	}

	//把当前跟踪结果作为下一次的参考
	swap(points[1],points[0]);
	swap(gray_prev,gray);

	imshow(window_name, output);
}


//--------addNewPoints函数 ：检测新店是否应该被添加----------------
//	return是否被添加的标志
bool addNewPoints()
{
	return points[0].size() <= 10;       //points.size()求行数     points.size()求列数

}

//-------acceptTrackedPoint函数：决定哪些跟踪点被接收-------------
bool acceptTrackedPoint(int i)
{
	return status[i] && ((abs(points[0][i].x - points[1][i].x) + abs(points[0][i].y - points[1][i].y)) > 2);
}

