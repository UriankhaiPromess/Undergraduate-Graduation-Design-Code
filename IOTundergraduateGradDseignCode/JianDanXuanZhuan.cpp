#include <iostream>
#include <fstream>
#include <cstdlib>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <vector>
#include <cassert>

using namespace std;
using namespace cv;

int main() 
{
	ifstream rotData("D:\\unityOutputs\\OriginOutputs\\zitaidata.txt");
	string line;
	float pi = 3.1415925;
	int numtmp = 0;
	int fileRows = 708;
	vector<float> thisFrameRotAngle(3);
	VideoCapture cap("D:\\unityOutputs\\OriginOutputs\\media\\movie_rgb.mp4");//读取视频

	//获取视频图像宽高
	int w = int(cap.get(CAP_PROP_FRAME_WIDTH));
	int h = int(cap.get(CAP_PROP_FRAME_HEIGHT));

	//获取fps
	double fps = cap.get(CAP_PROP_FPS);

	//设置输出视频
	VideoWriter out;
	out.open("D:\\unityOutputs\\OriginOutputs\\media\\ZhiXuanZhuan001.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, Size(w, h / 2), true);
	//声明当前帧的原始rgb图像和输出rgb图像
	Mat curr,frame_outMat;
	Mat outMat = cv::Mat::zeros(cv::Size(w, h), CV_8UC3);
	for (int i = 0; i < fileRows; i++)
	{
		getline(rotData,line);
		stringstream ss(line);
		float x;
		ss >> numtmp;
		for (int j = 0; j < 6; j++)
		{
			ss >> x;
			thisFrameRotAngle[j] = x;
		}
		//读取当前帧图像
		bool success = cap.read(curr);
		if (!success) { break; }
		//计算需要旋转的角度
		float needRotAngle = -(0 - thisFrameRotAngle[2])/180*pi;
		//旋转图像
		for (int y = 0; y < curr.rows; y++)
		{
			for (int x = 0; x < curr.cols; x++)
			{
				cv::Point2f current_pos(x, y);
				float point_x = (current_pos.x - w / 2) * cos(needRotAngle) - (current_pos.y - h / 2) * sin(needRotAngle) + curr.cols / 2;
				float point_y = (current_pos.x - w / 2) * sin(needRotAngle) + (current_pos.y - h / 2) * cos(needRotAngle) + curr.rows / 2;
				cv::Point2f original_point(point_x, point_y);
				cv::Point2i top_left((int)original_point.x, (int)original_point.y);
				//检测是不是在原图中采样
				if (top_left.x<0 || top_left.x>curr.cols - 2 || top_left.y<0 || top_left.y>curr.rows - 2)
				{
					continue;
				}
				//插值
				float dx = original_point.x - top_left.x;
				float dy = original_point.y - top_left.y;
				float weight_tl = (1.0 - dx) * (1.0 - dy);
				float weight_tr = (dx) * (1.0 - dy);
				float weight_bl = (1.0 - dx) * (dy);
				float weight_br = (dx) * (dy);
				for (int k = 0; k < 3; k++)
				{
					//uchar value = weight_tl * curr.at<cv::Vec3b>(top_left)[k] +weight_tr * curr.at<cv::Vec3b>(top_left.y, top_left.x + 1)[k] +weight_bl * curr.at<cv::Vec3b>(top_left.y + 1, top_left.x)[k] +weight_br * curr.at<cv::Vec3b>(top_left.x + 1, top_left.y + 1)[k];
					outMat.at<cv::Vec3b>(y, x)[k] = weight_tl * curr.at<cv::Vec3b>(top_left)[k] + weight_tr * curr.at<cv::Vec3b>(top_left.y, top_left.x + 1)[k] +weight_bl * curr.at<cv::Vec3b>(top_left.y + 1, top_left.x)[k] +weight_br * curr.at<cv::Vec3b>(top_left.y + 1, top_left.x + 1)[k];
					//outMat.at<cv::Vec3b>(y, x)[k] = curr.at<cv::Vec3b>(top_left)[k];
				}
			}
		}
		hconcat(curr, outMat, frame_outMat);
		if (frame_outMat.cols > 1920)
		{
			resize(frame_outMat, frame_outMat, Size(frame_outMat.cols / 2, frame_outMat.rows / 2));
		}
		out.write(frame_outMat);
		cout << "outframe: " << i << endl;
		waitKey(10);
	}
	//释放视频对象
	cap.release();
	out.release();
	destroyAllWindows();
	return 0;
}