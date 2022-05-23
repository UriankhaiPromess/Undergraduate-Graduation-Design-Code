#include <iostream>
#include <fstream>
#include <cstdlib>
#include <sstream>
#include <cmath>
#include <vector>
#include <cassert>
#include <openCV2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace cv;

int main()
{
	ifstream rotData("D:\\unityOutputs\\OriginOutputs\\zitaidata.txt");
	string line;
	int numtmp = 0;
	vector<float> frameAnglex(715);
	vector<float> frameAngley(715);
	vector<float> frameAnglez(715);
	vector<float> frameMovex(715);
	vector<float> frameMovey(715);
	vector<float> frameMovez(715);
	vector<float> frameMovexSmooth(715);
	vector<float> frameMoveySmooth(715);
	vector<float> frameMovezSmooth(715);

	vector<float> frameAngleySmooth(713);
	vector<float> needFrameAngel(3);
	vector<float> needFrameRot1(3),needFrameRot2(3);
	Eigen::MatrixXf cameraCoordinate(4, 1);
	Eigen::MatrixXf newCameraCoordinate(4, 1);
	Eigen::MatrixXf MrotateZ1(4, 4);
	Eigen::MatrixXf MrotateX1(4, 4);
	Eigen::MatrixXf MrotateY1(4, 4);
	Eigen::MatrixXf MrotateZ2(4, 4);
	Eigen::MatrixXf MrotateX2(4, 4);
	Eigen::MatrixXf MrotateY2(4, 4);
	Eigen::MatrixXf Mmove1(4, 4);
	Eigen::MatrixXf Mmove2(4, 4);
	Eigen::MatrixXf Mtrans(4, 4);


	VideoCapture cap_rgb("D:\\unityOutputs\\OriginOutputs\\media\\movie_rgb.mp4");
	VideoCapture cap_depth("D:\\unityOutputs\\OriginOutputs\\media\\movie_depth.mp4");

	int w = int(cap_rgb.get(CAP_PROP_FRAME_WIDTH));
	int h = int(cap_rgb.get(CAP_PROP_FRAME_HEIGHT));
	double fps = cap_rgb.get(CAP_PROP_FPS);

	VideoWriter out;
	out.open("D:\\unityOutputs\\OriginOutputs\\programOutPuts\\ShenDuChongJian001.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, Size(w, h / 2), true);

	Mat currMat_rgb, frameOutMat;
	Mat currMat_depth = cv::Mat::zeros(cv::Size(w, h), CV_16UC1);
	Mat currOutMat = cv::Mat::zeros(cv::Size(w, h), CV_8UC3);

	float pi = 3.1415892653;
	int fileRows = 709;
	float FOV_near = 0.3;
	float FOV_far = 100;
	float aspect = (float)w / (float)h;

	//cout << int(cap_rgb.get(CAP_PROP_FRAME_COUNT)) << " " << int(cap_depth.get(CAP_PROP_FRAME_COUNT)) << endl;

	for (int i = 0; i < fileRows; i++)
	{
		getline(rotData, line);
		stringstream ss(line);
		float x;
		ss >> numtmp;
		ss >> x;
		frameAnglex[i] = x;
		ss >> x;
		frameAngley[i] = x;
		ss >> x;
		frameAnglez[i] = x;
		ss >> x;
		frameMovex[i] = x;
		ss >> x;
		frameMovey[i] = x;
		ss >> x;
		frameMovez[i] = x;
	}
	//对数据进行中值滤波
	for (int i = 0; i < 707; i++)
	{
		if (i < 2)
		{
			frameAngleySmooth[i] = frameAngley[i];
			frameMovexSmooth[i] = frameMovex[i];
			frameMoveySmooth[i] = frameMovey[i];
			frameMovezSmooth[i] = frameMovez[i];
		}
		else
		{
			frameAngleySmooth[i] = (frameAngley[i - 2] + frameAngley[i - 1] + frameAngley[i] + frameAngley[i + 1] + frameAngley[i + 2]) / 5.0;
			frameMovexSmooth[i] = (frameMovex[i - 2] + frameMovex[i - 1] + frameMovex[i] + frameMovex[i + 1] + frameMovex[i + 2]) / 5.0;
			frameMoveySmooth[i] = (frameMovey[i - 2] + frameMovey[i - 1] + frameMovey[i] + frameMovey[i + 1] + frameMovey[i + 2]) / 5.0;
			frameMovezSmooth[i] = (frameMovez[i - 2] + frameMovez[i - 1] + frameMovez[i] + frameMovez[i + 1] + frameMovez[i + 2]) / 5.0;
		}
	}
	//逐帧处理
	for (int i = 0; i < 707; i++)
	{
		//初始化为黑色
		currOutMat = cv::Mat::zeros(cv::Size(w, h), CV_8UC3);
		//读取当前帧rgb图像、深度图
		cap_rgb.read(currMat_rgb);
		cap_depth.read(currMat_depth);
		//计算所应旋转的角度
			needFrameRot1[0] = (frameAnglex[i] - 0) / 180 * pi;
			needFrameRot1[1] = (frameAngley[i] - 0) / 180 * pi;
			needFrameRot1[2] = (frameAnglez[i] - 0) / 180 * pi;
			needFrameRot2[0] = 0;
			needFrameRot2[1] = -frameAngleySmooth[i] / 180 * pi;
			needFrameRot2[2] = 0;

		//输入各变换矩阵
		Mmove1 << 1, 0, 0, frameMovex[i],
			0, 1, 0, frameMovey[i],
			0, 0, 1, frameMovez[i],
			0, 0, 0, 1;
		MrotateZ1 << cos(needFrameRot1[2]), -sin(needFrameRot1[2]), 0, 0,
			sin(needFrameRot1[2]), cos(needFrameRot1[2]), 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;
		MrotateX1 << 1, 0, 0, 0,
			0, cos(needFrameRot1[0]), -sin(needFrameRot1[0]), 0,
			0, sin(needFrameRot1[0]), cos(needFrameRot1[0]), 0,
			0, 0, 0, 1;
		MrotateY1 << cos(needFrameRot1[1]), 0, sin(needFrameRot1[1]), 0,
			0, 1, 0, 0,
			-sin(needFrameRot1[1]), 0, cos(needFrameRot1[1]), 0,
			0, 0, 0, 1;
		Mmove2 << 1, 0, 0, -frameMovexSmooth[i],
			0, 1, 0, -frameMoveySmooth[i],
			0, 0, 1, -frameMovezSmooth[i],
			0, 0, 0, 1;
		MrotateZ2 << cos(needFrameRot2[2]), -sin(needFrameRot2[2]), 0, 0,
			sin(needFrameRot2[2]), cos(needFrameRot2[2]), 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;
		MrotateX2 << 1, 0, 0, 0,
			0, cos(needFrameRot2[0]), -sin(needFrameRot2[0]), 0,
			0, sin(needFrameRot2[0]), cos(needFrameRot2[0]), 0,
			0, 0, 0, 1;
		MrotateY2 << cos(needFrameRot2[1]), 0, sin(needFrameRot2[1]), 0,
			0, 1, 0, 0,
			-sin(needFrameRot2[1]), 0, cos(needFrameRot2[1]), 0,
			0, 0, 0, 1;
		//Mtrans = MrotateZ2 * MrotateX2 * MrotateY2 * Mmove2 * Mmove1 * MrotateY1 * MrotateX1 * MrotateZ1;
		Mtrans = MrotateZ2 * MrotateX2 * MrotateY2 * MrotateY1 * MrotateX1 * MrotateZ1;
		//逐像素处理
		for (int y = 0; y < currMat_depth.rows; y++)
		{
			for (int x = 0; x < currMat_rgb.cols; x++)
			{
				cv::Point2f current_pos(x, y);
				float d = (float)currMat_depth.ptr<ushort>(y)[x] / 65536;//d是深度图float值,白为0表示远黑为1表示近
				//计算在世界坐标系下改点的位置并写入originCoordinate
				float origin_z = (float)1.0 / ((FOV_near - FOV_far) / (FOV_near * FOV_far) * d + 1.0 / FOV_near);
				float origin_y = ((float)h / 2 - current_pos.y) / ((float)h / 2) * origin_z * tan(30.0 / 180.0 * pi);
				float origin_x = (current_pos.x-(float)w/2) / ((float)w / 2) * origin_z * tan(30.0 / 180.0 * pi) * aspect;
				//cout << x<<" "<<y<<" " << origin_x << " " << origin_y << " " << origin_z << endl;
				cameraCoordinate << origin_x,
									origin_y,
									origin_z,
									1;
				//旋转回世界坐标,再旋转到新的相机局部坐标
				
				//cout << MrotateZ2 * MrotateX2 * MrotateY2 * (MrotateZ1 * MrotateX1 * MrotateY1).inverse() << endl;
				newCameraCoordinate = Mtrans * cameraCoordinate;
				//cout << newCameraCoordinate <<cameraCoordinate<<endl;
				//将新的摄像机坐标转换为屏幕坐标
				//cout << newCameraCoordinate(1, 1) << " " << newCameraCoordinate(2, 1) << " " << newCameraCoordinate(3, 1) << endl;
				float newScreenPointx = w / 2 + newCameraCoordinate(0, 0) / (newCameraCoordinate(2, 0) * tan(30.0 / 180.0 * pi) * aspect) * w / 2;
				float newScreenPointy = h / 2 - newCameraCoordinate(1,0) / (newCameraCoordinate(2, 0) * tan(30.0 / 180.0 * pi)) * h / 2;
				//cout << newCameraCoordinate(2, 1) / (newCameraCoordinate(3, 1) * tan(30.0 / 180.0 * pi)) * h / 2 << endl;
				//cout << newCameraCoordinate(2, 1) << " " << (newCameraCoordinate(3, 1) * tan(30.0 / 180.0 * pi)) << endl;
				//cout << newCameraCoordinate << endl;
				cv::Point2i top_left((int)newScreenPointx, (int)newScreenPointy);
				//cout << newScreenPointx << " " << newScreenPointy << endl;
				if (top_left.x<1 || top_left.x>w - 2 || top_left.y<1 || top_left.y>h - 2) { continue; }
				else
				{
					//采样
					/*float dx = newScreenPointx - top_left.x;
					float dy = newScreenPointy - top_left.y;
					float weight_tl = (1.0 - dx) * (1.0 - dy);
					float weight_tr = (dx) * (1.0 - dy);
					float weight_bl = (1.0 - dx) * (dy);
					float weight_br = (dx) * (dy);*/
					for (int k = 0; k < 3; k++)
					{
						/*currOutMat.at<cv::Vec3b>(top_left)[k] += weight_tl * currMat_rgb.at<cv::Vec3b>(y, x)[k];
						currOutMat.at<cv::Vec3b>(top_left.y,top_left.x+1)[k] += weight_tr * currMat_rgb.at<cv::Vec3b>(y, x)[k];
						currOutMat.at<cv::Vec3b>(top_left.y+1,top_left.x)[k] += weight_bl * currMat_rgb.at<cv::Vec3b>(y, x)[k];
						currOutMat.at<cv::Vec3b>(top_left.y+1,top_left.x+1)[k] += weight_br * currMat_rgb.at<cv::Vec3b>(y, x)[k];*/
						currOutMat.at<cv::Vec3b>(top_left)[k] = currMat_rgb.at<cv::Vec3b>(y, x)[k];
					}
				}
			}
		}
		//写入
		//将原图扩大为1.36倍，然后截取原图尺寸相等大小区域
		//Mat T = getRotationMatrix2D(Point2f(currOutMat.cols / 2, currOutMat.rows / 2), 0, 1.36);
		//仿射变换
		//warpAffine(currOutMat, currOutMat, T, currOutMat.size());
		hconcat(currMat_rgb, currOutMat, frameOutMat);
		if (frameOutMat.cols > 1920)
		{
			resize(frameOutMat, frameOutMat, Size(frameOutMat.cols / 2, frameOutMat.rows / 2));
		}
		out.write(frameOutMat);
		cout << "outframe: " << i << endl;
		waitKey(10);
	}
	//释放
	cap_rgb.release();
	cap_depth.release();
	out.release();
	destroyAllWindows();
	return 0;
}
