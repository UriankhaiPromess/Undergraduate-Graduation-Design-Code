#include <opencv2/videostab/outlier_rejection.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videostab.hpp>
#include <string>
#include <iostream>

//#pragma comment(lib, "opencv_world455.lib")
#pragma comment(lib, "opencv_core455.lib")
#pragma comment(lib, "opencv_videoio455.lib")
#pragma comment(lib, "opencv_highgui455.lib")
#pragma comment(lib, "opencv_features2d455.lib")
#pragma comment(lib, "opencv_videostab455.lib")

using namespace std;
using namespace cv;
using namespace cv::videostab;

string inputPath = "D:\\unityOutputs\\OriginOutputs\\media\\movie_rgb.mp4";
string outputPath = "D:\\unityOutputs\\OriginOutputs\\media\\suijichouyang001.avi";//001 must

// 视频稳定输出
void videoOutput(Ptr<IFrameSource> stabFrames, string outputPath)
{
	VideoWriter writer;
	cv::Mat stabFrame;
	int nframes = 0;
	// 设置输出帧率
	double outputFps = 12;
	// 遍历搜索视频帧
	while (!(stabFrame = stabFrames->nextFrame()).empty())
	{
		nframes++;
		// 输出视频稳定帧
		if (!outputPath.empty())
		{
			if (!writer.isOpened())
				writer.open(outputPath, VideoWriter::fourcc('M', 'J', 'P', 'G'), outputFps, stabFrame.size());

			writer << stabFrame;
		}
		imshow("stabFrame", stabFrame);
		// esc键退出
		char key = static_cast<char>(waitKey(100));
		if (key == 27)
		{
			cout << endl;
			break;
		}
	}
	writer.release();
	std::cout << "nFrames: " << nframes << endl;
	std::cout << "finished " << endl;
}

void cacStabVideo(Ptr<IFrameSource> stabFrames, string srcVideoFile)
{
	try
	{

		Ptr<VideoFileSource> srcVideo = makePtr<VideoFileSource>(inputPath);
		cout << "frame count: " << srcVideo->count() << endl;

		// 运动估计
		double estPara = 0.1;
		Ptr<MotionEstimatorRansacL2> est =
			makePtr<MotionEstimatorRansacL2>(MM_AFFINE);

		// Ransac参数设置
		RansacParams ransac = est->ransacParams();
		ransac.size = 3;
		ransac.thresh = 5;
		ransac.eps = 0.5;

		// Ransac计算
		est->setRansacParams(ransac);
		est->setMinInlierRatio(estPara);

		// Fast特征检测
		Ptr<FastFeatureDetector> feature_detector =
			FastFeatureDetector::create();

		// 运动估计关键点匹配
		Ptr<KeypointBasedMotionEstimator> motionEstBuilder =
			makePtr<KeypointBasedMotionEstimator>(est);

		// 设置特征检测器
		motionEstBuilder->setDetector(feature_detector);
		//Ptr<IOutlierRejector> outlierRejector = makePtr<NullOutlierRejector>();
		Ptr<IOutlierRejector> outlierRejector = makePtr<NullOutlierRejector>();
		motionEstBuilder->setOutlierRejector(outlierRejector);
		// 3-Prepare the stabilizer
		StabilizerBase* stabilizer = 0;
		// first, prepare the one or two pass stabilizer
		bool isTwoPass = 1;
		int radius_pass = 15;
		if (isTwoPass)
		{
			// with a two pass stabilizer
			bool est_trim = true;
			TwoPassStabilizer* twoPassStabilizer = new TwoPassStabilizer();
			twoPassStabilizer->setEstimateTrimRatio(est_trim);
			twoPassStabilizer->setMotionStabilizer(
				makePtr<GaussianMotionFilter>(radius_pass));
			stabilizer = twoPassStabilizer;
		}
		else
		{
			// with an one pass stabilizer
			OnePassStabilizer* onePassStabilizer = new OnePassStabilizer();
			onePassStabilizer->setMotionFilter(
				makePtr<GaussianMotionFilter>(radius_pass));
			stabilizer = onePassStabilizer;
		}

		// second, set up the parameters
		int radius = 15;
		double trim_ratio = 0.1;
		bool incl_constr = false;
		stabilizer->setFrameSource(srcVideo);
		stabilizer->setMotionEstimator(motionEstBuilder);
		stabilizer->setRadius(radius);
		stabilizer->setTrimRatio(trim_ratio);
		stabilizer->setCorrectionForInclusion(incl_constr);
		stabilizer->setBorderMode(BORDER_REPLICATE);
		// cast stabilizer to simple frame source interface to read stabilized frames
		stabFrames.reset(dynamic_cast<IFrameSource*>(stabilizer));
		// 4-videoOutput the stabilized frames. The results are showed and saved.
		videoOutput(stabFrames, outputPath);
	}

	catch (const exception& e)
	{
		cout << "error: " << e.what() << endl;
		stabFrames.release();
	}
}

//对于实时视频流，使用缓存50帧来处理
int main(int argc, char* argv[])
{
	Ptr<IFrameSource> stabFrames;
	// 输入输出视频准备

	cacStabVideo(stabFrames, inputPath);
	stabFrames.release();
	destroyAllWindows();

	//getchar();

	return 0;
}