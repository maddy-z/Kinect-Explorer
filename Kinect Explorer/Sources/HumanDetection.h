#ifndef _HUMAN_DETECTION_H_
#define _HUMAN_DETECTION_H_

#include <XnOpenNI.h>
#include <XnCppWrapper.h>

#include <qimage.h>

#include <opencv2\opencv.hpp>

class HumanDetection
{

public:
	
	static bool HumanDetectingAlgo(QImage & destImgBuff, const xn::DepthGenerator & depthGen);
	static bool HumanDetectingAlgoWithoutProcessing(QImage & destImgBuff, const xn::DepthGenerator & depthGen);

	static void setCannyThresholdValue(int threshold_1, int threshold_2);

	static int thresholdValue_1;
	static int thresholdValue_2;

private:
	
	// static bool NearestNeighborFiltering(const cv::Mat & srcDepthMat, cv::Mat & destDepthMat);

	// static bool ConvertDepth2ColorfulQImage(const cv::Mat & srcDepthMat, QImage & destImg, /*unsigned int imgWidth, unsigned int imgHeight,*/ XnDepthPixel maxDeviceDepth);
	
	// static bool CopyDepthRawData2CvMat(const XnDepthPixel * srcDepthData, cv::Mat & destDepthMat);
	// static bool CopyColorRawData2CvMat(const XnRGB24Pixel * srcColorData, cv::Mat & destColorMat);

};

#endif