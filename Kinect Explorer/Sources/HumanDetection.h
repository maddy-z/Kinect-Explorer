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

};

#endif