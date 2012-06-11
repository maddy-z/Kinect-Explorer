#include <XnOpenNI.h>
#include <XnCppWrapper.h>

#include <opencv2\opencv.hpp>

#include "GlobalUtility.h"
#include "HumanDetection.h"
#include "KinectHoleFiller.h"

#include "qdebug.h"
#include "qimage.h"

// 
// Public Member Function
// 

int HumanDetection::thresholdValue_1 = 200;
int HumanDetection::thresholdValue_2 = 200;

bool HumanDetection::HumanDetectingAlgo(QImage & destImgBuff, const xn::DepthGenerator & depthGen)
{
	qDebug("Entering:\tbool HumanDetection::HumanDetectingAlgo");

	// 
	// Invalid Depth Stream Input, then return an empty QImage
	// 

	bool result;

	xn::DepthMetaData depthMD;
	depthGen.GetMetaData(depthMD);

	const XnDepthPixel * srcDepthData = depthMD.Data();
	if (srcDepthData == NULL) 
	{
		qDebug("Leaving:\tbool HumanDetection::HumanDetectingAlgo");
		return false;
	}

	int nXRes = depthMD.XRes();
	int nYRes = depthMD.YRes();
	
	// 
	// Step 0:	Copy Depth Pixel to cv::Mat
	// 
	qDebug("Step 0:	Copy Depth Pixel to cv::Mat");
	cv::Mat depthMat(nYRes, nXRes, CV_16UC1);
	result = GlobalUtility::CopyDepthRawBufToCvMat16u(srcDepthData, depthMat);
	
	// 
	// Step 1:	Fill Shadows of Depth Data
	// 
	qDebug("Step 1:	Fill Shadows of Depth Data");
	result = KinectHoleFiller::NearestNeighborhoodHoleFilling(depthMat, depthMat);

	// 
	// Step 2:	Median Blur
	// 
	qDebug("Step 2:	Median Filtering of Depth Data");
	medianBlur(depthMat, depthMat, 3);

	cv::Mat depthMat8u(nYRes, nXRes, CV_8UC1);
	result = GlobalUtility::CopyCvMat16uTo8u(depthMat, depthMat8u);

	// 
	// Step 3:	Canny Operation
	// 
	qDebug("Step 3:	Canny Operation of Depth Data");
	cv::Mat cannyedDepthMat(nYRes, nXRes, CV_8UC1);
	cv::Canny(depthMat8u, cannyedDepthMat, HumanDetection::thresholdValue_1, HumanDetection::thresholdValue_2);

	// 
	// Final Step:	Copy cv::Mat to QImage
	// 

	// result = GlobalUtility::ConvertDepthCvMat16uToYellowQImage(depthMat, destImgBuff);
	// result = GlobalUtility::ConvertDepthCvMat8uToYellowQImage(depthMat8u, destImgBuff);
	// result = GlobalUtility::ConvertDepthCvMat8uToGrayQImage(depthMat8u, destImgBuff);
	
	result = GlobalUtility::CopyCvMat8uToQImage(cannyedDepthMat, destImgBuff);

	qDebug("Leaving:\tbool HumanDetection::HumanDetectingAlgo");

	return result;
}

bool HumanDetection::HumanDetectingAlgoWithoutProcessing(QImage & destImgBuff, const xn::DepthGenerator & depthGen)
{
	qDebug("Entering:\tbool HumanDetection::HumanDetectingAlgo");

	// 
	// Invalid Depth Stream Input, then return an empty QImage
	// 

	bool result;

	xn::DepthMetaData depthMD;
	depthGen.GetMetaData(depthMD);

	const XnDepthPixel * srcDepthData = depthMD.Data();
	if (srcDepthData == NULL) {
		qDebug("Leaving:\tbool HumanDetection::HumanDetectingAlgo");
		return false;
	}

	int nXRes = depthMD.XRes(), nYRes = depthMD.YRes();
	
	cv::Mat depthMat(nYRes, nXRes, CV_16UC1);
	result = GlobalUtility::CopyDepthRawBufToCvMat16u(srcDepthData, depthMat);
	result = GlobalUtility::ConvertDepthCvMat16uToYellowQImage(depthMat, destImgBuff);

	qDebug("Leaving:\tbool HumanDetection::HumanDetectingAlgo");

	return result;
}

void HumanDetection::setCannyThresholdValue(int threshold_1, int threshold_2)
{
	HumanDetection::thresholdValue_1 = threshold_1;
	HumanDetection::thresholdValue_2 = threshold_2;
}

// 
// Private Member Function
// 
