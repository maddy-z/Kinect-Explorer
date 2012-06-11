#include <cstdio>
#include <cstdlib>

#include <string>
#include <sstream>

#include "GlobalUtility.h"

#include <XnOpenNI.h>
#include <XnCppWrapper.h>

namespace GlobalUtility

{

#define		OPENNI_MAX_DEPTH		10000

std::string DoubleToString(double value)
{
	std::stringstream ssTmp;
	std::string str;

	ssTmp << value; ssTmp >> str;

	return str;
}

std::string IntToString(int value)
{
	std::stringstream ssTmp;
	std::string str;

	ssTmp << value; ssTmp >> str;

	return str;
}

bool ConvertDepthToColor(const cv::Mat & depthMat, cv::Mat & colorMat, int maxDepth)
{
	assert (depthMat.size() == colorMat.size());
	assert (depthMat.type() == CV_16UC1 && colorMat.type() == CV_8UC3);

	if (depthMat.data == NULL || colorMat.data == NULL) {
		return false;
	}

	float depthHist[OPENNI_MAX_DEPTH];
	xnOSMemSet(depthHist, 0, OPENNI_MAX_DEPTH * sizeof(float));
	
	int imgHeight = depthMat.size().height, imgWidth = depthMat.size().width, depthRowStep = depthMat.step;

	unsigned int pointsNumber = 0;
	const uchar * pDepthData = depthMat.data;

	unsigned short depth;

	for (XnUInt y = 0; y < imgHeight; ++y) 
	{
		for (XnUInt x = 0; x < imgWidth; ++x) 
		{
			depth = *((const unsigned short *)(pDepthData + y * depthRowStep + x * 2));

			if ( depth ) {
				++depthHist[depth];
				++pointsNumber;
			}
		}
	}
	
	for (int i = 1; i < OPENNI_MAX_DEPTH; ++i) {
		depthHist[i] += depthHist[i-1];
	}

	if (pointsNumber > 0)
	{
		for (int i = 1; i < OPENNI_MAX_DEPTH; ++i) {
			depthHist[i] = (unsigned int)(256 * (1.0f - (depthHist[i] / pointsNumber)));
		}
	}
	
	int colorRowStep = colorMat.step;
	uchar * pColorData = colorMat.data;
	uchar * colorData;

	for (XnUInt y = 0; y < imgHeight; ++y) 
	{
		// linePtr += colorMat.step;

		for (XnUInt x = 0; x < imgWidth; ++x) 
		{
			depth = *((const unsigned short *)(pDepthData + y * depthRowStep + x * 2));
			colorData = pColorData + y * colorRowStep + x * 3;

			if ( depth )
			{
				colorData[0] = 0;
				colorData[1] = depthHist[depth];
				colorData[2] = depthHist[depth];
			}
			else {
				colorData[0] = depth;
				colorData[1] = depth;
				colorData[2] = depth;
			}
		}
	}

	return true;

	//unsigned short depth;

	//for (int y = 0; y < depthMat.size().height; ++y) 
	//{
	//	for (int x = 0; x < depthMat.size().width; ++x) 
	//	{
	//		depth = *((unsigned short *)(depthData + y * depthMat.step + x * 2));

	//		if (depth > maxDepth) depth = maxDepth;
	//		if (depth) depth = (maxDepth - depth) * 255 / maxDepth + 1;

	//		assert (depth >= 0 && depth <= 255);

	//		uchar * imagePtr = (uchar *)(colorData + y * colorMat.step + x * 3);
	//		
	//		imagePtr[0] = depth;
	//		imagePtr[1] = depth;
	//		imagePtr[2] = depth;
	//		// imagePtr[3] = 0xff;
	//	}
	//}

}

}		// End of Namespace -- GlobalUtility
