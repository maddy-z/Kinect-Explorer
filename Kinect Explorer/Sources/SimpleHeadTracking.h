#ifndef _SIMPLE_HEADTRACKING_H_
#define _SIMPLE_HEADTRACKING_H_

#include <XnOpenNI.h>
#include <XnCppWrapper.h>

#include <qimage.h>

#include <vector>

#include <opencv2\opencv.hpp>

class SimpleHeadTracking

{

public:
	
	static bool SimpleDepthHandlingFunc(QImage & destImgBuff, const xn::DepthGenerator & depthGen);

private:

	// 
	// Depth Data Preprocessing
	// 
	
	static bool FillShadow(const XnDepthPixel * srcDepthData, XnDepthPixel * destDepthData, int nXRes, int nYRes, XnDepthPixel maxDeviceDepth);
	static bool SmoothDepthData(const XnDepthPixel * srcDepthData, 
								XnDepthPixel * destDepthData, 
								int nXRes, 
								int nYRes, 
								int smoothWidth, 
								int smoothHeight);

	// 
	// Head Detection
	// 

	// TODO:
	// static int FindLocalMinimumOfRow(const cv::Mat & srcDepthMat);

	static int FindLocalMinimumOfRow(const XnDepthPixel * srcDepthData, int nXRes, int rowIndex, const xn::DepthGenerator & depthGen);
	static bool FindHeadCoordinate(const XnDepthPixel * srcDepthData, 
								   int nXRes, int nYRes, 
								   int & rowIndex, int & columnIndex, 
								   const xn::DepthGenerator & depthGen);

	static bool IsPossiblyOnHeadFast(const XnDepthPixel * srcDepthData, int nXRes, 
									 int rowIndex, int colIndex,
									 int & leftDeltaLen, int & rightDeltaLen,
									 const xn::DepthGenerator & depthGen);

	static bool IsPossiblyOnHead(const XnDepthPixel * srcDepthData, int nXRes, 
								 int rowIndex, int colIndex, 
								 const xn::DepthGenerator & depthGen);

	// 
	// Get Local Minimum Depth Map
	// 
	
	static bool GetLocalMinimumOfDepthMap(const XnDepthPixel * srcDepthData, std::vector<int> & minArray, const xn::DepthGenerator & depthGen);

};

#endif