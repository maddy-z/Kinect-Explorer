#ifndef _SIMPLE_HEADTRACKING_H_
#define _SIMPLE_HEADTRACKING_H_

#include <XnOpenNI.h>
#include <XnCppWrapper.h>

#include <qimage.h>
#include <vector>

#include <opencv2\opencv.hpp>

// 
// Simple Head Tracking
// 

class SimpleHeadTracking

{

public:
	
	static bool SimpleDepthHandlingFunc (	QImage & destImgBuff, const xn::DepthGenerator & depthGen );
	static bool SimpleDepthHandlingFunc2(	QImage & destImgBuff, const xn::DepthGenerator & depthGen, XnPoint3D & headImgCoor );
	static bool SimpleDepthHandlingFunc2(	cv::Mat & destImgBuff, const xn::DepthGenerator & depthGen, XnPoint3D & headImgCoor );

	static bool FindHeadCoordinate		(	const XnDepthPixel * srcDepthData, 
											const int nXRes, 
											const int nYRes, 
											const double fXToZ, 
											const double fYToZ, 
											int & rowIndex, 
											int & columnIndex, 
											const xn::DepthGenerator & depthGen );

	// 
	// Get Local Minimum Depth Map
	// 
	
	static bool GetLocalMinimumOfDepthMap ( const XnDepthPixel * srcDepthData, 
											const int nXRes, 
											const int nYRes,
											const double fXToZ,
											const double fYToZ, 
											const xn::DepthGenerator & depthGen, 
											std::vector<int> & minArray );

private:

	// 
	// Depth Data Preprocessing
	// 
	
	static bool FillShadow		(	const XnDepthPixel * srcDepthData, XnDepthPixel * destDepthData, int nXRes, int nYRes, XnDepthPixel maxDeviceDepth );
	static bool SmoothDepthData (	const XnDepthPixel * srcDepthData, 
									XnDepthPixel * destDepthData, 
									int nXRes, 
									int nYRes, 
									int smoothWidth, 
									int smoothHeight 
									);

	// 
	// Head Detection
	// 

	// TODO:
	// static int FindLocalMinimumOfRow(const cv::Mat & srcDepthMat);

	static int FindLocalMinimumOfRow (	const XnDepthPixel * srcDepthData, 
										const int nXRes, 
										const int nYRes, 
										const double fXToZ, 
										const double fYToZ, 
										const int rowIndex, 
										const xn::DepthGenerator & depthGen );

	static bool IsPossiblyOnHeadFast (	const XnDepthPixel * srcDepthData, 
										const int nXRes, 
										const int nYRes,
										const double fXToZ, 
										const double fYToZ,
										const int rowIndex, 
										const int colIndex,
										int & leftDeltaLen, 
										int & rightDeltaLen,
										const xn::DepthGenerator & depthGen );

	static bool IsPossiblyOnHead	 (	const XnDepthPixel * srcDepthData, 
										int nXRes, 
										int rowIndex, 
										int colIndex, 
										const xn::DepthGenerator & depthGen );

};

#endif