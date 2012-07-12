#ifndef _GLOBAL_UTILITY_H_
#define _GLOBAL_UTILITY_H_

#include "OpenNiUtility.h"
#include "OpenCvUtility.h"

#include <opencv2\opencv.hpp>

#include "qimage.h"
#include "qdebug.h"

#include <string>

namespace GlobalUtility

{
	// 
	// Common Utilities
	// 

	template<typename T>
	inline std::string ValueToString ( T value )
	{
		std::stringstream ssTmp;
		std::string str;

		ssTmp << value; 
		ssTmp >> str;

		return str;
	}

	//
	// Conversion between cv::Mat and XnDepthPixel Array or XnRGB24Pixel Array
	// 

	bool CopyCvMat16uToDepthRawBuf  ( const cv::Mat & srcDepthMat, XnDepthPixel * destDepthData );
	bool CopyDepthRawBufToCvMat16u  ( const XnDepthPixel * srcDepthData, cv::Mat & destDepthMat );
	bool CopyColorRawBufToCvMat8uc3 ( const XnRGB24Pixel * srcColorData, cv::Mat & destColorMat );

	bool CopyCvMat8uToQImage ( const cv::Mat & srcMat, QImage & destImg );
	bool CopyQImageToCvMat8uc3 ( const QImage & qImg, cv::Mat & destMat );

	// 
	// cv::Mat Format Conversion
	// 

	bool CopyCvMat16uTo8u ( const cv::Mat & srcCvMat, cv::Mat & destCvMat );
	bool CopyCvMat8uTo16u ( const cv::Mat & srcCvMat, cv::Mat & destCvMat );
	
	// 
	// Ways to Render Depth Data with Different Formats from Kinect Sensor
	// 

	bool ConvertDepthCvMat16uToYellowQImage ( const cv::Mat & srcDepthMat, QImage & destImg );
	bool ConvertDepthCvMat16uToGrayQImage ( const cv::Mat & srcDepthMat, QImage & destImg );
	bool ConvertDepthCvMat16uToColorfulQImage ( const cv::Mat & srcDepthMat, QImage & destImg, XnDepthPixel maxDeviceDepth );
	
	bool ConvertDepthCvMat8uToYellowQImage ( const cv::Mat & srcDepthMat, QImage & destImg );
	bool ConvertDepthCvMat8uToGrayQImage ( const cv::Mat & srcDepthMat, QImage & destImg );

	bool ConvertDepthCvMat16uToYellowCvMat ( const cv::Mat & srcDepthMat, cv::Mat & destMat );
	bool ConvertDepthCvMat16uToGrayCvMat ( const cv::Mat & srcDepthMat, cv::Mat & destMat );
	bool ConvertDepthCvMat16uToColorfulCvMat ( const cv::Mat & srcDepthMat, cv::Mat & destMat, XnDepthPixel maxDeviceDepth );

	// 
	// Easy Handling Methods for processing Depth Data ( In the form of cv::Mat )
	// 
	
	bool ConvertCvMat16uByThresholdValue ( const cv::Mat & srcMat, cv::Mat & destMat, double thresholdValue, int newValue = 0, bool isThresholdUnknownDepth = false );

}

#endif