#include "OpenNiUtility.h"

#include <XnOpenNI.h>
#include <XnCppWrapper.h>

namespace OpenNiUtility

{	
	
// Start Of Namespace (OpenNiUtility)

double CalcAverageDepth(const XnDepthPixel * srcDepthData, unsigned int srcRow, unsigned int srcCol)
{
	if (srcDepthData == NULL) {
		return (-1);
	}

	double totalDepthValue = 0.0;
	int totalDepthPixelCount = 0;

	const XnDepthPixel * pData = srcDepthData;
	int totalPixelNum = srcRow * srcCol;

	for (int i = 0; i < totalPixelNum; ++i, ++pData) 
	{
		if (*pData) {
			totalDepthValue += (*pData);
			totalDepthPixelCount++;
		}
	}

	return (totalDepthValue) / (double)(totalDepthPixelCount);
}

double CalcSmallestDepth(const XnDepthPixel * srcDepthData, unsigned int srcRow, unsigned int srcCol)
{
	if (srcDepthData == NULL) {
		return (-1);
	}

	unsigned short value = 0xffff;

	const XnDepthPixel * pData = srcDepthData;
	int totalPixelNum = srcRow * srcCol;
	
	for (int i = 0; i < totalPixelNum; ++i, ++pData) 
	{	
		if (*pData > 0 && value > *pData) {
			value = *pData;
		}
	}

	return value;
}

double CalcBiggestDepth(const XnDepthPixel * srcDepthData, unsigned int srcRow, unsigned int srcCol)
{
	if (srcDepthData == NULL) {
		return (-1);
	}

	unsigned short value = 0x0;
	
	const XnDepthPixel * pData = srcDepthData;
	int totalPixelNum = srcRow * srcCol;

	for (int i = 0; i < totalPixelNum; ++i, ++pData) 
	{
		if (value < *pData)
			value = *pData;
	}

	return value;	
}

}	// End of Namespace (OpenNiUtility)