#include "OpenNIUtility.h"

#include <XnCppWrapper.h>

double OpenNIUtility::CalcAverageDepth(const xn::DepthMetaData & depthMD)
{
	const XnDepthPixel * pDepth = depthMD.Data();

	double totalDepthValue = 0.0;
	int totalDepthPixelCount = 0;

	for (int i = 0; i < depthMD.XRes() * depthMD.YRes(); ++i, ++pDepth) 
	{
		if (*pDepth != 0) {
			totalDepthValue += (*pDepth);
			totalDepthPixelCount++;
		}
	}

	return (totalDepthValue) / (double)(totalDepthPixelCount);
}
double OpenNIUtility::CalcSmallestDepth(const xn::DepthMetaData & depthMD)
{
	int nXRes = depthMD.XRes();
	int nYRes = depthMD.YRes();

	unsigned short value = 0xffff;
	const XnDepthPixel * pData = depthMD.Data();
	for (int i = 0; i < nXRes * nYRes; ++i, ++pData) 
	{
		if (*pData == 0) {
			continue;
		}
		
		if (value > *pData) {
			value = *pData;
		}
	}

	return value;
}
double OpenNIUtility::CalcBiggestDepth(const xn::DepthMetaData & depthMD)
{
	int nXRes = depthMD.XRes();
	int nYRes = depthMD.YRes();

	unsigned short value = 0x0;
	const XnDepthPixel * pData = depthMD.Data();
	for (int i = 0; i < nXRes * nYRes; ++i, ++pData) 
	{
		if (value <= *pData)
			value = *pData;
	}

	return value;	
}