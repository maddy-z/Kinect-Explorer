#include <cmath>

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

	for ( int i = 0; i < totalPixelNum; ++i, ++pData ) {
		if ( value < *pData ) {
			value = *pData;
		}
	}

	return value;	
}

bool ConvertProjectiveToRealWorld ( const int count, 
									const int nXRes, 
									const int nYRes, 
									const double fXToZ, 
									const double fYToZ, 
									const XnPoint3D * proj, 
									XnPoint3D * real )
{
	assert ( count > 0 && nXRes > 0 && nYRes > 0 );
	
	if ( proj == NULL || real == NULL ) {
		return false;
	}

	for ( int i = 0; i < count; ++i ) {
		XnDouble fNormalizedX = (proj[i].X / (double)(nXRes) - 0.5);
		real[i].X = (XnFloat)(fNormalizedX * proj[i].Z * fXToZ);

		XnDouble fNormalizedY = (0.5 - proj[i].Y / (double)(nYRes));
		real[i].Y = (XnFloat)(fNormalizedY * proj[i].Z * fYToZ);

		real[i].Z = proj[i].Z;
	}

	return true;
}

bool ConvertRealWorldToProjective ( const int count, 
									const int nXRes, 
									const int nYRes, 
									const double fXToZ, 
									const double fYToZ, 
									const XnPoint3D * real, 
									XnPoint3D * proj )
{
	assert ( count > 0 && nXRes > 0 && nYRes > 0 );
	
	if ( proj == NULL || real == NULL ) {
		return false;
	}

	XnDouble fCoeffX = (double)(nXRes) / fXToZ;
	XnDouble fCoeffY = (double)(nYRes) / fYToZ;

	// We can assume resolution is even ( So Integer div is sufficient )
	int nHalfXRes = nXRes / 2;
	int nHalfYRes = nYRes / 2;

	for ( int i = 0; i < count; ++i )
	{
		proj[i].X = fCoeffX * real[i].X / real[i].Z + nHalfXRes;
		proj[i].Y = nHalfYRes - fCoeffY * real[i].Y / real[i].Z;
		proj[i].Z = real[i].Z;
	}

	return true;
}

}	// End of Namespace (OpenNiUtility)