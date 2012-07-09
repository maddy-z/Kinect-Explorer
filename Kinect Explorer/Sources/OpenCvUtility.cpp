#include	"OpenCvUtility.h"

#include	<opencv2\opencv.hpp>

#define		OPENNI_MAX_DEPTH		10000

namespace	OpenCvUtility				// Start Of Namespace ( OpenCvUtility )

{	

double CalcAverageDepth ( const cv::Mat & srcDepthMat )
{
	if ( srcDepthMat.empty() ) {
		return ( -1 );
	}

	assert ( srcDepthMat.type() == CV_16UC1 );

	double totalDepthValue = 0.0;
	int totalDepthPixelCount = 0;

	const unsigned short * pData = NULL;
	const uchar * pSrcRow = NULL;

	int srcRow = srcDepthMat.rows, srcCol = srcDepthMat.cols, srcRowStep = srcDepthMat.step;

	pSrcRow = srcDepthMat.data;
	for (int i = 0; i < srcRow; ++i, pSrcRow += srcRowStep) 
	{
		pData = (const unsigned short *)(pSrcRow);
		for (int j = 0; j < srcCol; ++j, ++pData) {
			if (*pData) {
				totalDepthValue += *pData;
				++totalDepthPixelCount;
			}
		}
	}

	return (totalDepthValue) / (double)(totalDepthPixelCount);
}
double CalcSmallestDepth(const cv::Mat & srcDepthMat)
{
	if ( srcDepthMat.empty() ) {
		return (-1);
	}

	unsigned short value = 0xffff;
	
	const unsigned short * pData = NULL;
	const uchar * pSrcRow = NULL;

	int srcRow = srcDepthMat.rows, srcCol = srcDepthMat.cols, srcRowStep = srcDepthMat.step;

	pSrcRow = srcDepthMat.data;
	for (int i = 0; i < srcRow; ++i, pSrcRow += srcRowStep) 
	{
		pData = (const unsigned short *)(pSrcRow);		
		
		for ( int j = 0; j < srcCol; ++j, ++pData ) {
			if ( *pData > 0 && value > *pData ) {
				value = *pData;
			}
		}
	}

	return value;
}
double CalcBiggestDepth(const cv::Mat & srcDepthMat)
{
	if ( srcDepthMat.empty() ) {
		return (-1);
	}

	unsigned short value = 0x0;
	
	const unsigned short * pData = NULL;
	const uchar * pSrcRow = NULL;

	int srcRow = srcDepthMat.rows, srcCol = srcDepthMat.cols, srcRowStep = srcDepthMat.step;

	pSrcRow = srcDepthMat.data;
	for ( int i = 0; i < srcRow; ++i, pSrcRow += srcRowStep ) 
	{
		pData = (const unsigned short *)(pSrcRow);

		for ( int j = 0; j < srcCol; ++j, ++pData ) {
			if ( *pData > value ) {
				value = *pData;
			}
		}
	}

	return value;
}

double CalcBiggestDepth8u(const cv::Mat & srcDepthMat)
{
	if (srcDepthMat.empty()) {
		return (-1);
	}

	uchar value = 0x0;
	
	const uchar * pData = NULL;
	const uchar * pSrcRow = NULL;

	int srcRow = srcDepthMat.rows, srcCol = srcDepthMat.cols, srcRowStep = srcDepthMat.step;

	pSrcRow = srcDepthMat.data;
	
	for ( int i = 0; i < srcRow; ++i, pSrcRow += srcRowStep ) 
	{
		pData = pSrcRow;
		
		for ( int j = 0; j < srcCol; ++j, ++pData ) {
			if ( *pData > value ) {
				value = (*pData);
			}
		}
	}

	return value;
}

void CalcMinimumOfDepthHistogram ( const cv::Mat & srcDepthMat, int & minIndex, int & minHistValue, int threshold )
{
	minIndex = -1;
	minHistValue = -1;

	if ( srcDepthMat.empty() ) {
		return;
	}

	assert ( srcDepthMat.type() == CV_16UC1 );

	const int nXRes = srcDepthMat.size().width;
	const int nYRes = srcDepthMat.size().height;
	const int srcRowStep = srcDepthMat.step;

	const uchar * srcRowPtr = NULL;
	const unsigned short * srcDataPtr = NULL;

	float depthHist[OPENNI_MAX_DEPTH];
	memset ( depthHist, 0, OPENNI_MAX_DEPTH * sizeof ( float ) );

	unsigned int pointsNumber = 0;
	srcRowPtr = srcDepthMat.data;

	for ( int y = 0; y < nYRes; ++y, srcRowPtr += srcRowStep ) 
	{
		srcDataPtr = ( const unsigned short * )( srcRowPtr );
	
		for ( int x = 0; x < nXRes; ++x, ++srcDataPtr ) 
		{
			unsigned short tmp = (*srcDataPtr);
	
			if ( tmp && tmp <= threshold ) {
				++depthHist[tmp];
				++pointsNumber;
			}
		}
	}

	float tmpHist[OPENNI_MAX_DEPTH];
	memcpy ( tmpHist, depthHist, OPENNI_MAX_DEPTH * sizeof ( float ) );

	for ( int i = 3; i < threshold - 1; ++i ) 
	// for (int i = 2; i < OPENNI_MAX_DEPTH - 2; ++i) 
	{
		depthHist[i] = tmpHist[i-2] * 0.054 + 
			tmpHist[i-1] * 0.242 + tmpHist[i] * 0.399 + tmpHist[i+1] * 0.242 + tmpHist[i+2] * 0.054;
	}

	// 
	// Search for Minimum Value Of Depth Histogram
	// 

	int maxValue[2], maxIndex[2], tmp;

	maxValue[0] = depthHist[0];
	maxValue[1] = depthHist[1];

	maxIndex[0] = 0;
	maxIndex[1] = 1;

	for ( int i = 0; i <= threshold; ++i ) 
	{
		if ( maxValue[0] > maxValue[1] ) {
			tmp = maxValue[0], maxValue[0] = maxValue[1], maxValue[1] = tmp;
		}

		if (depthHist[i] <= maxValue[0]) {
			continue;
		}
		else if (depthHist[i] > maxValue[0] && depthHist[i] <= maxValue[1]) {
			maxValue[0] = depthHist[i];
			maxIndex[0] = i;
		}
		else if (depthHist[i] > maxValue[1]) {
			maxValue[0] = maxValue[1];
			maxIndex[0] = maxIndex[1];

			maxValue[1] = depthHist[i];
			maxIndex[1] = i;
		}
	}

	if ( maxIndex[0] > maxIndex[1] ) {
		tmp = maxIndex[0], maxIndex[0] = maxIndex[1], maxIndex[1] = tmp;
		tmp = maxValue[0], maxValue[0] = maxValue[1], maxValue[1] = tmp;
	}

	tmp = ( maxValue[0] > maxValue[1] ) ? ( maxValue[0] ) : ( maxValue[1] );
	int tmpIndex;

	for ( int i = maxIndex[0] + 1; i < maxIndex[1]; ++i ) 
	{
		if ( depthHist[i] < tmp ) 
		{
			tmp = depthHist[i];
			tmpIndex = i;
		}
	}

	if ( tmp == ((maxValue[0] > maxValue[1]) ? maxValue[0] : maxValue[1]) ) {
		return;
	}

	minHistValue = tmp;
	minIndex = tmpIndex;

	return;
}

}						

// End of Namespace ( OpenCvUtility )