#include <stdio.h>
#include <stdlib.h>

#include <XnOpenNI.h>
#include <XnCppWrapper.h>

#include <opencv2\opencv.hpp>

#include <qimage.h>
#include <qdebug.h>

#include "GlobalUtility.h"
#include "OpenCvUtility.h"

#define			OPENNI_MAX_DEPTH		10000

namespace GlobalUtility

{

bool CopyCvMat16uToDepthRawBuf(const cv::Mat & srcDepthMat, XnDepthPixel * destDepthData)
{
	if (srcDepthMat.empty() || destDepthData == NULL) {
		return false;
	}

	assert (srcDepthMat.type() == CV_16UC1);

	const int nXRes = srcDepthMat.size().width;
	const int nYRes = srcDepthMat.size().height;
	const int srcRowStep = srcDepthMat.step;

	const uchar * srcRowPtr = NULL;
	const XnDepthPixel * srcDataPtr = NULL;

	srcRowPtr = srcDepthMat.data;

	for (int y = 0; y < nYRes; ++y, srcRowPtr += srcRowStep) 
	{
		srcDataPtr = (const XnDepthPixel *)(srcRowPtr);

		for (int x = 0; x < nXRes; ++x, ++srcDataPtr, ++destDepthData) {
			*destDepthData = *srcDataPtr;
		}
	}

	return true;
}
bool CopyDepthRawBufToCvMat16u(const XnDepthPixel * srcDepthData, cv::Mat & destDepthMat)
{
	if (srcDepthData == NULL || destDepthMat.empty()) {
		return false;
	}

	assert (destDepthMat.type() == CV_16UC1);
	
	const uchar * destRowPtr = NULL;
	XnDepthPixel * destDataPtr = NULL;
	
	const int destHeight = destDepthMat.size().height;
	const int destWidth = destDepthMat.size().width;
	const int destRowStep = destDepthMat.step;

	destRowPtr = destDepthMat.data;
	
	for (int y = 0; y < destHeight; ++y, destRowPtr += destRowStep) 
	{
		destDataPtr = (XnDepthPixel *)(destRowPtr);

		for (int x = 0; x < destWidth; ++x, ++srcDepthData, ++destDataPtr) 
		{
			*destDataPtr = *srcDepthData;
		}
	}

	return true;
}
bool CopyColorRawBufToCvMat8uc3(const XnRGB24Pixel * srcColorData, cv::Mat & destColorMat)
{
	if (srcColorData == NULL || destColorMat.empty()) {
		return false;
	}

	assert (destColorMat.type() == CV_8UC3);

	const uchar * srcDataPtr = NULL;
	const uchar * destRowPtr = NULL;
	uchar * destDataPtr = NULL;
	
	const int destHeight = destColorMat.size().height;
	const int destWidth = destColorMat.size().width;
	const int destRowStep = destColorMat.step;

	srcDataPtr = (const uchar *)(srcColorData);
	destRowPtr = destColorMat.data;
	
	for (int y = 0; y < destHeight; ++y, destRowPtr += destRowStep) 
	{
		destDataPtr = (uchar *)(destRowPtr);

		for (int x = 0; x < destWidth; ++x, srcDataPtr += 3, destDataPtr += 3) 
		{
			destDataPtr[0] = srcDataPtr[0];
			destDataPtr[1] = srcDataPtr[1];
			destDataPtr[2] = srcDataPtr[2];
		}
	}

	return true;
}

bool CopyCvMat16uTo8u(const cv::Mat & srcCvMat, cv::Mat & destCvMat)
{
	assert (srcCvMat.size() == destCvMat.size());
	assert (srcCvMat.type() == CV_16UC1 && destCvMat.type() == CV_8UC1);

	if (srcCvMat.empty() || destCvMat.empty()) {
		return false;
	}

	const int nXRes = srcCvMat.size().width;
	const int nYRes = srcCvMat.size().height;
	const int srcRowStep = srcCvMat.step, destRowStep = destCvMat.step;

	const uchar * srcRowPtr = NULL;
	const unsigned short * srcDataPtr = NULL;

	assert (sizeof(unsigned short) == 2);

	uchar * destRowPtr = NULL;
	uchar * destDataPtr = NULL;

	double maxValue = OpenCvUtility::CalcBiggestDepth(srcCvMat);
	double minValue = OpenCvUtility::CalcSmallestDepth(srcCvMat);

	srcRowPtr = srcCvMat.data;
	destRowPtr = destCvMat.data;

	for (int y = 0; y < nYRes; ++y, srcRowPtr += srcRowStep, destRowPtr += destRowStep) 
	{
		srcDataPtr = (const unsigned short *)(srcRowPtr);
		destDataPtr = destRowPtr;

		for ( int x = 0; x < nXRes; ++x, ++srcDataPtr, ++destDataPtr ) {
			(*destDataPtr) = ((double)(*srcDataPtr) - minValue) / (maxValue - minValue + 1.0f) * 255;
		}
	}

	return true;
}
bool CopyCvMat8uTo16u(const cv::Mat & srcCvMat, cv::Mat & destCvMat)
{
	assert (srcCvMat.size() == destCvMat.size());
	assert (srcCvMat.type() == CV_8UC1 && destCvMat.type() == CV_16UC1);

	if (srcCvMat.empty() || destCvMat.empty()) {
		return false;
	}

	const int nXRes = srcCvMat.size().width;
	const int nYRes = srcCvMat.size().height;
	const int srcRowStep = srcCvMat.step, destRowStep = destCvMat.step;

	const uchar * srcRowPtr = NULL;
	const uchar * srcDataPtr = NULL;

	uchar * destRowPtr = NULL;
	unsigned short * destDataPtr = NULL;

	srcRowPtr = srcCvMat.data;
	destRowPtr = destCvMat.data;

	for (int y = 0; y < nYRes; ++y, srcRowPtr += srcRowStep, destRowPtr += destRowStep) 
	{
		srcDataPtr = srcRowPtr;
		destDataPtr = (unsigned short *)(destRowPtr);

		for (int x = 0; x < nXRes; ++x, ++srcDataPtr, ++destDataPtr) {
			(*destDataPtr) = (*srcDataPtr);
		}
	}

	return true;
}

bool ConvertDepthCvMat16uToYellowQImage(const cv::Mat & srcDepthMat, QImage & destImg)
{
	if (srcDepthMat.empty() || destImg.isNull()) {
		return false;
	}

	assert (srcDepthMat.size().height == destImg.height() && srcDepthMat.size().width == destImg.width());
	assert (srcDepthMat.type() == CV_16UC1);

	const int nXRes = srcDepthMat.size().width;
	const int nYRes = srcDepthMat.size().height;
	const int srcRowStep = srcDepthMat.step;

	const uchar * srcRowPtr = NULL;
	const unsigned short * srcDataPtr = NULL;

	float depthHist[OPENNI_MAX_DEPTH];
	memset(depthHist, 0, OPENNI_MAX_DEPTH * sizeof(float));

	unsigned int pointsNumber = 0;
	srcRowPtr = srcDepthMat.data;

	for (int y = 0; y < nYRes; ++y, srcRowPtr += srcRowStep) 
	{
		srcDataPtr = (const unsigned short *)(srcRowPtr);
	
		for (int x = 0; x < nXRes; ++x, ++srcDataPtr) 
		{
			unsigned short tmp = *srcDataPtr;
			if ( tmp ) {
				++depthHist[tmp];
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
			depthHist[i] = (unsigned int)(256 * (1.0f - depthHist[i] / pointsNumber));
		}
	}
	
	srcRowPtr = srcDepthMat.data;
	
	for (int y = 0; y < nYRes; ++y, srcRowPtr += srcRowStep) 
	{
		srcDataPtr = (const unsigned short *)(srcRowPtr);
		uchar * imageptr = destImg.scanLine(y);
	
		for (int x = 0; x < nXRes; ++x, ++srcDataPtr, imageptr += 4) 
		{
			unsigned short tmp = (*srcDataPtr);
			if ( tmp )
			{
				imageptr[0] = 0;
				imageptr[1] = depthHist[tmp];
				imageptr[2] = depthHist[tmp];
				imageptr[3] = 0xff;
			}
			else 
			{
				imageptr[3] = imageptr[2] = imageptr[1] = imageptr[0] = 0;
			}
		}
	}

	return true;
}
bool ConvertDepthCvMat16uToColorfulQImage(const cv::Mat & srcDepthMat, QImage & destImg, XnDepthPixel maxDepth)
{
	qDebug("Entering:\tbool GlobalUtility::ConvertDepthCvMat16uToColorfulQImage");

	if (srcDepthMat.empty() || destImg.isNull()) 
	{
		qDebug("Leaving:\tbool GlobalUtility::ConvertDepthCvMat16uToColorfulQImage");
		return false;
	}

	assert (srcDepthMat.rows == destImg.height() && srcDepthMat.cols == destImg.width());
	assert (srcDepthMat.type() == CV_16UC1);

	int srcRow = srcDepthMat.rows, srcCol = srcDepthMat.cols;
	int srcRowStep = srcDepthMat.step;

	const uchar * pSrcRow = NULL;
	const unsigned short * pSrcData = NULL;

	// double minDepth = OpenCvUtility::CalcSmallestDepth(srcDepthMat);
	double maxDepthW = OpenCvUtility::CalcBiggestDepth(srcDepthMat);

	// qDebug("Min Depth = %f", minDepth);
	// qDebug("Max Depth = %f", maxDepthW);

	unsigned short h, s = 255, v = 255;
	pSrcRow = srcDepthMat.data;

	for (int i = 0; i < srcRow; ++i, pSrcRow += srcRowStep) 
	{
		pSrcData = (const unsigned short *)(pSrcRow);
		uchar * imagePtr = destImg.scanLine(i);

		for (int j = 0; j < srcCol; ++j, ++pSrcData, imagePtr += 4) 
		{
			if (*pSrcData) 
			{
				h = (float)(*pSrcData) * 360.0f / (maxDepthW + 0.3f);
				OpenCvUtility::HSV2RGB(h, s, v, imagePtr[0], imagePtr[1], imagePtr[2]);
			}
			else 
			{
				imagePtr[2] = imagePtr[1] = imagePtr[0] = 0;
				imagePtr[3] = 0xff;
			}
		}
	}

	qDebug("Leaving:\tbool GlobalUtility::ConvertDepthCvMat16uToColorfulQImage");

	return true;
}
bool ConvertDepthCvMat16uToGrayQImage(const cv::Mat & srcDepthMat, QImage & destImg)
{
	if (srcDepthMat.empty() || destImg.isNull()) 
	{
		return false;
	}

	assert (srcDepthMat.rows == destImg.height() && srcDepthMat.cols == destImg.width());
	assert (srcDepthMat.type() == CV_16UC1);

	double maxDepth = OpenCvUtility::CalcBiggestDepth(srcDepthMat);

	const int nXRes = srcDepthMat.cols;
	const int nYRes = srcDepthMat.rows;
	const int srcRowStep = srcDepthMat.step;

	const uchar * srcRowPtr = NULL;
	const unsigned short * srcDataPtr = NULL;

	srcRowPtr = srcDepthMat.data;

	for (int y = 0; y < nYRes; ++y, srcRowPtr += srcRowStep) 
	{
		srcDataPtr = (const unsigned short *)(srcRowPtr);
		uchar * imagePtr = destImg.scanLine(y);

		for (int x = 0; x < nXRes; ++x, ++srcDataPtr, imagePtr += 4) 
		{
			if (*srcDataPtr) {
				uchar value = 255.f * (1.f - (double)(*srcDataPtr) / maxDepth); 
		
				imagePtr[2] = imagePtr[1] = imagePtr[0] = value;
				imagePtr[3] = 0xff;
			}
			else {
				imagePtr[2] = imagePtr[1] = imagePtr[0] = 0;
				imagePtr[3] = 0xff;
			}
		}
	}
	
	return true;
}

bool ConvertDepthCvMat8uToYellowQImage(const cv::Mat & srcDepthMat, QImage & destImg)
{
	if (srcDepthMat.empty() || destImg.isNull()) {
		return false;
	}

	assert (srcDepthMat.size().height == destImg.height() && srcDepthMat.size().width == destImg.width());
	assert (srcDepthMat.type() == CV_8UC1);

	const int nXRes = srcDepthMat.size().width;
	const int nYRes = srcDepthMat.size().height;
	const int srcRowStep = srcDepthMat.step;

	const uchar * srcRowPtr = NULL;
	const uchar * srcDataPtr = NULL;

	float depthHist[256];
	memset(depthHist, 0, 256 * sizeof(float));

	unsigned int pointsNumber = 0;
	srcRowPtr = srcDepthMat.data;

	for (int y = 0; y < nYRes; ++y, srcRowPtr += srcRowStep) 
	{
		srcDataPtr = srcRowPtr;
	
		for (int x = 0; x < nXRes; ++x, ++srcDataPtr) 
		{
			uchar tmp = *srcDataPtr;
			if ( tmp ) {
				++depthHist[tmp];
				++pointsNumber;
			}
		}
	}
	for (int i = 1; i < 256; ++i) {
		depthHist[i] += depthHist[i-1];
	}
	if (pointsNumber > 0)
	{
		for (int i = 1; i < 256; ++i) {
			depthHist[i] = 256.0f * (1.0f - depthHist[i] / (float)(pointsNumber));
		}
	}
	
	srcRowPtr = srcDepthMat.data;
	
	for (int y = 0; y < nYRes; ++y, srcRowPtr += srcRowStep) 
	{
		srcDataPtr = srcRowPtr;
		uchar * imageptr = destImg.scanLine(y);
	
		for (int x = 0; x < nXRes; ++x, ++srcDataPtr, imageptr += 4) 
		{
			uchar tmp = *srcDataPtr;
			if ( tmp )
			{
				imageptr[0] = 0;
				imageptr[1] = depthHist[tmp];
				imageptr[2] = depthHist[tmp];
				imageptr[3] = 0xff;
			}
			else 
			{
				imageptr[3] = imageptr[2] = imageptr[1] = imageptr[0] = 0;
			}
		}
	}

	return true;
}
bool ConvertDepthCvMat8uToGrayQImage(const cv::Mat & srcDepthMat, QImage & destImg)
{
	if (srcDepthMat.empty() || destImg.isNull()) {
		return false;
	}

	assert (srcDepthMat.size().height == destImg.height() && srcDepthMat.size().width == destImg.width());
	assert (srcDepthMat.type() == CV_8UC1);

	double maxDepth = OpenCvUtility::CalcBiggestDepth8u(srcDepthMat);

	const int nXRes = srcDepthMat.size().width;
	const int nYRes = srcDepthMat.size().height;
	const int srcRowStep = srcDepthMat.step;

	const uchar * srcRowPtr = NULL;
	const uchar * srcDataPtr = NULL;

	srcRowPtr = srcDepthMat.data;

	for (int y = 0; y < nYRes; ++y, srcRowPtr += srcRowStep) 
	{
		srcDataPtr = srcRowPtr;
		uchar * imagePtr = destImg.scanLine(y);

		for (int x = 0; x < nXRes; ++x, ++srcDataPtr, imagePtr += 4) 
		{
			if (*srcDataPtr) {
				uchar value = 255.f * (1.f - (double)(*srcDataPtr) / maxDepth); 
		
				imagePtr[2] = imagePtr[1] = imagePtr[0] = value;
				imagePtr[3] = 0xff;
			}
			else {
				imagePtr[2] = imagePtr[1] = imagePtr[0] = 0;
				imagePtr[3] = 0xff;
			}
		}
	}
	
	return true;
}

bool ConvertDepthCvMat16uToYellowCvMat(const cv::Mat & srcDepthMat, cv::Mat & destMat)
{
	if ( srcDepthMat.empty() || destMat.empty() ) {
		return false;
	}

	assert ( srcDepthMat.size() == destMat.size() );
	assert ( srcDepthMat.type() == CV_16UC1 );

	const int nXRes = srcDepthMat.size().width;
	const int nYRes = srcDepthMat.size().height;
	const int srcRowStep = srcDepthMat.step;

	const uchar * srcRowPtr = NULL;
	const unsigned short * srcDataPtr = NULL;

	float depthHist[OPENNI_MAX_DEPTH];
	memset(depthHist, 0, OPENNI_MAX_DEPTH * sizeof(float));

	unsigned int pointsNumber = 0;
	srcRowPtr = srcDepthMat.data;

	for (int y = 0; y < nYRes; ++y, srcRowPtr += srcRowStep) 
	{
		srcDataPtr = (const unsigned short *)(srcRowPtr);
	
		for (int x = 0; x < nXRes; ++x, ++srcDataPtr) 
		{
			unsigned short tmp = *srcDataPtr;
			if ( tmp ) {
				++depthHist[tmp];
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
			depthHist[i] = (unsigned int)(256 * (1.0f - depthHist[i] / pointsNumber));
		}
	}
	
	srcRowPtr = srcDepthMat.data;
	
	for (int y = 0; y < nYRes; ++y, srcRowPtr += srcRowStep) 
	{
		srcDataPtr = (const unsigned short *)(srcRowPtr);
		uchar * imageptr = destMat.ptr(y);
	
		for (int x = 0; x < nXRes; ++x, ++srcDataPtr, imageptr += destMat.channels()) 
		{
			unsigned short tmp = (*srcDataPtr);
			if ( tmp )
			{
				imageptr[0] = 0;
				imageptr[1] = depthHist[tmp];
				imageptr[2] = depthHist[tmp];
				// imageptr[3] = 0xff;
			}
			else 
			{
				// imageptr[3] = 0;
				imageptr[2] = imageptr[1] = imageptr[0] = 0;
			}
		}
	}

	return true;
}

bool ConvertDepthCvMat16uToGrayCvMat(const cv::Mat & srcDepthMat, cv::Mat & destMat)
{
	if ( srcDepthMat.empty() || destMat.empty() ) 
	{
		return false;
	}

	assert ( srcDepthMat.size() == destMat.size() );
	assert ( srcDepthMat.type() == CV_16UC1 );

	double maxDepth = OpenCvUtility::CalcBiggestDepth ( srcDepthMat );

	const int nXRes = srcDepthMat.cols;
	const int nYRes = srcDepthMat.rows;
	const int srcRowStep = srcDepthMat.step;

	const uchar * srcRowPtr = NULL;
	const unsigned short * srcDataPtr = NULL;

	srcRowPtr = srcDepthMat.data;

	for (int y = 0; y < nYRes; ++y, srcRowPtr += srcRowStep) 
	{
		srcDataPtr = (const unsigned short *)(srcRowPtr);
		uchar * imagePtr = destMat.ptr(y);

		for ( int x = 0; x < nXRes; ++x, ++srcDataPtr, imagePtr += destMat.channels() ) 
		{
			if (*srcDataPtr) {
				uchar value = 255.f * (1.f - (double)(*srcDataPtr) / maxDepth); 
		
				imagePtr[2] = imagePtr[1] = imagePtr[0] = value;
				// imagePtr[3] = 0xff;
			}
			else {
				imagePtr[2] = imagePtr[1] = imagePtr[0] = 0;
				// imagePtr[3] = 0xff;
			}
		}
	}
	
	return true;
}

bool ConvertDepthCvMat16uToColorfulCvMat(const cv::Mat & srcDepthMat, cv::Mat & destMat, XnDepthPixel maxDeviceDepth)
{
	if ( srcDepthMat.empty() || destMat.empty() ) 
	{
		return false;
	}

	assert ( srcDepthMat.size() == destMat.size() );
	assert ( srcDepthMat.type() == CV_16UC1 );

	int srcRow = srcDepthMat.rows;
	int srcCol = srcDepthMat.cols;
	int srcRowStep = srcDepthMat.step;

	const uchar * pSrcRow = NULL;
	const unsigned short * pSrcData = NULL;

	// double minDepth = OpenCvUtility::CalcSmallestDepth(srcDepthMat);
	double maxDepthW = OpenCvUtility::CalcBiggestDepth(srcDepthMat);

	// qDebug("Min Depth = %f", minDepth);
	// qDebug("Max Depth = %f", maxDepthW);

	unsigned short h, s = 255, v = 255;
	pSrcRow = srcDepthMat.data;

	for (int i = 0; i < srcRow; ++i, pSrcRow += srcRowStep) 
	{
		pSrcData = (const unsigned short *)(pSrcRow);
		uchar * imagePtr = destMat.ptr(i);

		for ( int j = 0; j < srcCol; ++j, ++pSrcData, imagePtr += destMat.channels() ) 
		{
			if (*pSrcData) 
			{
				h = (float)(*pSrcData) * 360.0f / (maxDepthW + 0.3f);
				OpenCvUtility::HSV2RGB(h, s, v, imagePtr[0], imagePtr[1], imagePtr[2]);
			}
			else 
			{
				imagePtr[2] = imagePtr[1] = imagePtr[0] = 0;
				// imagePtr[3] = 0xff;
			}
		}
	}

	return true;
}

bool CopyCvMat8uToQImage(const cv::Mat & srcMat, QImage & destImg)
{
	if (srcMat.empty() || destImg.isNull()) {
		return false;
	}

	assert (srcMat.size().height == destImg.height() && srcMat.size().width == destImg.width());
	assert (srcMat.type() == CV_8UC1);

	// double maxDepth = OpenCvUtility::CalcBiggestDepth8u(srcDepthMat);

	const int nXRes = srcMat.size().width;
	const int nYRes = srcMat.size().height;
	const int srcRowStep = srcMat.step;

	const uchar * srcRowPtr = NULL;
	const uchar * srcDataPtr = NULL;

	srcRowPtr = srcMat.data;

	for ( int y = 0; y < nYRes; ++y, srcRowPtr += srcRowStep ) 
	{
		srcDataPtr = srcRowPtr;
		uchar * imagePtr = destImg.scanLine(y);

		for (int x = 0; x < nXRes; ++x, ++srcDataPtr, imagePtr += 4) 
		{
			uchar value = *srcDataPtr;

			imagePtr[2] = imagePtr[1] = imagePtr[0] = value;
			imagePtr[3] = 0xff;
		}
	}
	
	return true;
}

bool ConvertCvMat16uByThresholdValue ( const cv::Mat & srcMat, cv::Mat & destMat, double thresholdValue, int newValue )
{
	if ( srcMat.empty() || destMat.empty() ) {
		return false;
	}

	assert ( srcMat.size() == destMat.size() );
	assert ( srcMat.type() == destMat.type() );
	assert ( srcMat.type() == CV_16UC1 );

	memcpy ( destMat.data, srcMat.data, srcMat.step * srcMat.size().height );

	const int nXRes = srcMat.size().width;
	const int nYRes = srcMat.size().height;
	const int srcRowStep = srcMat.step, destRowStep = destMat.step;

	const uchar * srcRowPtr = NULL;
	const unsigned short * srcDataPtr = NULL;
	uchar * destRowPtr = NULL;
	unsigned short * destDataPtr = NULL;

	srcRowPtr = srcMat.data;
	destRowPtr = destMat.data;

	for ( int y = 0; y < nYRes; ++y, srcRowPtr += srcRowStep, destRowPtr += destRowStep ) {
		srcDataPtr = (const unsigned short *)(srcRowPtr);
		destDataPtr = (unsigned short *)(destRowPtr);

		for (int x = 0; x < nXRes; ++x, ++srcDataPtr, ++destDataPtr) {
			if (*srcDataPtr > thresholdValue) {
				*destDataPtr = newValue;
			}
		}
	}

	return true;
}

}