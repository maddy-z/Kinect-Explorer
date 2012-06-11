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

//bool HumanDetection::NearestNeighborFiltering(const cv::Mat & srcDepthMat, cv::Mat & destDepthMat)
//{
//	qDebug() << "Entering:\tbool HumanDetection::NearestNeighborFiltering()";
//
//	assert (srcDepthMat.size() == destDepthMat.size());
//	assert (srcDepthMat.type() == destDepthMat.type());
//	assert (srcDepthMat.type() == CV_16UC1);
//
//	if (srcDepthMat.empty() || destDepthMat.empty()) {
//		qDebug() << "Leaving:\tbool HumanDetection::NearestNeighborFiltering()";
//		return false;
//	}
//
//	int depthHeight = srcDepthMat.size().height, depthWidth = srcDepthMat.size().width;
//	int srcRowStep = srcDepthMat.step, destRowStep = destDepthMat.step;
//
//	const uchar * srcRowPtr = NULL;
//	const XnDepthPixel * srcDataPtr = NULL;
//	uchar * destRowPtr = NULL;
//	XnDepthPixel * destDataPtr = NULL;
//
//	// 
//	// Create Distance Map
//	// 
//
//	int * const distMap = new int[depthHeight * depthWidth];
//	int * distRowPtr = NULL, * distDataPtr = NULL;
//	int adj[4];
//
//	// First Scan
//	srcRowPtr = srcDepthMat.data;
//	destRowPtr = destDepthMat.data;
//	distRowPtr = distMap;
//
//	srcDataPtr = (const XnDepthPixel *)(srcRowPtr);
//	destDataPtr = (XnDepthPixel *)(destRowPtr);
//	distDataPtr = distRowPtr;
//
//	if (*srcDataPtr) {
//		*distDataPtr = 0;
//		*destDataPtr = *srcDataPtr;
//	}
//	else {
//		*distDataPtr = depthWidth + 1;
//		*destDataPtr = 0;
//	}
//
//	for (int x = 1; x < depthWidth; ++x) 
//	{
//		XnDepthPixel tmp = srcDataPtr[x];
//		
//		if (tmp) {
//			distDataPtr[x] = 0;
//			destDataPtr[x] = tmp;
//		}
//		else 
//		{
//			if (distDataPtr[x-1] + 1 < depthWidth + 1) {
//				destDataPtr[x] = destDataPtr[x-1];
//				distDataPtr[x] = distDataPtr[x-1] + 1;
//			}
//			else {
//				distDataPtr[x] = depthWidth + 1;
//				destDataPtr[x] = 0;
//			}
//		}
//	}
//	srcRowPtr += srcRowStep;
//	destRowPtr += destRowStep;
//	distRowPtr += depthWidth;
//
//	for (int y = 1; y < depthHeight; ++y, srcRowPtr += srcRowStep, destRowPtr += destRowStep, distRowPtr += depthWidth) 
//	{
//		srcDataPtr = (const XnDepthPixel *)(srcRowPtr);
//		destDataPtr = (XnDepthPixel *)(destRowPtr);
//		distDataPtr = distRowPtr;
//
//		// 
//		// x = 0
//		// 
//		if (*srcDataPtr) {
//			*distDataPtr = 0;
//			*destDataPtr = *srcDataPtr;
//		}
//		else {
//			adj[0] = *(distDataPtr-depthWidth);
//			adj[1] = *(distDataPtr-depthWidth+1);
//
//			if (adj[0] < adj[1]) { 
//				*distDataPtr = adj[0] + 1;
//				*destDataPtr = *(XnDepthPixel *)((uchar *)destDataPtr - destRowStep);
//			}
//			else {
//				*distDataPtr = adj[1] + 1;
//				*destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr+1) - destRowStep);
//			}
//		}
//		++srcDataPtr;
//		++destDataPtr;
//		++distDataPtr;
//
//		for (int x = 1; x < depthWidth - 1; ++x, ++srcDataPtr, ++distDataPtr, ++destDataPtr) 
//		{
//			if (*srcDataPtr) 
//			{
//				*distDataPtr = 0;
//				*destDataPtr = *srcDataPtr;
//			}
//			else 
//			{
//				// 8-adjant Points
//				adj[0] = *(distDataPtr-1);
//				adj[1] = *(distDataPtr-depthWidth);
//				adj[2] = *(distDataPtr-depthWidth-1);
//				adj[3] = *(distDataPtr-depthWidth+1);
//
//				int min = adj[0];
//				int min_i = 0;
//				for (int k = 1; k < 4; ++k) 
//				{
//					if (adj[k] < min) {
//						min = adj[k];
//						min_i = k;
//					}
//				}
//
//				*distDataPtr = min + 1;
//				
//				switch (min_i) 
//				{
//				case 0: *destDataPtr = *(destDataPtr-1); break;
//				case 1: *destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr)-destRowStep); break;
//				case 2: *destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr-1)-destRowStep); break;
//				case 3: *destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr+1)-destRowStep); break;
//				}
//			}
//		}
//
//		// 
//		// x = depthWidth - 1
//		// 
//		if (*srcDataPtr) {
//			*distDataPtr = 0;
//			*destDataPtr = *srcDataPtr;
//		}
//		else {
//			adj[0] = *(distDataPtr-1);
//			adj[1] = *(distDataPtr-depthWidth);
//			adj[2] = *(distDataPtr-depthWidth-1);
//
//			int min = adj[0];
//			int min_i = 0;
//			for (int k = 1; k < 3; ++k) 
//			{
//				if (adj[k] < min) {
//					min = adj[k];
//					min_i = k;
//				}
//			}
//
//			*distDataPtr = min + 1;
//			
//			switch (min_i) 
//			{
//			case 0: *destDataPtr = *(destDataPtr-1); break;
//			case 1: *destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr)-destRowStep); break;
//			case 2: *destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr-1)-destRowStep); break;
//			}
//		}
//	}
//
//	// 
//	// Second Scan
//	// 
//
//	// srcRowPtr = srcDepthMat.data + (depthHeight-1) * srcRowStep + (depthWidth - 1) * 2;
//	destRowPtr = destDepthMat.data + (depthHeight-1) * destRowStep + (depthWidth - 1) * 2;
//	distRowPtr = distMap + depthHeight * depthWidth - 1;
//
//	// srcDataPtr = (const XnDepthPixel *)(srcRowPtr);
//	destDataPtr = (XnDepthPixel *)(destRowPtr);
//	distDataPtr = distRowPtr;
//
//	if (*distDataPtr > depthWidth + 1) 
//	{
//		*distDataPtr = depthWidth + 1;
//		*destDataPtr = 0;
//	}
//
//	// --srcDataPtr;
//	--destDataPtr;
//	--distDataPtr;
//
//	for (int x = depthWidth - 2; x >= 0; --x, /*--srcDataPtr,*/ --destDataPtr, --distDataPtr) 
//	{
//		// XnDepthPixel tmp = srcDataPtr[x];
//		
//		// if (distDataPtr[x] == 0) { continue; }
//
//		if (*distDataPtr > *(distDataPtr+1) + 1) 
//		{
//			*distDataPtr = *(distDataPtr+1) + 1;
//			*destDataPtr = *(destDataPtr+1);
//		}
//
//		if (*distDataPtr > depthWidth + 1) {
//			*distDataPtr = depthWidth + 1;
//			*destDataPtr = 0;
//		}
//		//if (distDataPtr[x] == 0) {
//		//	/*distDataPtr[x] = 0;
//		//	destDataPtr[x] = tmp;*/
//		//}
//		//else 
//		//{
//		//	if (distDataPtr[x] > distDataPtr[x+1]) {
//		//		distDataPtr[x] = distDataPtr[x+1] + 1;
//		//		destDataPtr[x] = destDataPtr[x+1];
//		//	// if (distDataPtr[x] < depthWidth + 1) {
//		//		// destDataPtr[x] = destDataPtr[x-1];
//		//		// distDataPtr[x] = distDataPtr[x-1] + 1;
//		//	}
//		//	/*else {
//		//		distDataPtr[x] = depthWidth + 1;
//		//		destDataPtr[x] = 0;
//		//	}*/
//		//}
//	}
//	
//	// srcRowPtr -= srcRowStep;
//	destRowPtr -= destRowStep;
//	distRowPtr -= depthWidth;
//
//	for (int y = depthHeight - 2; y >= 0; --y, /*srcRowPtr -= srcRowStep,*/ destRowPtr -= destRowStep, distRowPtr -= depthWidth) 
//	{
//		// srcDataPtr = (const XnDepthPixel *)(srcRowPtr);
//		destDataPtr = (XnDepthPixel *)(destRowPtr);
//		distDataPtr = distRowPtr;
//
//		// 
//		// x = depthWidth - 1
//		// 
//		
//		if (*distDataPtr == 0) {
//			// continue;
//			// *distDataPtr = 0;
//			// *destDataPtr = *srcDataPtr;
//		}
//		else 
//		{
//			adj[0] = *(distDataPtr+depthWidth);
//			adj[1] = *(distDataPtr+depthWidth-1);
//
//			int min, min_i;
//
//			if (adj[0] < adj[1]) {
//				min = adj[0];
//				min_i = 0;
//			}
//			else {
//				min = adj[1];
//				min_i = 1;
//			}
//
//			if (*distDataPtr <= min + 1) { continue; }
//			
//			*distDataPtr = min + 1;
//
//			switch (min_i) 
//			{
//			case 0: *destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr)+destRowStep); break;
//			case 1: *destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr-1)+destRowStep); break;
//			}
//		}
//
//		// --srcDataPtr;
//		--destDataPtr;
//		--distDataPtr;
//
//		for (int x = depthWidth - 2; x >= 1; --x, /*--srcDataPtr,*/ --distDataPtr, --destDataPtr) 
//		{
//			if (*distDataPtr == 0) 
//			{
//				continue;
//				// *distDataPtr = 0;
//				// *destDataPtr = *srcDataPtr;
//			}
//			else 
//			{
//				// 8-adjant Points
//				adj[0] = *(distDataPtr+1);
//				adj[1] = *(distDataPtr+depthWidth);
//				adj[2] = *(distDataPtr+depthWidth-1);
//				adj[3] = *(distDataPtr+depthWidth+1);
//
//				int min = adj[0];
//				int min_i = 0;
//				for (int k = 1; k < 4; ++k) 
//				{
//					if (adj[k] < min) {
//						min = adj[k];
//						min_i = k;
//					}
//				}
//
//				if ( *distDataPtr <= min + 1 ) { continue; }
//	
//				*distDataPtr = min + 1;
//				
//				switch (min_i) 
//				{
//				case 0: *destDataPtr = *(destDataPtr+1); break;
//				case 1: *destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr)+destRowStep); break;
//				case 2: *destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr-1)+destRowStep); break;
//				case 3: *destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr+1)+destRowStep); break;
//				}
//			}
//		}
//
//		// 
//		// x = 0
//		// 
//		if (*distDataPtr == 0) {
//			/**distDataPtr = 0;
//			*destDataPtr = *srcDataPtr;*/
//		}
//		else {
//			adj[0] = *(distDataPtr+1);
//			adj[1] = *(distDataPtr+depthWidth);
//			adj[2] = *(distDataPtr+depthWidth+1);
//
//			int min = adj[0];
//			int min_i = 0;
//			for (int k = 1; k < 3; ++k) 
//			{
//				if (adj[k] < min) {
//					min = adj[k];
//					min_i = k;
//				}
//			}
//
//			if ( *distDataPtr <= min + 1 ) { continue; }
//
//			*distDataPtr = min + 1;
//			
//			switch (min_i) 
//			{
//			case 0: *destDataPtr = *(destDataPtr+1); break;
//			case 1: *destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr)+destRowStep); break;
//			case 2: *destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr+1)+destRowStep); break;
//			}
//		}
//	}
//
//	delete [] distMap;
//
//	//// Second Scan
//	//srcRowPtr = srcDepthMat.dataend - srcDepthMat.step + depthWidth * srcDepthMat.channels();
//	//distRowPtr = distMap;
//
//	//for (int y = 0; y < depthHeight; ++y, srcRowPtr -= srcRowStep, distRowPtr += depthWidth) 
//	//{
//	//	srcDataPtr = (const XnDepthPixel *)(srcRowPtr);
//	//	distDataPtr = distRowPtr;
//	//	// destDataPtr = (XnDepthPixel *)(destRowPtr);
//
//	//	for (int x = 0; x < depthWidth; ++x, srcDataPtr[-x], ++distDataPtr) 
//	//	{
//	//		if (srcDataPtr[-x]) 
//	//		{
//	//			*distDataPtr = 0;
//	//		}
//	//		else 
//	//		{
//	//			adj1 = distMap[(y)*depthWidth + (x+1)];
//	//			adj2 = distMap[(y+1)*depthWidth + (x)];
//	//			adj3 = distMap[(y+1)*depthWidth + (x-1)];
//	//			adj4 = distMap[(y+1)*depthWidth + (x+1)];
//
//	//			int min1 = (adj1 < adj2) ? adj1 : adj2;
//	//			int min2 = (adj3 < adj4) ? adj3 : adj4;
//	//			int min = (min1 < min2) ? (min1 + 1) : (min2 + 1);
//
//	//			*distDataPtr = ;
//	//		}
//	//	}
//	//}
//
//	//srcRowPtr = srcDepthMat.data;
//	//destRowPtr = destDepthMat.data;
//
//	//XnDepthPixel tmp;
//	//cv::Size TopLeft, DownRight;
//
//	//for (int y = 0; y < depthHeight; ++y, srcRowPtr += srcRowStep, destRowPtr += destRowStep) 
//	//{
//	//	srcDataPtr = (const XnDepthPixel *)(srcRowPtr);
//	//	destDataPtr = (XnDepthPixel *)(destRowPtr);
//
//	//	for (int x = 0; x < depthWidth; ++x) 
//	//	{
//	//		destDataPtr[x] = srcDataPtr[x];
//
//	//		for (int k = 1; destDataPtr[x] == 0; ++k) 
//	//		{
//	//			TopLeft.width = x - k;
//	//			TopLeft.height = y - k;
//
//	//			DownRight.width = x + k;
//	//			DownRight.height = y + k;
//
//	//			for (int m = TopLeft.width; m <= DownRight.width /*&& destDataPtr[x] == 0*/; ++m) 
//	//			{
//	//				if (m < 0) { continue; }
//	//				if (m >= depthWidth) { break; }
//	//				
//	//				if (TopLeft.height >= 0) 
//	//				{
//	//					tmp = CV_IMAGE_ELEM(&IplImage(srcDepthMat), XnDepthPixel, TopLeft.height, m);
//	//
//	//					if (tmp > 0) {
//	//						destDataPtr[x] = tmp;
//	//						break;
//	//					}
//	//				}
//	//				
//	//				if (DownRight.height < depthHeight) 
//	//				{
//	//					tmp = CV_IMAGE_ELEM(&IplImage(srcDepthMat), XnDepthPixel, DownRight.height, m);
//	//					
//	//					if (tmp > 0) {
//	//						destDataPtr[x] = tmp;
//	//						break;
//	//					}
//	//				}
//	//			}
//
//	//			for (int m = TopLeft.height; m <= DownRight.height/* && destDataPtr[x] == 0*/; ++m) 
//	//			{
//	//				if (m < 0) { continue; }
//	//				if (m >= depthHeight) { break; }
//
//	//				if (TopLeft.width >= 0) 
//	//				{
//	//					tmp = CV_IMAGE_ELEM(&IplImage(srcDepthMat), XnDepthPixel, m, TopLeft.width);
//
//	//					if (tmp > 0) {
//	//						destDataPtr[x] = tmp;
//	//						break;
//	//					}
//	//				}
//
//	//				if (DownRight.width < depthWidth) 
//	//				{
//	//					tmp = CV_IMAGE_ELEM(&IplImage(srcDepthMat), XnDepthPixel, m, DownRight.width);
//
//	//					if (tmp > 0) {
//	//						destDataPtr[x] = tmp;
//	//						break;
//	//					}
//	//				}
//	//			}
//	//		}
//	//	}
//	//}
//
//	qDebug() << "Leaving:\tbool HumanDetection::NearestNeighborFiltering()";
//
//	return true;
//}

//bool HumanDetection::ConvertDepth2ColorfulQImage(const cv::Mat & srcDepthMat, QImage & destImg, /*unsigned int imgWidth, unsigned int imgHeight, */XnDepthPixel maxDepth)
//{
//	// qDebug("Entering:\tbool HumanDetection::ConvertDepthToColorfulQImage");
//
//	assert (srcDepthMat.type() == CV_16UC1);
//
//	cv::Mat hsv24Mat(srcDepthMat.size(), CV_8UC3);
//
//	// qDebug("After Entering, hsv24Mat Ref Count = %d", *(hsv24Mat.refcount));
//
//	int srcRow = srcDepthMat.rows;
//	int srcCol = srcDepthMat.cols;
//	int srcRowStep = srcDepthMat.step;
//	int destRowStep = hsv24Mat.step;
//
//	const uchar * pSrcRow = srcDepthMat.data;
//	uchar * pHsv24Row = hsv24Mat.data;
//	
//	const XnDepthPixel * pSrcData = NULL;
//	uchar * pHsv24Data = NULL;
//
//	assert (srcRow == destImg.height() && srcCol == destImg.width());
//
//	double minDepth = OpenCvUtility::CalcSmallestDepth(srcDepthMat);
//	double maxDepthW = OpenCvUtility::CalcBiggestDepth(srcDepthMat);
//
//	// qDebug("Min Depth = %f", minDepth);
//	// qDebug("Max Depth = %f", maxDepthW);
//
//	for (int i = 0; i < srcRow; ++i, pSrcRow += srcRowStep, pHsv24Row += destRowStep) 
//	{
//		pSrcData = (const XnDepthPixel *)(pSrcRow);
//		pHsv24Data = pHsv24Row;
//		
//		for (int j = 0; j < srcCol; ++j, pHsv24Data += 3, ++pSrcData) 
//		{
//			if (*pSrcData) 
//			{
//				uchar h = ((*pSrcData) - 1) * 180.0f / (maxDepth);
//				OpenCvUtility::HSV2RGB(h, 255, 255, pHsv24Data[0], pHsv24Data[1], pHsv24Data[2]);
//			
//				/*	
//				pHsv24Data[0] = ((*pSrcData) - minDepth - 1) * 180.0f / (maxDepthW);
//				pHsv24Data[1] = 255;
//				pHsv24Data[2] = 255;
//				*/
//			}
//			else 
//			{
//				pHsv24Data[0] = 0;
//				pHsv24Data[1] = 0;
//				pHsv24Data[2] = 0;
//			}
//		}
//	}
//
//	// cv::cvtColor(hsv24Mat, hsv24Mat, CV_HSV2RGB);
//
//	// 
//	// Convert cv::Mat to QImage
//	// 
//	
//	pHsv24Row = hsv24Mat.data;
//
//	for (int y = 0; y < srcRow; ++y, pHsv24Row += destRowStep) 
//	{
//		uchar * imagePtr = destImg.scanLine(y);
//		
//		pHsv24Data = pHsv24Row;
//		for (int x = 0; x < srcCol; ++x, pHsv24Data += 3, imagePtr += 4) 
//		{
//			imagePtr[0] = pHsv24Data[0];
//			imagePtr[1] = pHsv24Data[1];
//			imagePtr[2] = pHsv24Data[2];
//			imagePtr[3] = 0xff;
//		}
//	}
//	
//	// qDebug("Before Leaving, hsv24Mat Ref Count = %d", *(hsv24Mat.refcount));
//	// qDebug("Leaving:\tbool HumanDetection::ConvertDepthToColorfulQImage");
//
//	return true;
//}
//
//bool HumanDetection::CopyDepthRawData2CvMat(const XnDepthPixel * srcDepthData, cv::Mat & destDepthMat)
//{
//	if (srcDepthData == NULL || destDepthMat.empty()) {
//		return false;
//	}
//
//	assert (destDepthMat.type() == CV_16UC1);
//	
//	const uchar * destRowPtr = NULL;
//	XnDepthPixel * destDataPtr = NULL;
//	
//	const int destHeight = destDepthMat.size().height;
//	const int destWidth = destDepthMat.size().width;
//	const int destRowStep = destDepthMat.step;
//
//	destRowPtr = destDepthMat.data;
//	
//	for (int y = 0; y < destHeight; ++y, destRowPtr += destRowStep) 
//	{
//		destDataPtr = (XnDepthPixel *)(destRowPtr);
//
//		for (int x = 0; x < destWidth; ++x, ++srcDepthData, ++destDataPtr) 
//		{
//			*destDataPtr = *srcDepthData;
//		}
//	}
//
//	return true;
//}
//
//bool HumanDetection::CopyColorRawData2CvMat(const XnRGB24Pixel * srcColorData, cv::Mat & destColorMat)
//{
//	if (srcColorData == NULL || destColorMat.empty()) {
//		return false;
//	}
//
//	assert (destColorMat.type() == CV_8UC3);
//
//	uchar * srcDataPtr = NULL;
//	uchar * destDataPtr = NULL;
//	
//	const uchar * destRowPtr = NULL;
//	
//	const int destHeight = destColorMat.size().height;
//	const int destWidth = destColorMat.size().width;
//	const int destRowStep = destColorMat.step;
//
//	srcDataPtr = (uchar *)(srcColorData);
//	destRowPtr = destColorMat.data;
//	
//	for (int y = 0; y < destHeight; ++y, destRowPtr += destRowStep) 
//	{
//		destDataPtr = (uchar *)(destRowPtr);
//
//		for (int x = 0; x < destWidth; ++x, srcDataPtr += 3, destDataPtr += 3) 
//		{
//			destDataPtr[0] = srcDataPtr[0];
//			destDataPtr[1] = srcDataPtr[1];
//			destDataPtr[2] = srcDataPtr[2];
//		}
//	}
//
//	return true;
//}