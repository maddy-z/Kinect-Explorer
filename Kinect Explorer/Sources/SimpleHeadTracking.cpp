#include <XnOpenNI.h>
#include <XnCppWrapper.h>

#include <qimage.h>
#include <qdebug.h>

#include "GlobalUtility.h"
#include "OpenCvUtility.h"
#include "OpenNiUtility.h"

#include "SimpleHeadTracking.h"
#include "HumanDetection.h"

#include "KinectHoleFiller.h"

// 
// MACRO DEFINITIONS
// 

#define			OPENNI_MAX_DEPTH			10000

// -------------------------------
//  Public Static Member Function
// -------------------------------

bool SimpleHeadTracking::
	 SimpleDepthHandlingFunc ( QImage & destImgBuff, const xn::DepthGenerator & depthGen )
{
	// 
	// Invalid Depth Stream Input, Then Return an Empty QImage
	// 

	xn::DepthMetaData depthMD;
	depthGen.GetMetaData ( depthMD );

	const XnDepthPixel * srcDepthData = depthMD.Data();
	if ( srcDepthData == NULL ) {
		return false;
	}

	bool result = false;
	int nXRes = depthMD.XRes(), nYRes = depthMD.YRes();

	// Step 0:	Preparation Work -- Copy Depth Raw Data to cv::Mat
	cv::Mat depthMat ( nYRes, nXRes, CV_16UC1 );
	result = GlobalUtility::CopyDepthRawBufToCvMat16u ( srcDepthData, depthMat );

	// cv::Mat depthDTMat ( nYRes, nXRes, CV_8UC1 );
	// result = KinectHoleFiller::DistanceTransform ( depthMat, depthMat, 10001 );
	// result = KinectHoleFiller::DistanceTransform2 ( depthMat, depthMat, 10001 );
	// GlobalUtility::CopyCvMat16uTo8u ( depthMat, depthDTMat );
	
	// cv::namedWindow ( "DT Image", 1 );
	// cv::imshow ( "DT Image", depthDTMat );
	// cv::waitKey(15);

	// Step 1:	Filter Depth Data By Threshold Value
	const int thresholdDepthValue = 1500.0f, newDepthValue = 2000;
	result = GlobalUtility::ConvertCvMat16uByThresholdValue ( depthMat, depthMat, thresholdDepthValue, newDepthValue );

	// Step 2:	Fill Depth Shadow
	result = KinectHoleFiller::NearestNeighborhoodHoleFilling2 ( depthMat, depthMat, OPENNI_MAX_DEPTH );

	// Step 3:	Filter Depth Data
	cv::medianBlur ( depthMat, depthMat, 3 );

	// Step 3:	Calculate Local Minimum Depth Histogram Value
	// int minDepth, minHistValue;
	// OpenCvUtility::CalcMinimumOfDepthHistogram ( depthMat, minDepth, minHistValue, 5000 );

	// Step 4:	Threshold cv::Mat By Threshold Value
	// minDepth = 5000;
	// if ( minDepth >= 0 ) {
	// 	GlobalUtility::ConvertCvMat16uByThresholdValue ( depthMat, depthMat, minDepth );
	// }

	// Step 4:	Convert Depth cv::Mat to QImage 
	result = GlobalUtility::ConvertDepthCvMat16uToGrayQImage ( depthMat, destImgBuff );
	
	static QRgb whiteColor = qRgb ( 255, 255, 255 );
	static QRgb blackColor = qRgb ( 0, 0, 0 );

	// Step 5:	Find and Draw Rough Head Coordinates in Preprocessed Range Data
	XnDepthPixel * destDepthData = new XnDepthPixel[nXRes * nYRes];
	result = GlobalUtility::CopyCvMat16uToDepthRawBuf ( depthMat, destDepthData );

	int rIndex, cIndex;
	if ( SimpleHeadTracking::FindHeadCoordinate ( destDepthData, nXRes, nYRes, rIndex, cIndex, depthGen ) ) 
	{	
		for (int i = 0; i < 25; ++i) 
		{
			if ( rIndex + i < nYRes )	{ destImgBuff.setPixel ( cIndex, rIndex + i, blackColor ); }
			if ( rIndex - i >= 0 )		{ destImgBuff.setPixel ( cIndex, rIndex - i, blackColor ); }
			if ( cIndex + i < nXRes )	{ destImgBuff.setPixel ( cIndex + i, rIndex, blackColor ); }
			if ( cIndex - i >= 0 )		{ destImgBuff.setPixel ( cIndex - i, rIndex, blackColor ); }
		}
	}

	// Optional Step6: Draw Local Minimum Point Per Line
	std::vector<int> minArray;
	SimpleHeadTracking::GetLocalMinimumOfDepthMap(destDepthData, minArray, depthGen);

	for (int i = 0; i < minArray.size(); ++i) {
		if (minArray[i] == -1) { continue; }
		destImgBuff.setPixel(minArray[i], i, whiteColor);
	}

	delete [] destDepthData;

	return true;
}

// --------------------------------
//  Private Static Member Function
// --------------------------------

bool SimpleHeadTracking::FillShadow (	const XnDepthPixel * srcDepthData, 
										XnDepthPixel * destDepthData, 
										int nXRes, 
										int nYRes, 
										XnDepthPixel maxDeviceDepth )
{
	// qDebug() << "Entering:\tbool SimpleHeadTracking::FillShadow()";

	if (srcDepthData == NULL || destDepthData == NULL) {
		return false;
	}

	XnDepthPixel lastKnownPixel, tmpDepth;

	const XnDepthPixel * srcDataPtr = srcDepthData;
	XnDepthPixel * destDataPtr = destDepthData;

	for (int y = 0; y < nYRes; ++y) 
	{
		lastKnownPixel = maxDeviceDepth;
	
		for (int x = 0; x < nXRes; ++x,  ++srcDataPtr, ++destDataPtr) 
		{
			tmpDepth = (*srcDataPtr);
			// tmpDepth = srcDepthData[y * nXRes + x];

			if (tmpDepth) {
				lastKnownPixel = tmpDepth;
				*destDataPtr = tmpDepth;
				// destDepthData[y * nXRes + x] = tmpDepth;
			}
			else {
				*destDataPtr = lastKnownPixel;
				// destDepthData[y * nXRes + x] = lastKnownPixel;
			}
		}
	}

	// qDebug() << "Leaving:\tbool SimpleHeadTracking::FillShadow()";

	return true;
}

bool SimpleHeadTracking::SmoothDepthData(const XnDepthPixel * srcDepthData, 
										 XnDepthPixel * destDepthData, 
										 int nXRes, 
										 int nYRes, 
										 int smoothWidth, 
										 int smoothHeight)
{
	// qDebug() << "Entering:\tbool SimpleHeadTracking::SmoothDepthData()";

	if (smoothHeight <= 0 || smoothWidth <= 0) { return false; }
	if (srcDepthData == NULL || destDepthData == NULL) { return false; }

	xnOSMemCopy(destDepthData, srcDepthData, nXRes * nYRes * sizeof(XnDepthPixel));

	XnDepthPixel * destDepthDataPtr = destDepthData;
	// int kernelNumber = (2 * smoothHeight + 1) * (2 * smoothWidth + 1);

	for (int i = 0; i < nYRes; ++i) 
	{
		for (int j = 0; j < nXRes; ++j, ++destDepthDataPtr) 
		{
			int total = 0;
			int count = 0;

			for (int m = i - smoothHeight; m <= i + smoothHeight; ++m) 
			{
				if ( m < 0 ) { continue; }
				if ( m >= nYRes ) { break; }

				for (int n = j - smoothWidth; n <= j + smoothWidth; ++n) 
				{
					if (n >= 0 && n < nXRes)
					{
						total += destDepthData[m * nXRes + n]; 
						count = count + 1;
					}
				}
			}

			*destDepthDataPtr = (float) ( (float)(total) / (float)(count) );
			// *destDepthDataPtr = (total - (*destDepthDataPtr)) / (kernelNumber - 1);
			// destDepthData[i * nXRes + j] = (total - srcDepthData[i * nXRes + j]) / (kernelNumber - 1);
		}
	}

	/*
	for (int i = 0; i < nYRes; ++i) 
	{
		for (int j = 0; j < nXRes; ++j) 
		{
			if (i - 1 >= 0) { destDepthData[i * nXRes + j] += destDepthData[(i - 1) * nXRes + j]; }
			if (j - 1 >= 0) { destDepthData[i * nXRes + j] += destDepthData[i * nXRes + (j - 1)]; }
			if (i - 1 >= 0 && j - 1 >= 0) { destDepthData[i * nXRes + j] += destDepthData[(i - 1) * nXRes + (j - 1)]; }
		}
	}
	
	int A, B, C, D;
	
	for (int i = 0; i < nYRes; ++i) 
	{
		for (int j = 0; j < nXRes; ++j) 
		{
			if (i - smoothHeight < 0 || j - smoothWidth < 0) { A = 0; }
			else { A = destDepthData[(i-smoothHeight) * nXRes + (j-smoothWidth)]; }

			if (i + smoothHeight >= nYRes || j - smoothWidth < 0) { B = 0; }
			else { B = destDepthData[(i+smoothHeight) * nXRes + (j-smoothWidth)]; }

			if (i - smoothHeight < 0 || j + smoothWidth >= nXRes) { C = 0; }
			else { C = destDepthData[(i-smoothHeight) * nXRes + (j+smoothWidth)]; }
			
			if (i + smoothHeight >= nYRes || j + smoothWidth >= nXRes) { D = 0; }
			else { D = destDepthData[(i+smoothHeight) * nXRes + (j+smoothWidth)]; }

			XnDepthPixel finalTmpValue = (A - B - C + D) / ((2 * smoothWidth + 1) * (2 * smoothHeight + 1));
			destDepthData[i * nXRes + j] = (finalTmpValue >= 0) ? (finalTmpValue) : 0;
			destDepthData[i * nXRes + j] = (finalTmpValue < 10000) ? (finalTmpValue) : 9999;
		}
	}
	*/

	// delete [] tmpDepthData;

	// qDebug() << "Leaving:\tbool SimpleHeadTracking::SmoothDepthData()";

	return true;
}

int SimpleHeadTracking::FindLocalMinimumOfRow(const XnDepthPixel * srcDepthData, 
											  int nXRes,
											  int rowIndex,
											  const xn::DepthGenerator & depthGen)
{
	// qDebug() << "Entering:\tint SimpleHeadTracking::FindLocalMinimumOfRow()";

	int leftDeltaLen, rightDeltaLen;
	const XnDepthPixel * pDepthData = srcDepthData + rowIndex * nXRes + 1;
	
	XnDepthPixel tmp;

	for (int i = 1; i < nXRes - 1; ++i, ++pDepthData) 
	{
		tmp = (*pDepthData);
		if ( (tmp == 0) || tmp > *(pDepthData-1) || tmp > *(pDepthData+1) )	{			// Not Local Minimum Value
			continue;
		}
		
		if ( IsPossiblyOnHeadFast(srcDepthData, nXRes, rowIndex, i, leftDeltaLen, rightDeltaLen, depthGen) == true ) 
		{
			// qDebug() << "Leaving:\tint SimpleHeadTracking::FindLocalMinimumOfRow()";
			return i + (rightDeltaLen - leftDeltaLen) / 2;
		}

		i += rightDeltaLen;
	}

	//for (int i = 1; i < nXRes - 1; ++i, ++pDepthData) 
	//{
	//	//XnDepthPixel prevPixel = srcDepthData[rowIndex * nXRes + (i-1)];
	//	//XnDepthPixel postPixel = srcDepthData[rowIndex * nXRes + (i+1)];
	//	//XnDepthPixel curPixel = srcDepthData[rowIndex * nXRes + i];

	//	//if (!(curPixel <= prevPixel && curPixel <= postPixel))								// Not Local Minimum Value
	//	//{
	//	//	continue;
	//	//}

	//	if ( (*pDepthData) > *(pDepthData-1) || (*pDepthData) > *(pDepthData+1) )				// Not Local Minimum Value
	//	{
	//		continue;
	//	}
	//	
	//	if ( IsPossiblyOnHead(srcDepthData, nXRes, rowIndex, i, depthGen) == true )
	//	{
	//		qDebug() << "Leaving:\tint SimpleHeadTracking::FindLocalMinimumOfRow()";
	//		return i;
	//	}
	//}

	// qDebug() << "Leaving:\tint SimpleHeadTracking::FindLocalMinimumOfRow()";

	return (-1);						// Not Find Local Minimum Point Of Line #rowIndex
}

bool SimpleHeadTracking::IsPossiblyOnHeadFast(const XnDepthPixel * srcDepthData, int nXRes, 
											  int rowIndex, int colIndex, 
											  int & leftDeltaLen, int & rightDeltaLen,
											  const xn::DepthGenerator & depthGen)
{
	const int baseIndex = rowIndex * nXRes + colIndex;
	double distance = srcDepthData[baseIndex];

	int leftInnerDeltaIndex = 1;
	while (colIndex - leftInnerDeltaIndex >= 0)
	{
		XnDepthPixel tmp = srcDepthData[baseIndex - leftInnerDeltaIndex];
		if (distance > tmp || tmp - distance > 150) {							// 100 mm ( 10 cm )
			break;
		}

		++leftInnerDeltaIndex;
	}

	int rightInnerDeltaIndex = 1;
	while (colIndex + rightInnerDeltaIndex < nXRes) 
	{
		XnDepthPixel tmp = srcDepthData[baseIndex + rightInnerDeltaIndex];
		if (distance > tmp || tmp - distance > 150) {							// 100 mm ( 10 cm )
			break;
		}

		++rightInnerDeltaIndex;
	}

	leftDeltaLen = leftInnerDeltaIndex - 1;
	rightDeltaLen = rightInnerDeltaIndex - 1;

	int leftInnerIndex = colIndex - leftDeltaLen;
	int rightInnerIndex = colIndex + rightDeltaLen;

	XnPoint3D projectiveBoundPts[2];
	XnPoint3D realBoundPts[2];

	projectiveBoundPts[0].X = leftInnerIndex;
	projectiveBoundPts[0].Y = rowIndex;
	projectiveBoundPts[0].Z = srcDepthData[rowIndex * nXRes + leftInnerIndex];

	projectiveBoundPts[1].X = rightInnerIndex;
	projectiveBoundPts[1].Y = rowIndex;
	projectiveBoundPts[1].Z = srcDepthData[rowIndex * nXRes + rightInnerIndex];

	// 
	// Convert Depth Map into Real World Coordinates
	// 
	
	depthGen.ConvertProjectiveToRealWorld(2, projectiveBoundPts, realBoundPts);

	if (realBoundPts[1].X - realBoundPts[0].X < 150 || realBoundPts[1].X - realBoundPts[0].X > 225) {
		return false;
	}

	// -----------------------------------------------------------
	// -----------------------------------------------------------
	
	XnPoint3D projectiveCenterPt;
	XnPoint3D realCenterPt;

	projectiveCenterPt.X = (leftInnerIndex + rightInnerIndex) / 2;
	projectiveCenterPt.Y = rowIndex;
	projectiveCenterPt.Z = distance;

	depthGen.ConvertProjectiveToRealWorld(1, &projectiveCenterPt, &realCenterPt);
	
	// -----------------------------------------------------------
	
	XnPoint3D leftOuterBoundProjPts[2], rightOuterBoundProjPts[2];
	XnPoint3D leftOuterBoundRealPts[2], rightOuterBoundRealPts[2];

	leftOuterBoundRealPts[0].X = realCenterPt.X - 300;
	leftOuterBoundRealPts[0].Y = realCenterPt.Y;
	leftOuterBoundRealPts[0].Z = realCenterPt.Z;
	leftOuterBoundRealPts[1].X = realCenterPt.X - 200;
	leftOuterBoundRealPts[1].Y = realCenterPt.Y;
	leftOuterBoundRealPts[1].Z = realCenterPt.Z;

	rightOuterBoundRealPts[0].X = realCenterPt.X + 200;
	rightOuterBoundRealPts[0].Y = realCenterPt.Y;
	rightOuterBoundRealPts[0].Z = realCenterPt.Z;
	rightOuterBoundRealPts[1].X = realCenterPt.X + 300;
	rightOuterBoundRealPts[1].Y = realCenterPt.Y;
	rightOuterBoundRealPts[1].Z = realCenterPt.Z;

	depthGen.ConvertRealWorldToProjective(2, leftOuterBoundRealPts, leftOuterBoundProjPts);
	depthGen.ConvertRealWorldToProjective(2, rightOuterBoundRealPts, rightOuterBoundProjPts);
	
	int leftOuterBoundIndex[2], rightOuterBoundIndex[2];

	// leftOuterBoundIndex[0] = leftInnerIndex - 15;
	leftOuterBoundIndex[0] = leftOuterBoundProjPts[0].X;
	leftOuterBoundIndex[1] = leftOuterBoundProjPts[1].X;
	
	// rightOuterBoundIndex[1] = rightInnerIndex + 15;
	rightOuterBoundIndex[0] = rightOuterBoundProjPts[0].X;
	rightOuterBoundIndex[1] = rightOuterBoundProjPts[1].X;

	for (int i = 0; i < 2; ++i) 
	{
		if (leftOuterBoundIndex[i] < 0 || leftOuterBoundIndex[i] >= nXRes) {
			return false;
		}
	}

	for (int i = 0; i < 2; ++i) {
		if (rightOuterBoundIndex[i] < 0 || rightOuterBoundIndex[i] >= nXRes) {
			return false;
		}
	}

	int bi = rowIndex * nXRes;

	for (int i = leftOuterBoundIndex[0]; i < leftOuterBoundIndex[1]; ++i) 
	{
		if (srcDepthData[bi+i] > 0 && srcDepthData[bi+i] - distance < 250/* && srcDepthData[bi + i] - distance > -220*/) {
			return false;
		}
	}

	for (int i = rightOuterBoundIndex[0]; i <= rightOuterBoundIndex[1]; ++i)
	{
		if (srcDepthData[bi+i] > 0 && srcDepthData[bi + i] - distance < 250/* && srcDepthData[bi + i] - distance > -220*/) {
			return false;
		}
	}

	/*
	if (leftOuterIndex < 0) { leftOuterIndex = 0; return false; }
	if (rightOuterIndex >= nXRes) { rightOuterIndex = nXRes - 1; return false; }

	int leftOuterDepth = srcDepthData[rowIndex * nXRes + leftOuterIndex];
	int rightOuterDepth = srcDepthData[rowIndex * nXRes + rightOuterIndex];

	if (leftOuterDepth - distance <= 200 || rightOuterDepth - distance <= 200) {
		return false;
	}*/

	return true;
}

bool SimpleHeadTracking::IsPossiblyOnHead(const XnDepthPixel * srcDepthData, 
										  int nXRes, 
										  int rowIndex, 
										  int colIndex, 
										  const xn::DepthGenerator & depthGen)
{
	const int baseIndex = rowIndex * nXRes + colIndex;
	double distance = srcDepthData[baseIndex];

	int leftInnerDeltaIndex = 1;
	while (colIndex - leftInnerDeltaIndex >= 0)
	{
		if (srcDepthData[baseIndex - leftInnerDeltaIndex] - distance > 100) {				// 100 mm ( 10 cm )
			break;
		}

		++leftInnerDeltaIndex;
	}

	int rightInnerDeltaIndex = 1;
	while (colIndex + rightInnerDeltaIndex < nXRes) 
	{
		if (srcDepthData[baseIndex + rightInnerDeltaIndex] - distance > 100) {				// 100 mm ( 10 cm )
			break;
		}

		++rightInnerDeltaIndex;
	}

	int leftInnerIndex = colIndex - (leftInnerDeltaIndex - 1);
	int rightInnerIndex = colIndex + (rightInnerDeltaIndex - 1);

	XnPoint3D projectiveBoundPts[2];
	XnPoint3D realBoundPts[2];

	projectiveBoundPts[0].X = leftInnerIndex;
	projectiveBoundPts[0].Y = rowIndex;
	projectiveBoundPts[0].Z = srcDepthData[rowIndex * nXRes + leftInnerIndex];

	projectiveBoundPts[1].X = rightInnerIndex;
	projectiveBoundPts[1].Y = rowIndex;
	projectiveBoundPts[1].Z = srcDepthData[rowIndex * nXRes + rightInnerIndex];

	// 
	// Convert Depth Map into Real World Coordinates
	// 
	
	depthGen.ConvertProjectiveToRealWorld(2, projectiveBoundPts, realBoundPts);

	if (realBoundPts[1].X - realBoundPts[0].X < 100 || realBoundPts[1].X - realBoundPts[0].X > 250) {
		return false;
	}

	// ---------------------------------------------------------

	XnPoint3D projectiveCenterPt;
	XnPoint3D realCenterPt;

	projectiveCenterPt.X = colIndex;
	projectiveCenterPt.Y = rowIndex;
	projectiveCenterPt.Z = distance;

	depthGen.ConvertProjectiveToRealWorld(1, &projectiveCenterPt, &realCenterPt);

	// ---------------------------------------------------------

	XnPoint3D outerBoundProjPts[2];
	XnPoint3D outerBoundRealPts[2];

	outerBoundRealPts[0].X = realCenterPt.X - 250;
	outerBoundRealPts[0].Y = realCenterPt.Y;
	outerBoundRealPts[0].Z = realCenterPt.Z;

	outerBoundRealPts[1].X = realCenterPt.X + 250;
	outerBoundRealPts[1].Y = realCenterPt.Y;
	outerBoundRealPts[1].Z = realCenterPt.Z;

	depthGen.ConvertRealWorldToProjective(2, outerBoundRealPts, outerBoundProjPts);
	
	int leftOuterIndex = outerBoundProjPts[0].X;
	int rightOuterIndex = outerBoundProjPts[1].X;
	
	if (leftOuterIndex < 0) { leftOuterIndex = 0; }
	else if (leftOuterIndex >= nXRes) { leftOuterIndex = nXRes - 1; }

	if (rightOuterIndex < 0) { rightOuterIndex = 0; }
	else if (rightOuterIndex >= nXRes) { rightOuterIndex = nXRes - 1; }

	int leftOuterDepth = srcDepthData[rowIndex * nXRes + leftOuterIndex];
	int rightOuterDepth = srcDepthData[rowIndex * nXRes + rightOuterIndex];

	if (leftOuterDepth - distance <= 200 || rightOuterDepth - distance <= 200) {
		return false;
	}

	return true;

	// int lowBound = (int)(CalcMatchingPixelsNumber(640, 1.025, 100, distance) / 2);
	// int highBound = (int)(CalcMatchingPixelsNumber(640, 1.025, 250, distance) / 2);

	/*
	for (int i = (-1) * lowBound; i <= lowBound; ++i) 
	{
		if (srcDepthData[rowIndex * nXRes + i + colIndex] - distance > 100) {
			return false;
		}
	}

	if (srcDepthData[rowIndex * nXRes + highBound + colIndex] - distance >= 200 &&
		srcDepthData[rowIndex * nXRes - highBound + colIndex] - distance >= 200)
	{
		return true;
	}
	else
	{
		return false;
	}
	*/
}

bool SimpleHeadTracking::FindHeadCoordinate(const XnDepthPixel * srcDepthData, 
											int nXRes, 
											int nYRes, 
											int & rowIndex, 
											int & colIndex, 
											const xn::DepthGenerator & depthGen)
{
	if (srcDepthData == NULL) { return false; }

	int lineCount = 0;
	int totalColIndex = 0;
	int lastStartRowIndex = 0;
	
	double totalDistance = 0.0f;

	XnPoint3D projHeadCoor[2], realHeadCoor[2];

	for (int i = 0; i < nYRes; ++i)
	{
		int cIndex = SimpleHeadTracking::FindLocalMinimumOfRow(srcDepthData, nXRes, i, depthGen);
		if (cIndex == -1)
		{
			if (lineCount == 0) 
			{
				lastStartRowIndex++;
				continue;
			}

			projHeadCoor[0].X = 0;
			projHeadCoor[0].Y = lastStartRowIndex;
			projHeadCoor[0].Z = totalDistance / lineCount;
			
			projHeadCoor[1].X = 0;
			projHeadCoor[1].Y = i - 1;
			projHeadCoor[1].Z = totalDistance / lineCount;

			depthGen.ConvertProjectiveToRealWorld(2, projHeadCoor, realHeadCoor);

			if ( realHeadCoor[0].Y - realHeadCoor[1].Y < 300 && realHeadCoor[0].Y - realHeadCoor[1].Y > 200 ) 
			{
				rowIndex = lastStartRowIndex + lineCount / 2;
				colIndex = totalColIndex / lineCount;

				return true;
			}

			projHeadCoor[0].X = 0;
			projHeadCoor[0].Y = lastStartRowIndex + lineCount - 1;
			projHeadCoor[0].Z = totalDistance / lineCount;

			depthGen.ConvertProjectiveToRealWorld(1, projHeadCoor, realHeadCoor);

			if ( realHeadCoor[0].Y - realHeadCoor[1].Y >= 90 ) 
			{
				lastStartRowIndex = i + 1;
				lineCount = 0;
				totalColIndex = 0;
				totalDistance = 0.0f;
			}
		}
		else {
			lineCount += 1;
			totalColIndex += cIndex;
			totalDistance += srcDepthData[i * nXRes + cIndex];
		}
	}

	return false;
}

// 
// Get Local Minimum Depth Map
// 
	
bool SimpleHeadTracking::GetLocalMinimumOfDepthMap(const XnDepthPixel * srcDepthData, std::vector<int> & minArray, const xn::DepthGenerator & depthGen)
{
	if (srcDepthData == NULL) {
		return false;
	}

	xn::DepthMetaData depthMD;
	depthGen.GetMetaData(depthMD);

	minArray.clear();

	int nXRes = depthMD.XRes();
	int nYRes = depthMD.YRes();

	for (int i = 0; i < nYRes; ++i) {
		int index = SimpleHeadTracking::FindLocalMinimumOfRow(srcDepthData, nXRes, i, depthGen);
		minArray.push_back(index);
	}

	return true;
}
