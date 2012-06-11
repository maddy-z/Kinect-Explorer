#include <opencv2\opencv.hpp>

#include <XnOpenNI.h>
#include <XnCppWrapper.h>

#include "KinectHoleFiller.h"

#include <qimage.h>
#include <qdebug.h>

bool KinectHoleFiller::NearestNeighborhoodHoleFilling(const cv::Mat & srcDepthMat, cv::Mat & destDepthMat)
{
	qDebug() << "Entering:\tbool KinectHoleFiller::NearestNeighborHoleFilling()";

	assert (srcDepthMat.size() == destDepthMat.size());
	assert (srcDepthMat.type() == destDepthMat.type());
	assert (srcDepthMat.type() == CV_16UC1);

	if (srcDepthMat.empty() || destDepthMat.empty()) {
		qDebug() << "Leaving:\tbool KinectHoleFiller::NearestNeighborHoleFilling()";
		return false;
	}

	int depthHeight = srcDepthMat.size().height, depthWidth = srcDepthMat.size().width;
	int srcRowStep = srcDepthMat.step, destRowStep = destDepthMat.step;

	const uchar * srcRowPtr = NULL;
	const XnDepthPixel * srcDataPtr = NULL;
	uchar * destRowPtr = NULL;
	XnDepthPixel * destDataPtr = NULL;

	// 
	// Create Distance Map
	// 

	int * const distMap = new int[depthHeight * depthWidth];
	int * distRowPtr = NULL, * distDataPtr = NULL;
	int adj[4];

	// First Scan
	srcRowPtr = srcDepthMat.data;
	destRowPtr = destDepthMat.data;
	distRowPtr = distMap;

	srcDataPtr = (const XnDepthPixel *)(srcRowPtr);
	destDataPtr = (XnDepthPixel *)(destRowPtr);
	distDataPtr = distRowPtr;

	if (*srcDataPtr) {
		*distDataPtr = 0;
		*destDataPtr = *srcDataPtr;
	}
	else {
		*distDataPtr = depthWidth + 1;
		*destDataPtr = 0;
	}

	for (int x = 1; x < depthWidth; ++x) 
	{
		XnDepthPixel tmp = srcDataPtr[x];
		
		if (tmp) {
			distDataPtr[x] = 0;
			destDataPtr[x] = tmp;
		}
		else 
		{
			if (distDataPtr[x-1] + 1 < depthWidth + 1) {
				destDataPtr[x] = destDataPtr[x-1];
				distDataPtr[x] = distDataPtr[x-1] + 1;
			}
			else {
				distDataPtr[x] = depthWidth + 1;
				destDataPtr[x] = 0;
			}
		}
	}
	srcRowPtr += srcRowStep;
	destRowPtr += destRowStep;
	distRowPtr += depthWidth;

	for (int y = 1; y < depthHeight; ++y, srcRowPtr += srcRowStep, destRowPtr += destRowStep, distRowPtr += depthWidth) 
	{
		srcDataPtr = (const XnDepthPixel *)(srcRowPtr);
		destDataPtr = (XnDepthPixel *)(destRowPtr);
		distDataPtr = distRowPtr;

		// 
		// x = 0
		// 
		if (*srcDataPtr) {
			*distDataPtr = 0;
			*destDataPtr = *srcDataPtr;
		}
		else {
			adj[0] = *(distDataPtr-depthWidth);
			adj[1] = *(distDataPtr-depthWidth+1);

			if (adj[0] < adj[1]) { 
				*distDataPtr = adj[0] + 1;
				*destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr) - destRowStep);
			}
			else {
				*distDataPtr = adj[1] + 1;
				*destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr+1) - destRowStep);
			}
		}
		++srcDataPtr;
		++destDataPtr;
		++distDataPtr;

		for ( int x = 1; x < depthWidth - 1; ++x, ++srcDataPtr, ++distDataPtr, ++destDataPtr ) 
		{
			if ( *srcDataPtr ) 
			{
				*distDataPtr = 0;
				*destDataPtr = *srcDataPtr;
			}
			else 
			{
				// 8-adjant Points
				adj[0] = *(distDataPtr-1);
				adj[1] = *(distDataPtr-depthWidth-1);
				adj[2] = *(distDataPtr-depthWidth);
				adj[3] = *(distDataPtr-depthWidth+1);

				int min = adj[0];
				int min_i = 0;
				for (int k = 1; k < 4; ++k) 
				{
					if (adj[k] < min) {
						min = adj[k];
						min_i = k;
					}
				}

				*distDataPtr = min + 1;
				
				switch ( min_i ) 
				{
				case 0: *destDataPtr = *(destDataPtr-1); break;
				case 1: *destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr-1)-destRowStep); break;
				case 2: *destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr)-destRowStep); break;
				case 3: *destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr+1)-destRowStep); break;
				}
			}
		}

		// 
		// x = depthWidth - 1
		// 
		if (*srcDataPtr) {
			*distDataPtr = 0;
			*destDataPtr = *srcDataPtr;
		}
		else {
			adj[0] = *(distDataPtr-1);
			adj[1] = *(distDataPtr-depthWidth-1);
			adj[2] = *(distDataPtr-depthWidth);

			int min = adj[0];
			int min_i = 0;
			for (int k = 1; k < 3; ++k) 
			{
				if (adj[k] < min) {
					min = adj[k];
					min_i = k;
				}
			}

			*distDataPtr = min + 1;
			
			switch (min_i) 
			{
			case 0: *destDataPtr = *(destDataPtr-1); break;
			case 1: *destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr-1)-destRowStep); break;
			case 2: *destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr)-destRowStep); break;
			}
		}
	}

	// 
	// Second Scan
	// 

	destRowPtr = destDepthMat.data + (depthHeight - 1) * destRowStep + (depthWidth - 1) * sizeof(XnDepthPixel);
	distRowPtr = distMap + depthHeight * depthWidth - 1;

	destDataPtr = (XnDepthPixel *)(destRowPtr);
	distDataPtr = distRowPtr;

	if (*distDataPtr > depthWidth + 1) 
	{
		*distDataPtr = depthWidth + 1;
		*destDataPtr = 0;
	}

	--destDataPtr;
	--distDataPtr;

	for (int x = depthWidth - 2; x >= 0; --x, --destDataPtr, --distDataPtr) 
	{
		if (*distDataPtr > *(distDataPtr+1) + 1) 
		{
			*distDataPtr = *(distDataPtr+1) + 1;
			*destDataPtr = *(destDataPtr+1);
		}

		if (*distDataPtr > depthWidth + 1) {
			*distDataPtr = depthWidth + 1;
			*destDataPtr = 0;
		}
	}

	destRowPtr -= destRowStep;
	distRowPtr -= depthWidth;

	for (int y = depthHeight - 2; y >= 0; --y, destRowPtr -= destRowStep, distRowPtr -= depthWidth) 
	{
		destDataPtr = (XnDepthPixel *)(destRowPtr);
		distDataPtr = distRowPtr;

		// 
		// x = depthWidth - 1
		// 
		
		if (*distDataPtr > 0)
		// else 
		{
			adj[0] = *(distDataPtr+depthWidth);
			adj[1] = *(distDataPtr+depthWidth-1);

			int min, min_i;

			if (adj[0] < adj[1]) { min = adj[0]; min_i = 0; }
			else { min = adj[1]; min_i = 1; }

			if (*distDataPtr >= min + 1) 
			{
				*distDataPtr = min + 1;

				switch (min_i) 
				{
				case 0: *destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr) + destRowStep); break;
				case 1: *destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr-1) + destRowStep); break;
				}
			}
		}

		--destDataPtr;
		--distDataPtr;

		for (int x = depthWidth - 2; x >= 1; --x, --distDataPtr, --destDataPtr) 
		{
			/*
			if (*distDataPtr == 0) 
			{
				continue;
			}
			else */
			
			if (*distDataPtr > 0)
			{
				// 8-adjant Points
				adj[0] = *(distDataPtr + 1);
				adj[1] = *(distDataPtr + depthWidth + 1);
				adj[2] = *(distDataPtr + depthWidth);
				adj[3] = *(distDataPtr + depthWidth - 1);

				int min = adj[0];
				int min_i = 0;
				for (int k = 1; k < 4; ++k) 
				{
					if (adj[k] < min) {
						min = adj[k];
						min_i = k;
					}
				}

				if ( *distDataPtr > min + 1 ) 
				{
					*distDataPtr = min + 1;
				
					switch (min_i) 
					{
					case 0: *destDataPtr = *(destDataPtr+1); break;
					case 1: *destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr+1) + destRowStep); break;
					case 2: *destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr) + destRowStep); break;
					case 3: *destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr-1) + destRowStep); break;
					}
				}
			}
		}

		// 
		// x = 0
		// 

		/*
		if (*distDataPtr == 0) {
			;
		}
		else {
		*/
		
		if (*distDataPtr > 0) 
		{ 
			adj[0] = *(distDataPtr + 1);
			adj[1] = *(distDataPtr + depthWidth + 1);
			adj[2] = *(distDataPtr + depthWidth);

			int min = adj[0];
			int min_i = 0;
			for (int k = 1; k < 3; ++k) 
			{
				if (adj[k] < min) {
					min = adj[k];
					min_i = k;
				}
			}

			if ( *distDataPtr > min + 1 ) 
			{ 

				*distDataPtr = min + 1;
			
				switch (min_i) 
				{
				case 0: *destDataPtr = *(destDataPtr+1); break;
				case 1: *destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr+1) + destRowStep); break;
				case 2: *destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr) + destRowStep); break;
				}
			}
		}
	}

	delete [] distMap;

	qDebug() << "Leaving:\tbool KinectHoleFiller::NearestNeighborHoleFilling()";

	return true;
}

bool KinectHoleFiller::FmmInpaintHoleFilling(const cv::Mat & srcDepthMat, cv::Mat & destDepthMat)
{
	// TODO:
	return false;
}