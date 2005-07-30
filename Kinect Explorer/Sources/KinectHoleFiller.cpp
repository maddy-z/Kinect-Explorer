#include <opencv2\opencv.hpp>

#include <XnOpenNI.h>
#include <XnCppWrapper.h>

#include <qimage.h>
#include <qdebug.h>

#include "KinectHoleFiller.h"

bool 
KinectHoleFiller::NearestNeighborhoodHoleFilling  ( const cv::Mat & srcDepthMat, cv::Mat & destDepthMat )
{
	qDebug() << "Entering:\tbool KinectHoleFiller::NearestNeighborHoleFilling()";

	assert ( srcDepthMat.size() == destDepthMat.size() );
	assert ( srcDepthMat.type() == destDepthMat.type() );
	assert ( srcDepthMat.type() == CV_16UC1 );

	if ( srcDepthMat.empty() || destDepthMat.empty() ) {
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

	if ( *srcDataPtr ) {
		*distDataPtr = 0;
		*destDataPtr = *srcDataPtr;
	}
	else {
		*distDataPtr = depthWidth + 1;
		*destDataPtr = 0;
	}

	for ( int x = 1; x < depthWidth; ++x ) 
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

	for ( int y = 1; y < depthHeight; ++y, srcRowPtr += srcRowStep, destRowPtr += destRowStep, distRowPtr += depthWidth ) 
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
		else 
		{
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
				case 0:		*destDataPtr = *(destDataPtr-1); break;
				case 1:		*destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr-1)-destRowStep); break;
				case 2:		*destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr)-destRowStep); break;
				case 3:		*destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr+1)-destRowStep); break;
				}
			}
		}

		// 
		// x = depthWidth - 1
		// 

		if ( *srcDataPtr ) {
			*distDataPtr = 0;
			*destDataPtr = *srcDataPtr;
		}
		else 
		{
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
			case 0:		*destDataPtr = *(destDataPtr-1); break;
			case 1:		*destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr-1)-destRowStep); break;
			case 2:		*destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr)-destRowStep); break;
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

	for ( int y = depthHeight - 2; y >= 0; --y, destRowPtr -= destRowStep, distRowPtr -= depthWidth ) 
	{
		destDataPtr = (XnDepthPixel *)(destRowPtr);
		distDataPtr = distRowPtr;

		// 
		// x = depthWidth - 1
		// 
		
		if ( *distDataPtr > 0 )
		// else 
		{
			adj[0] = *(distDataPtr+depthWidth);
			adj[1] = *(distDataPtr+depthWidth-1);

			int min, min_i;

			if (adj[0] < adj[1]) { min = adj[0]; min_i = 0; }
			else { min = adj[1]; min_i = 1; }

			if ( *distDataPtr >= min + 1 ) 
			{
				*distDataPtr = min + 1;

				switch (min_i) 
				{
				case 0:		*destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr) + destRowStep);		break;
				case 1:		*destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr-1) + destRowStep);	break;
				}
			}
		}

		--destDataPtr;
		--distDataPtr;

		for (int x = depthWidth - 2; x >= 1; --x, --distDataPtr, --destDataPtr) 
		{	
			if (*distDataPtr > 0)
			{
				// 8-Adjant Points
				adj[0] = *(distDataPtr + 1);
				adj[1] = *(distDataPtr + depthWidth + 1);
				adj[2] = *(distDataPtr + depthWidth);
				adj[3] = *(distDataPtr + depthWidth - 1);

				int min = adj[0];
				int min_i = 0;
				for ( int k = 1; k < 4; ++k ) 
				{
					if ( adj[k] < min ) {
						min = adj[k];
						min_i = k;
					}
				}

				if ( *distDataPtr > min + 1 ) 
				{
					*distDataPtr = min + 1;
				
					switch ( min_i ) 
					{
					case 0:		*destDataPtr = *(destDataPtr+1);											break;
					case 1:		*destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr+1) + destRowStep);	break;
					case 2:		*destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr) + destRowStep);		break;
					case 3:		*destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr-1) + destRowStep);	break;
					}
				}
			}
		}

		// 
		// x = 0
		// 
		
		if ( *distDataPtr > 0 ) 
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
				case 0:		*destDataPtr = *(destDataPtr+1);											break;
				case 1:		*destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr+1) + destRowStep);	break;
				case 2:		*destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr) + destRowStep);		break;
				}
			}
		}
	}

	delete [] distMap;

	qDebug() << "Leaving:\tbool KinectHoleFiller::NearestNeighborHoleFilling()";

	return true;
}

bool 
KinectHoleFiller::NearestNeighborhoodHoleFilling2 ( const cv::Mat & srcDepthMat, cv::Mat & destDepthMat, int maxDepth )
{
	assert ( srcDepthMat.size() == destDepthMat.size() );
	assert ( srcDepthMat.type() == destDepthMat.type() );
	assert ( srcDepthMat.type() == CV_16UC1 );

	if ( srcDepthMat.empty() || destDepthMat.empty() ) {
		return false;
	}

	int depthWidth = srcDepthMat.size().width, depthHeight = srcDepthMat.size().height;
	int srcRowStep = srcDepthMat.step, destRowStep = destDepthMat.step;

	const uchar * srcRowPtr				= NULL;
	const XnDepthPixel * srcDataPtr		= NULL;
	uchar * destRowPtr					= NULL;
	XnDepthPixel * destDataPtr			= NULL;

	// 
	// Create Distance Map
	// 
	
	int adj[4];
	int * distRowPtr = NULL, * distDataPtr = NULL;
	int * const distMap = new int[depthHeight * depthWidth];
	
	memset ( distMap, 0, depthHeight * depthWidth * sizeof (int) );

	// 
	// First Scan
	// 
	
	srcRowPtr  = srcDepthMat.data	+ srcRowStep;
	destRowPtr = destDepthMat.data	+ destRowStep;
	distRowPtr = distMap			+ depthWidth;

	for ( int y = 1; y < depthHeight - 1; ++y, srcRowPtr += srcRowStep, destRowPtr += destRowStep, distRowPtr += depthWidth ) 
	{
		srcDataPtr = (const XnDepthPixel *)(srcRowPtr) + 1;
		destDataPtr = (XnDepthPixel *)(destRowPtr) + 1;
		distDataPtr = (distRowPtr) + 1;

		for ( int x = 1; x < depthWidth - 1; ++x, ++srcDataPtr, ++destDataPtr, ++distDataPtr ) 
		{
			if ( (*srcDataPtr) && (*srcDataPtr) <= maxDepth ) {
				*distDataPtr = 0;
				*destDataPtr = *srcDataPtr;
			}
			else if ( (*srcDataPtr) == 0 ) 
			{
				adj[0] = *(distDataPtr - 1) + 1;
				adj[1] = *(distDataPtr - depthWidth - 1) + 2;
				adj[2] = *(distDataPtr - depthWidth) + 1;
				adj[3] = *(distDataPtr - depthWidth + 1) + 2;
			
				int min = adj[0];
				int min_i = 0;
		
				for ( int k = 1; k < 4; ++k ) {
					if ( adj[k] < min ) {
						min = adj[k]; min_i = k;
					}
				}
				
				*distDataPtr = min;
				
				switch ( min_i ) 
				{
				case 0:		*destDataPtr = *(XnDepthPixel *)(destDataPtr - 1);								break;
				case 1:		*destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr - 1) - destRowStep);		break;
				case 2:		*destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr) - destRowStep);			break;
				case 3:		*destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr + 1) - destRowStep);		break;					
				}
			}
		}
	}

	// 
	// Second Scan
	// 

	destRowPtr = destDepthMat.data + ( depthHeight - 1 ) * ( destRowStep ) - 1;
	distRowPtr = distMap + ( depthHeight - 1 ) * ( depthWidth ) - 1;

	for ( int y = depthHeight - 2; y >= 1; --y, destRowPtr -= destRowStep, distRowPtr -= depthWidth ) 
	{
		destDataPtr = (XnDepthPixel *)(destRowPtr) - 1;
		distDataPtr = distRowPtr - 1;

		for ( int x = depthWidth - 2; x >= 1; --x, --distDataPtr, --destDataPtr ) 
		{
			if ( (*distDataPtr) > 0 )
			{
				adj[0] = *(distDataPtr + 1)				 + 1;
				adj[1] = *(distDataPtr + depthWidth + 1) + 2;
				adj[2] = *(distDataPtr + depthWidth)	 + 1;
				adj[3] = *(distDataPtr + depthWidth - 1) + 2;

				int min = adj[0];
				int min_i = 0;
				for ( int k = 1; k < 4; ++k ) 
				{
					if ( adj[k] < min ) {
						min = adj[k];
						min_i = k;
					}
				}

				if ( *distDataPtr > min ) 
				{
					*distDataPtr = min;
				
					switch ( min_i ) 
					
					{
					
					case 0:		
						
						if (*destDataPtr < *(XnDepthPixel *)(destDataPtr + 1)) {
							*destDataPtr = *(XnDepthPixel *)(destDataPtr + 1);									
						}

						break;

					case 1:		
						
						if (*destDataPtr < *(XnDepthPixel *)((uchar *)(destDataPtr + 1) + destRowStep)) {
							*destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr + 1) + destRowStep);		
						}

						break;

					case 2:		
						
						if (*destDataPtr < *(XnDepthPixel *)((uchar *)(destDataPtr) + destRowStep)) {
							*destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr) + destRowStep);			
						}

						break;

					case 3:		
						
						if (*destDataPtr < *(XnDepthPixel *)((uchar *)(destDataPtr - 1) + destRowStep)) {
							*destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr - 1) + destRowStep);		
						}

						break;

					}
				}
			}
		}
	}

	delete [] distMap;

	return true;
}

bool 
KinectHoleFiller::NearestNeighborhoodHoleFilling3 ( const cv::Mat & srcDepthMat, cv::Mat & destDepthMat, int maxDepth )
{
	assert ( srcDepthMat.size() == destDepthMat.size() );
	assert ( srcDepthMat.type() == destDepthMat.type() );
	assert ( srcDepthMat.type() == CV_16UC1 );

	if ( srcDepthMat.empty() || destDepthMat.empty() ) {
		return false;
	}

	int depthWidth = srcDepthMat.size().width, depthHeight = srcDepthMat.size().height;
	int srcRowStep = srcDepthMat.step, destRowStep = destDepthMat.step;

	const uchar * srcRowPtr			= NULL;
	const XnDepthPixel * srcDataPtr = NULL;
	uchar * destRowPtr				= NULL;
	XnDepthPixel * destDataPtr		= NULL;

	// 
	// Create Distance Map
	// 
	
	int adj[4], adjValue[4];
	int * distRowPtr = NULL, * distDataPtr = NULL;
	int * const distMap = new int[depthHeight * depthWidth];
	
	memset ( distMap, 0, depthHeight * depthWidth * sizeof (int) );

	// 
	// First Scan
	// 
	
	srcRowPtr  = srcDepthMat.data	+ srcRowStep;
	destRowPtr = destDepthMat.data	+ destRowStep;
	distRowPtr = distMap			+ depthWidth;

	for ( int y = 1; y < depthHeight - 1; ++y, srcRowPtr += srcRowStep, destRowPtr += destRowStep, distRowPtr += depthWidth ) 
	{
		srcDataPtr = (const XnDepthPixel *)(srcRowPtr) + 1;
		destDataPtr = (XnDepthPixel *)(destRowPtr) + 1;
		distDataPtr = (distRowPtr) + 1;

		for ( int x = 1; x < depthWidth - 1; ++x, ++srcDataPtr, ++destDataPtr, ++distDataPtr ) 
		{
			if ( (*srcDataPtr) && (*srcDataPtr) <= maxDepth ) {
				*distDataPtr = 0;
				*destDataPtr = *srcDataPtr;
			}
			else if ( (*srcDataPtr) == 0 ) 
			{
				/*
				adj[0] = *(distDataPtr - 1) + 1;
				adj[1] = *(distDataPtr - depthWidth - 1) + 2;
				adj[2] = *(distDataPtr - depthWidth) + 1;
				adj[3] = *(distDataPtr - depthWidth + 1) + 2;
				*/

				adjValue[0] = *(XnDepthPixel *)(srcDataPtr - 1);
				adjValue[1] = *(XnDepthPixel *)((uchar *)(srcDataPtr - 1) - destRowStep);
				adjValue[2] = *(XnDepthPixel *)((uchar *)(srcDataPtr) - destRowStep);
				adjValue[3] = *(XnDepthPixel *)((uchar *)(srcDataPtr + 1) - destRowStep);

				int max = adjValue[0];
				int max_i = 0;
		
				for ( int k = 1; k < 4; ++k ) {
					if ( adjValue[k] > max ) {
						max = adjValue[k]; max_i = k;
					}
				}
				
				*distDataPtr = 1;
				*destDataPtr = max;

				/*
				switch ( max_i ) 
				{
				case 0:		*destDataPtr = *(XnDepthPixel *)(destDataPtr - 1);								break;
				case 1:		*destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr - 1) - destRowStep);		break;
				case 2:		*destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr) - destRowStep);			break;
				case 3:		*destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr + 1) - destRowStep);		break;		
				}
				*/
			}
		}
	}

	// 
	// Second Scan
	// 

	destRowPtr = destDepthMat.data + ( depthHeight - 1 ) * ( destRowStep ) - 1;
	distRowPtr = distMap + ( depthHeight - 1 ) * ( depthWidth ) - 1;

	for ( int y = depthHeight - 2; y >= 1; --y, destRowPtr -= destRowStep, distRowPtr -= depthWidth ) 
	{
		destDataPtr = (XnDepthPixel *)(destRowPtr) - 1;
		distDataPtr = distRowPtr - 1;

		for ( int x = depthWidth - 2; x >= 1; --x, --distDataPtr, --destDataPtr ) 
		{
			if ( *distDataPtr > 0  && *destDataPtr > 0 )
			{
				/*
				adj[0] = *(distDataPtr + 1)				 + 1;
				adj[1] = *(distDataPtr + depthWidth + 1) + 2;
				adj[2] = *(distDataPtr + depthWidth)	 + 1;
				adj[3] = *(distDataPtr + depthWidth - 1) + 2;
				*/

				adjValue[0] = *(XnDepthPixel *)(destDataPtr + 1);
				adjValue[1] = *(XnDepthPixel *)((uchar *)(destDataPtr + 1) + destRowStep);
				adjValue[2] = *(XnDepthPixel *)((uchar *)(destDataPtr) + destRowStep);
				adjValue[3] = *(XnDepthPixel *)((uchar *)(destDataPtr - 1) + destRowStep);

				int max = adjValue[0];
				int max_i = 0;
				
				for ( int k = 1; k < 4; ++k ) 
				{
					if ( adjValue[k] > max ) {
						max = adjValue[k];
						max_i = k;
					}
				}

				if ( max > *destDataPtr ) {
					*destDataPtr = max;
				}

				/*
				switch ( max_i ) 			
				{	
				case 0:		*destDataPtr = *(XnDepthPixel *)(destDataPtr + 1);								break;
				case 1:		*destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr + 1) + destRowStep);		break;
				case 2:		*destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr) + destRowStep);			break;
				case 3:		*destDataPtr = *(XnDepthPixel *)((uchar *)(destDataPtr - 1) + destRowStep);		break;
				}*/
			}
		}
	}

	delete [] distMap;

	return true;
}


bool 
KinectHoleFiller::DistanceTransform ( const cv::Mat & srcDepthMat, cv::Mat & destDepthMat, int maxDepth )
{
	assert ( srcDepthMat.size() == destDepthMat.size() );
	assert ( srcDepthMat.type() == destDepthMat.type() );
	assert ( srcDepthMat.type() == CV_16UC1 );

	if ( srcDepthMat.empty() || destDepthMat.empty() ) {
		return false;
	}

	int depthWidth = srcDepthMat.size().width;
	int depthHeight = srcDepthMat.size().height;
	int srcRowStep = srcDepthMat.step;
	int destRowStep = destDepthMat.step;

	const uchar * srcRowPtr = NULL;
	const XnDepthPixel * srcDataPtr = NULL;
	uchar * destRowPtr = NULL;
	XnDepthPixel * destDataPtr = NULL;

	// 
	// Create Distance Map
	// 

	int adj[4];
	int * distRowPtr = NULL, * distDataPtr = NULL;
	int * const distMap = new int[depthHeight * depthWidth];
	
	memset ( distMap, 0, depthHeight * depthWidth );

	// 
	// First Scan
	// 
	
	srcRowPtr = srcDepthMat.data;
	distRowPtr = distMap;

	srcDataPtr = (const XnDepthPixel *)(srcRowPtr);
	distDataPtr = distRowPtr;

	if ( *srcDataPtr && *srcDataPtr <= maxDepth ) {
		*distDataPtr = 0;
	}
	else if ( *srcDataPtr == 0 ) {
		*distDataPtr = depthWidth + 1;
	}

	for ( int x = 1; x < depthWidth; ++x ) 
	{
		XnDepthPixel tmp = srcDataPtr[x];
		
		if ( tmp && tmp <= maxDepth ) {
			distDataPtr[x] = 0;
		}
		else if ( tmp == 0 ) 
		{
			if ( distDataPtr[x-1] + 1 < depthWidth + 1 ) {
				distDataPtr[x] = distDataPtr[x-1] + 1;
			}
			else {
				distDataPtr[x] = depthWidth + 1;
			}
		}
	}
	srcRowPtr += srcRowStep;
	distRowPtr += depthWidth;

	for ( int y = 1; y < depthHeight; ++y, srcRowPtr += srcRowStep, distRowPtr += depthWidth ) 
	{
		srcDataPtr = (const XnDepthPixel *)(srcRowPtr);
		distDataPtr = distRowPtr;

		// 
		// x = 0
		// 

		if ( *srcDataPtr && *srcDataPtr <= maxDepth ) {
			*distDataPtr = 0;
		}
		else if ( *srcDataPtr == 0 )
		{
			adj[0] = *(distDataPtr - depthWidth);
			adj[1] = *(distDataPtr - depthWidth + 1);

			if ( adj[0] + 1 <= adj[1] + 2 ) { 
				*distDataPtr = adj[0] + 1;
			}
			else {
				*distDataPtr = adj[1] + 2;
			}
		}
		++srcDataPtr;
		++distDataPtr;

		for ( int x = 1; x < depthWidth - 1; ++x, ++srcDataPtr, ++distDataPtr ) 
		{
			if ( *srcDataPtr && *srcDataPtr <= maxDepth ) 
			{
				*distDataPtr = 0;
			}
			else if ( *srcDataPtr == 0 )
			{
				// 
				// 8-adjant Points
				// 

				adj[0] = *(distDataPtr - 1) + 1;
				adj[1] = *(distDataPtr - depthWidth - 1) + 2;
				adj[2] = *(distDataPtr - depthWidth) + 1;
				adj[3] = *(distDataPtr - depthWidth + 1) + 2;

				int min = adj[0];
				int min_i = 0;				
				for (int k = 1; k < 4; ++k) 
				{
					if (adj[k] < min) {
						min = adj[k];
						min_i = k;
					}
				}

				*distDataPtr = min;
			}
		}

		// 
		// x = depthWidth - 1
		// 

		if ( *srcDataPtr && *srcDataPtr <= maxDepth ) {
			*distDataPtr = 0;
		}
		else if ( *srcDataPtr == 0 )
		{
			adj[0] = *(distDataPtr - 1) + 1;
			adj[1] = *(distDataPtr - depthWidth - 1) + 2;
			adj[2] = *(distDataPtr - depthWidth) + 1;

			int min = adj[0];
			int min_i = 0;
			for (int k = 1; k < 3; ++k) 
			{
				if (adj[k] < min) {
					min = adj[k];
					min_i = k;
				}
			}

			*distDataPtr = min;
		}
	}

	// 
	// Second Scan
	// 

	distRowPtr = distMap + depthHeight * depthWidth - 1;
	distDataPtr = distRowPtr;

	if (*distDataPtr > depthWidth + 1) 
	{
		*distDataPtr = depthWidth + 1;
	}
	--distDataPtr;

	for ( int x = depthWidth - 2; x >= 0; --x, --distDataPtr ) 
	{
		if (*distDataPtr > *(distDataPtr+1) + 1) 
		{
			*distDataPtr = *(distDataPtr+1) + 1;
		}

		if (*distDataPtr > depthWidth + 1) {
			*distDataPtr = depthWidth + 1;
		}
	}
	distRowPtr -= depthWidth;

	for ( int y = depthHeight - 2; y >= 0; --y, distRowPtr -= depthWidth ) 
	{
		// destDataPtr = (XnDepthPixel *)(destRowPtr);
		distDataPtr = distRowPtr;

		// 
		// x = depthWidth - 1
		// 
		
		if ( *distDataPtr > 0 )
		{
			adj[0] = *( distDataPtr + depthWidth );
			adj[1] = *( distDataPtr + depthWidth - 1 );

			int min, min_i;

			if ( adj[0] + 1 < adj[1] + 2 ) { min = adj[0] + 1; min_i = 0; }
			else { min = adj[1] + 2; min_i = 1; }

			if ( *distDataPtr >= min ) 
			{
				*distDataPtr = min;
			}
		}
		--distDataPtr;

		for ( int x = depthWidth - 2; x >= 1; --x, --distDataPtr ) 
		{			
			if ( *distDataPtr > 0 )
			{
				// 
				// 8-adjant Points
				// 
				
				adj[0] = *(distDataPtr + 1) + 1;
				adj[1] = *(distDataPtr + depthWidth + 1) + 2;
				adj[2] = *(distDataPtr + depthWidth) + 1;
				adj[3] = *(distDataPtr + depthWidth - 1) + 2;

				int min = adj[0];
				int min_i = 0;
				for ( int k = 1; k < 4; ++k ) 
				{
					if (adj[k] < min) {
						min = adj[k];
						min_i = k;
					}
				}

				if ( *distDataPtr > min ) {
					*distDataPtr = min;
				}
			}
		}

		// 
		// x = 0
		// 
		
		if ( *distDataPtr > 0 ) 
		{ 
			adj[0] = *(distDataPtr + 1) + 1;
			adj[1] = *(distDataPtr + depthWidth + 1) + 2;
			adj[2] = *(distDataPtr + depthWidth) + 1;

			int min = adj[0];
			int min_i = 0;
			for ( int k = 1; k < 3; ++k ) 
			{
				if (adj[k] < min) {
					min = adj[k];
					min_i = k;
				}
			}

			if ( *distDataPtr > min ) 
			{ 
				*distDataPtr = min;
			}
		}
	}

	// 
	// Copy Distance Transform to cv::Mat
	// 

	destRowPtr = destDepthMat.data;
	distRowPtr = distMap;

	destDataPtr = (XnDepthPixel *)(destRowPtr);
	distDataPtr = distRowPtr;

	int minDist = INT_MAX, maxDist = INT_MIN;
	for ( int i = 0; i < depthHeight; ++i, distRowPtr += depthWidth ) 
	{
		distDataPtr = distRowPtr;

		for ( int j = 0; j < depthWidth; ++j, ++distDataPtr ) {
			if ( *distDataPtr < minDist ) { minDist = *distDataPtr; }
			if ( *distDataPtr > maxDist ) { maxDist = *distDataPtr; }
		}
	}

	destRowPtr = destDepthMat.data;
	distRowPtr = distMap;

	destDataPtr = (XnDepthPixel *)(destRowPtr);
	distDataPtr = distRowPtr;

	double delta = maxDist - minDist;
	for ( int i = 0; i < depthHeight; ++i, distRowPtr += depthWidth, destRowPtr += destRowStep ) 
	{
		distDataPtr = distRowPtr;
		destDataPtr = (XnDepthPixel *)(destRowPtr);

		for ( int j = 0; j < depthWidth; ++j, ++distDataPtr, ++destDataPtr ) {
			*destDataPtr = (double)((*distDataPtr) - minDist) / (delta + 1.0f) * 65535.0f;
		}
	}

	delete [] distMap;

	return true;
}

bool 
KinectHoleFiller::DistanceTransform2 ( const cv::Mat & srcDepthMat, cv::Mat & destDepthMat, int maxDepth )
{
	assert ( srcDepthMat.size() == destDepthMat.size() );
	assert ( srcDepthMat.type() == destDepthMat.type() );
	assert ( srcDepthMat.type() == CV_16UC1 );

	if ( srcDepthMat.empty() || destDepthMat.empty() ) {
		return false;
	}

	int depthWidth = srcDepthMat.size().width, depthHeight = srcDepthMat.size().height;
	int srcRowStep = srcDepthMat.step, destRowStep = destDepthMat.step;

	const uchar * srcRowPtr = NULL;
	const XnDepthPixel * srcDataPtr = NULL;
	uchar * destRowPtr = NULL;
	XnDepthPixel * destDataPtr = NULL;

	// 
	// Create Distance Map
	// 
	
	int adj[4];
	int * distRowPtr = NULL, * distDataPtr = NULL;
	int * const distMap = new int[depthHeight * depthWidth];
	
	memset ( distMap, 0, depthHeight * depthWidth * sizeof (int) );

	// 
	// First Scan
	// 
	
	srcRowPtr = srcDepthMat.data + srcRowStep;
	distRowPtr = distMap + depthWidth;

	for ( int y = 1; y < depthHeight - 1; ++y, srcRowPtr += srcRowStep, distRowPtr += depthWidth ) 
	{
		srcDataPtr = (const XnDepthPixel *)(srcRowPtr) + 1;
		distDataPtr = distRowPtr + 1;

		for ( int x = 1; x < depthWidth - 1; ++x, ++srcDataPtr, ++distDataPtr ) 
		{
			if ( *srcDataPtr && *srcDataPtr <= maxDepth ) {
				*distDataPtr = 0;
			}
			else if ( *srcDataPtr == 0 ) 
			{
				adj[0] = *(distDataPtr - 1) + 1;
				adj[1] = *(distDataPtr - depthWidth - 1) + 2;
				adj[2] = *(distDataPtr - depthWidth) + 1;
				adj[3] = *(distDataPtr - depthWidth + 1) + 2;
			
				*distDataPtr = adj[0];
				for ( int k = 1; k < 4; ++k ) {
					if ( adj[k] < *distDataPtr ) { *distDataPtr = adj[k]; }
				}
			}
		}
	}

	// 
	// Second Scan
	// 

	distRowPtr = distMap + ( depthHeight - 1 ) * depthWidth - 1;
	distDataPtr = distRowPtr - 1;

	for ( int y = depthHeight - 2; y >= 1; --y, distRowPtr -= depthWidth ) 
	{
		distDataPtr = distRowPtr - 1;

		for ( int x = depthWidth - 2; x >= 1; --x, --distDataPtr ) 
		{			
			if ( *distDataPtr > 0 )
			{
				adj[0] = *(distDataPtr + 1) + 1;
				adj[1] = *(distDataPtr + depthWidth + 1) + 2;
				adj[2] = *(distDataPtr + depthWidth) + 1;
				adj[3] = *(distDataPtr + depthWidth - 1) + 2;

				int min = adj[0];
				for ( int k = 1; k < 4; ++k ) {
					if ( adj[k] < min ) { min = adj[k]; }
				}

				if ( *distDataPtr > min ) { *distDataPtr = min; }
			}
		}
	}

	// 
	// Copy Distance Transform to cv::Mat
	// 

	distRowPtr = distMap;
	distDataPtr = distRowPtr;

	int minDist = INT_MAX, maxDist = INT_MIN;
	for ( int i = 0; i < depthHeight; ++i, distRowPtr += depthWidth ) 
	{
		distDataPtr = distRowPtr;

		for ( int j = 0; j < depthWidth; ++j, ++distDataPtr ) {
			if ( *distDataPtr < minDist ) { minDist = *distDataPtr; }
			if ( *distDataPtr > maxDist ) { maxDist = *distDataPtr; }
		}
	}

	destRowPtr = destDepthMat.data;
	distRowPtr = distMap;

	double delta = maxDist - minDist;

	for ( int i = 0; i < depthHeight; ++i, distRowPtr += depthWidth, destRowPtr += destRowStep ) 
	{
		distDataPtr = distRowPtr;
		destDataPtr = (XnDepthPixel *)(destRowPtr);

		for ( int j = 0; j < depthWidth; ++j, ++distDataPtr, ++destDataPtr ) {
			*destDataPtr = (double)((*distDataPtr) - minDist) / (delta + 1.0f) * 65535.0f;
		}
	}

	delete [] distMap;

	return true;
}

bool KinectHoleFiller::FmmInpaintHoleFilling(const cv::Mat & srcDepthMat, cv::Mat & destDepthMat)
{
	// TODO:
	return false;
}