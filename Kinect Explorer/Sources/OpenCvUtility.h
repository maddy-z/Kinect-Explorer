#ifndef _OPENCV_UTILITY_H_
#define _OPENCV_UTILITY_H_

#define		OPENNI_MAX_DEPTH		10000

#include <opencv2\opencv.hpp>

namespace OpenCvUtility

{

double CalcAverageDepth(const cv::Mat & srcDepthMat);

double CalcBiggestDepth(const cv::Mat & srcDepthMat);
double CalcSmallestDepth(const cv::Mat & srcDepthMat);

double CalcBiggestDepth8u(const cv::Mat & srcDepthMat);

// TODO:
// double CalcSmallestDepth(const cv::Mat & srcDepthMat);

void CalcMinimumOfDepthHistogram ( const cv::Mat & srcDepthMat, int & minIndex, int & minHistValue, int threshold = OPENNI_MAX_DEPTH );

bool DownSamplingCvMat16u ( const cv::Mat & srcMat, int factor, cv::Mat & destMat );

inline void HSV2RGB ( unsigned short H, uchar S, uchar V, uchar & R, uchar & G, uchar & B )
{
	// 
	// H[0 - 360], S[0 - 255], V[0 - 255]
	// 
	
	if (S == 0) {
		R = G = B = V;
		return;
	}

	float h = H / 60.f;
	float s = S / 255.f;
	int i = (int)(h);	

	float f, a, b, c;

	f = h - i;
	a = V * (1.f - s);
	b = V * (1.f - s * f);
	c = V * (1.f - s * (1 - f));

	assert (f >= 0.0f && a >= 0.0f && b >= 0.0f && c >= 0.0f);
	assert (i >= 0 && i < 6);

	switch (i) 

	{ 

	case 0: R = V; G = c; B = a; break;
	case 1: R = b; G = V; B = a; break;
	case 2: R = a; G = V; B = c; break;
	case 3: R = a; G = b; B = V; break;
	case 4: R = c; G = a; B = V; break;
	case 5: R = V; G = a; B = b; break;

	}

	return;
}

}

#endif