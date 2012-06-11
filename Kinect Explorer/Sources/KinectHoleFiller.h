#ifndef _KINECT_HOLEFILLER_H_
#define _KINECT_HOLEFILLER_H_

#include <opencv2\opencv.hpp>

class KinectHoleFiller
{

public:
	
	static bool NearestNeighborhoodHoleFilling(const cv::Mat & srcDepthMat, cv::Mat & destDepthMat);
	static bool FmmInpaintHoleFilling(const cv::Mat & srcDepthMat, cv::Mat & destDepthMat);

};

#endif