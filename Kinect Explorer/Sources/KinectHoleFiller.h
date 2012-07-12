#ifndef _KINECT_HOLEFILLER_H_
#define _KINECT_HOLEFILLER_H_

#include <opencv2\opencv.hpp>

class KinectHoleFiller

{

public:
	
	static bool NearestNeighborhoodHoleFilling ( const cv::Mat & srcDepthMat, cv::Mat & destDepthMat );
	static bool NearestNeighborhoodHoleFilling2 ( const cv::Mat & srcDepthMat, cv::Mat & destDepthMat, int maxDepth );
	static bool NearestNeighborhoodHoleFilling3 ( const cv::Mat & srcDepthMat, cv::Mat & destDepthMat, int maxDepth );

	static bool DistanceTransform ( const cv::Mat & srcDepthMat, cv::Mat & destDepthMat, int maxDepth );
	static bool DistanceTransform2 ( const cv::Mat & srcDepthMat, cv::Mat & destDepthMat, int maxDepth );
	
	static bool FmmInpaintHoleFilling ( const cv::Mat & srcDepthMat, cv::Mat & destDepthMat );

};

#endif