#pragma once

#include <opencv2\opencv.hpp>

#include <string>

namespace GlobalUtility
{
	std::string DoubleToString(double value);
	std::string IntToString(int value);

	bool ConvertDepthToColor(const cv::Mat & depthMat, cv::Mat & colorMat, int maxDepth);
}