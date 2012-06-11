#pragma once

#include <XnCppWrapper.h>

class OpenNIUtility
{
public:
	
	static double CalcAverageDepth(const xn::DepthMetaData & depthMD);

	static double CalcBiggestDepth(const xn::DepthMetaData & depthMD);
	static double CalcSmallestDepth(const xn::DepthMetaData & depthMD);
};