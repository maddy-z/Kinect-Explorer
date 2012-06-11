#ifndef _OPENNI_UTILITY_H_
#define _OPENNI_UTILITY_H_

#include <XnCppWrapper.h>

namespace OpenNiUtility

{

double CalcAverageDepth(const XnDepthPixel * srcDepthData, unsigned int srcRow, unsigned int srcCol);

double CalcBiggestDepth(const XnDepthPixel * srcDepthData, unsigned int srcRow, unsigned int srcCol);
double CalcSmallestDepth(const XnDepthPixel * srcDepthData, unsigned int srcRow, unsigned int srcCol);

}

#endif