#ifndef _OPENNI_UTILITY_H_
#define _OPENNI_UTILITY_H_

#include <XnCppWrapper.h>

namespace OpenNiUtility

{

double	CalcAverageDepth  ( const XnDepthPixel * srcDepthData, unsigned int srcRow, unsigned int srcCol );

double	CalcBiggestDepth  ( const XnDepthPixel * srcDepthData, unsigned int srcRow, unsigned int srcCol );
double	CalcSmallestDepth ( const XnDepthPixel * srcDepthData, unsigned int srcRow, unsigned int srcCol );

bool	ConvertProjectiveToRealWorld (	const int count, 
										const int nXRes, const int nYRes, 
										const double fXToZ, const double fYToZ, 
										const XnPoint3D * proj, XnPoint3D * real );

bool	ConvertRealWorldToProjective (	const int count, 
										const int nXRes, const int nYRes, 
										const double fXToZ, const double fYToZ, 
										const XnPoint3D * real, XnPoint3D * proj );

}

#endif