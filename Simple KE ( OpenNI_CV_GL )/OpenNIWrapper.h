#pragma once

#include <stdio.h>
#include <stdlib.h>

#include <string>

#include <XnStatus.h>

class OpenNIWrapper
{

public:

	static bool CheckOpenNiErrorStatus(XnStatus eResult, std::string str) { return CheckOpenNiErrorStatus(eResult, str.c_str());}
	static bool CheckOpenNiErrorStatus(XnStatus eResult, const char * str) 
	{
		if( eResult != XN_STATUS_OK ) {
			printf("%s:\t%s\n", str, xnGetStatusString( eResult ));
			return false;
		}

		return true;
	}
};