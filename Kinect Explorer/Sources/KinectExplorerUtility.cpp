#include <string>
#include <sstream>

#include "KinectExplorerUtility.h"

namespace KinectExplorerUtility
{

std::string DoubleToString(double value)
{
	std::stringstream ssTmp;
	std::string str;

	ssTmp << value; ssTmp >> str;

	return str;
}

std::string IntToString(int value)
{
	std::stringstream ssTmp;
	std::string str;

	ssTmp << value; ssTmp >> str;

	return str;
}

}