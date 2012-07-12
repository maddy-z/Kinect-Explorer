#include <cstdio>
#include <cstdlib>
#include <climits>
#include <cmath>

#include <iostream>

#include <XnOpenNI.h>
#include <XnCppWrapper.h>
#include <XnEnumerationErrors.h>

#include <opencv2\opencv.hpp>

#include "..\Kinect Explorer\Sources\GlobalUtility.h"
#include "..\Kinect Explorer\Sources\OpenCvUtility.h"
#include "..\Kinect Explorer\Sources\OpenNiUtility.h"
#include "..\Kinect Explorer\Sources\SimpleHeadTracking.h"

#include "qimage.h"

// 
// MACROS DEFINITIONS
// 

#define		CONFIG_XML_PATH			"Config\\OpenNiProductConfig.xml"

#define		IMAGE_WIN_NAME			"Color Viewer"
#define		DEPTH_WIN_NAME			"Depth Viewer"

#define		INVALID_KEY_VALUE		-1
#define		ESC_KEY_VALUE			27

#define		HANDLING_DEPTH_DATA

// 
// Global Constants
// 

xn::Context				g_Context;
xn::ScriptNode			g_ScriptNode;

xn::DepthGenerator		g_DepthGen;
xn::ImageGenerator		g_ImageGen;
xn::DepthMetaData		g_DepthMD;
xn::ImageMetaData		g_ImageMD;

// 
// Main Function
// 

int main ( int argc, char ** argv )
{
	// 
	// Initializing OpenNI Settings
	// 

	int ctlWndKey = -1;

	XnStatus nRetVal = XN_STATUS_OK;
	xn::EnumerationErrors errors;

	// 
	// Initialize Context Object
	// 
	
	nRetVal = g_Context.InitFromXmlFile ( CONFIG_XML_PATH, g_ScriptNode, &errors );
	if ( nRetVal == XN_STATUS_NO_NODE_PRESENT ) 
	{
		XnChar strError[1024];
		errors.ToString ( strError, 1024 );
		printf ( "XN_STATUS_NO_NODE_PRESENT:\n%s\n", strError );
	
		system ( "pause" );
		return (nRetVal);
	}
	else if ( nRetVal != XN_STATUS_OK )
	{
		printf("Open FAILED:\n%s\n", xnGetStatusString(nRetVal));	
		
		system("pause");
		return (nRetVal);
	}

	// 
	// Handle Image & Depth Generator Node
	// 
	
	nRetVal = g_Context.FindExistingNode ( XN_NODE_TYPE_DEPTH, g_DepthGen );
	if ( nRetVal != XN_STATUS_OK )
	{
		printf("No Depth Node Exists! Please Check your XML.\n");
		return ( nRetVal );
	}	
	nRetVal = g_Context.FindExistingNode ( XN_NODE_TYPE_IMAGE, g_ImageGen );
	if ( nRetVal != XN_STATUS_OK )
	{
		printf("No Image Node Exists! Please Check your XML.\n");
		return ( nRetVal );
	}

	// g_DepthGen.GetAlternativeViewPointCap().SetViewPoint( g_ImageGen );

	g_DepthGen.GetMetaData ( g_DepthMD );
	g_ImageGen.GetMetaData ( g_ImageMD );

	assert ( g_ImageMD.PixelFormat() == XN_PIXEL_FORMAT_RGB24 );
	assert ( g_DepthMD.PixelFormat() == XN_PIXEL_FORMAT_GRAYSCALE_16_BIT );

	// 
	// Create OpenCV Showing Window.
	// 

	cv::namedWindow ( IMAGE_WIN_NAME, CV_WINDOW_AUTOSIZE );

	// cv::Mat depthImgMat  ( g_DepthMD.YRes(), g_DepthMD.XRes(), CV_16UC1 );
	cv::Mat depthImgShow ( g_DepthMD.YRes(), g_DepthMD.XRes(), CV_8UC3  );
	cv::Mat colorImgMat  ( g_ImageMD.YRes(), g_ImageMD.XRes(), CV_8UC3 );

	QImage depthImgBuff ( g_DepthMD.XRes(), g_DepthMD.YRes(), QImage::Format_RGB32 );

	bool flipColor	= true;					// Flag for Fliping Color
	
	int headRowIndex;
	int headColIndex;

	// 
	// Start to Loop
	// 

	while ( ctlWndKey != ESC_KEY_VALUE ) 
	{
		// 
		// Try to Get New Frame From Kinect
		// 
	
		nRetVal = g_Context.WaitAnyUpdateAll ();

		g_DepthGen.GetMetaData ( g_DepthMD );
		g_ImageGen.GetMetaData ( g_ImageMD );

		// assert ( g_DepthMD.FullXRes() == g_DepthMD.XRes() && g_DepthMD.FullYRes() == g_DepthMD.YRes() );
		// assert ( g_ImageMD.FullXRes() == g_ImageMD.XRes() && g_ImageMD.FullYRes() == g_ImageMD.YRes() );

		// To Handle Image Data
		GlobalUtility::CopyColorRawBufToCvMat8uc3 ( (const XnRGB24Pixel *)(g_ImageMD.Data()), colorImgMat );
		
		if ( ctlWndKey == 's' || ctlWndKey == 'S' ) {			// Switch
			flipColor = !flipColor;
		}

		if ( flipColor ) { 
			cv::cvtColor ( colorImgMat, colorImgMat, CV_RGB2BGR );
		}

		cv::imshow ( IMAGE_WIN_NAME, colorImgMat  );

#ifdef HANDLING_DEPTH_DATA

		// To Handle Depth Data
		if ( SimpleHeadTracking::SimpleDepthHandlingFunc ( depthImgBuff, g_DepthGen ) == true ) {
			GlobalUtility::CopyQImageToCvMat8uc3 ( depthImgBuff, depthImgShow );
		}

		cv::namedWindow ( DEPTH_WIN_NAME, CV_WINDOW_AUTOSIZE );
		cv::imshow ( DEPTH_WIN_NAME, depthImgShow );

		// GlobalUtility::CopyDepthRawBufToCvMat16u  ( g_DepthMD.Data(), depthImgMat );
		// GlobalUtility::ConvertDepthCvMat16uToYellowCvMat ( depthImgMat, depthImgShow );
		// GlobalUtility::ConvertDepthCvMat16uToGrayCvMat ( depthImgMat, depthImgShow );

#endif

		ctlWndKey = cvWaitKey ( 15 );

	}

	g_Context.Release ();

	system ("pause");
	exit ( EXIT_SUCCESS );

}