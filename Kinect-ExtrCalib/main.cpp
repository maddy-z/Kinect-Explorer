#include	<cstdio>
#include	<cstdlib>
#include	<iostream>

#include	<XnOpenNI.h>
#include	<XnCppWrapper.h>
#include	<XnEnumerationErrors.h>

#include	"..\Kinect Explorer\Sources\GlobalUtility.h"
#include	"..\Kinect Explorer\Sources\OpenCvUtility.h"
#include	"..\Kinect Explorer\Sources\OpenNiUtility.h"

#include	"ARTagHelper.h"
#include	"ExtrCalibrator.h"

// 
// MACROS DEFINITIONS
// 

#define		CONFIG_XML_PATH				"Config\\OpenNiProductConfig.xml"

#define		ARTAG_CONFIG_A3_FILE		"Config\\marker_cfg_a3.txt"
#define		ARTAG_CONFIG_FILE			"Config\\marker_cfg_a4.txt"
#define		ARTAG_POS_A3_FILE			"Config\\marker_pos_a3.txt"
#define		ARTAG_POS_FILE				"Config\\marker_pos_a4.txt"

#define		KINECT_INTR_FILE			"Config\\intr.txt"
#define		KINECT_DIST_FILE			"Config\\dist.txt"

#define		IMAGE_WIN_NAME				"Color Viewer"
#define		DEPTH_WIN_NAME				"Depth Viewer"

#define		INVALID_KEY_VALUE			-1
#define		ESC_KEY_VALUE				27

// #define		SHOW_DEPTH_WINDOW

// 
// FORWARD DECLARATION
// 

void ClickOnMouse ( int mouseEvent, int x, int y, int, void * );

// 
// Global Constants
// 

xn::Context				g_Context;
xn::ScriptNode			g_ScriptNode;

xn::DepthGenerator		g_DepthGen;
xn::ImageGenerator		g_ImageGen;
xn::DepthMetaData		g_DepthMD;
xn::ImageMetaData		g_ImageMD;

const int colorImgWidth		= 640;
const int colorImgHeight	= 480;

// 
// Main Function
// 

int main ( int argc, char ** argv )
{
	// 
	// Initializing Calibration Related
	// 

	// ARTagHelper artagHelper	( colorImgWidth, colorImgHeight, ARTAG_CONFIG_FILE,		ARTAG_POS_FILE );
	ARTagHelper artagHelper		( colorImgWidth, colorImgHeight, ARTAG_CONFIG_A3_FILE,	ARTAG_POS_A3_FILE );

	ExtrCalibrator extrCalibrator ( 6, KINECT_INTR_FILE, KINECT_DIST_FILE );
	// unsigned char * kinectImgBuf = new unsigned char[colorImgWidth * colorImgHeight * 3];

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
		return ( nRetVal );
	}
	else if ( nRetVal != XN_STATUS_OK )
	{
		printf ( "Open FAILED:\n%s\n", xnGetStatusString ( nRetVal ) );	
	
		system ( "pause" );
		return ( nRetVal );
	}

	// 
	// Handle the Depth Generator Node.
	// 
	
	nRetVal = g_Context.FindExistingNode ( XN_NODE_TYPE_DEPTH, g_DepthGen );
	if ( nRetVal != XN_STATUS_OK )
	{
		printf ( "No Depth Node Exists! Please Check your XML.\n" );
		return ( nRetVal );
	}
	
	// 
	// Handle the Image Generator node
	// 
	
	nRetVal = g_Context.FindExistingNode ( XN_NODE_TYPE_IMAGE, g_ImageGen );
	if ( nRetVal != XN_STATUS_OK )
	{
		printf ( "No Image Node Exists! Please Check your XML.\n" );
		return ( nRetVal );
	}

	// g_DepthGen.GetAlternativeViewPointCap().SetViewPoint( g_ImageGen );

	g_DepthGen.GetMetaData ( g_DepthMD );
	g_ImageGen.GetMetaData ( g_ImageMD );

	assert ( g_ImageMD.PixelFormat() == XN_PIXEL_FORMAT_RGB24 );
	assert ( g_DepthMD.PixelFormat() == XN_PIXEL_FORMAT_GRAYSCALE_16_BIT );

	// 
	// Create OpenCV Showing Window and Related Data Structures
	// 

	cv::namedWindow ( IMAGE_WIN_NAME, CV_WINDOW_AUTOSIZE );
	cv::namedWindow ( DEPTH_WIN_NAME, CV_WINDOW_AUTOSIZE );

	cv::Mat depthImgMat  ( g_DepthMD.YRes(), g_DepthMD.XRes(), CV_16UC1 );
	cv::Mat depthImgShow ( g_DepthMD.YRes(), g_DepthMD.XRes(), CV_8UC3  );
	cv::Mat colorImgMat  ( g_ImageMD.YRes(), g_ImageMD.XRes(), CV_8UC3 );

#define ARTAG_DEBUG

#ifdef ARTAG_DEBUG
	
	cv::setMouseCallback ( IMAGE_WIN_NAME, ClickOnMouse, 0 );

#endif

	bool flipColor = true;

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

		assert ( g_DepthMD.FullXRes() == g_DepthMD.XRes() && g_DepthMD.FullYRes() == g_DepthMD.YRes() );
		assert ( g_ImageMD.FullXRes() == g_ImageMD.XRes() && g_ImageMD.FullYRes() == g_ImageMD.YRes() );

		GlobalUtility::CopyColorRawBufToCvMat8uc3 ( (const XnRGB24Pixel *)(g_ImageMD.Data()), colorImgMat );

#ifdef SHOW_DEPTH_WINDOW

		GlobalUtility::CopyDepthRawBufToCvMat16u  ( (const XnDepthPixel *)(g_DepthMD.Data()), depthImgMat );
		// GlobalUtility::ConvertDepthCvMat16uToYellowCvMat ( depthImgMat, depthImgShow );
		GlobalUtility::ConvertDepthCvMat16uToGrayCvMat ( depthImgMat, depthImgShow );

		cv::imshow ( DEPTH_WIN_NAME, depthImgShow );

#endif

		ctlWndKey = cvWaitKey ( 15 );

		if ( ctlWndKey == 'f' || ctlWndKey == 'F' ) 
		{
			artagHelper.Clear();

			artagHelper.FindMarkerCorners ( (unsigned char *)(g_ImageMD.Data()) );
			artagHelper.PrintMarkerCornersPos2dInCam ();
			extrCalibrator.ExtrCalib ( artagHelper );

			std::cout << "\nKinect Extr Matrix:" << std::endl;
			extrCalibrator.PrintMatrix ( extrCalibrator.GetMatrix ( ExtrCalibrator::EXTR ) );

			std::cout	<< "Reprojection ERROR = " 
						<< extrCalibrator.ComputeReprojectionErr ( artagHelper ) << std::endl
						// << extrCalibrator.ComputeReprojectionErr ( artagHelper.m_MarkerCornerPosCam2d, artagHelper.m_MarkerCornerPos3d, 24 ) << std::endl
						<< "Valid Marker Number = " << artagHelper.GetValidMarkerNumber() << std::endl
						<< std::endl;
		}
		if ( ctlWndKey == 's' || ctlWndKey == 'S' )
		{
			flipColor = !flipColor;
		}

		if ( flipColor ) { 
			cv::cvtColor ( colorImgMat, colorImgMat, CV_RGB2BGR );
		}

		artagHelper.DrawMarkersInCameraImage ( colorImgMat );
		cv::imshow ( IMAGE_WIN_NAME, colorImgMat );
	}

	g_Context.Release ();
	
	system	( "pause" );
	exit	( EXIT_SUCCESS );

}

// --------------------------------------------------------
// --------------------------------------------------------

// 
// Util Functions
// 

void ClickOnMouse ( int mouseEvent, int x, int y, int, void * )
{
	if ( mouseEvent != CV_EVENT_LBUTTONDOWN ) {
		return;
	}

	printf("Mouse Clicked On Pt <x, y> = <%d, %d>\n", x, y);
}