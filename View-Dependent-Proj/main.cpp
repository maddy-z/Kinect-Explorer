#include <cstdio>
#include <cstdlib>
#include <climits>
#include <cmath>

#include <iostream>

#include <Windows.h>

#include <XnOpenNI.h>
#include <XnCppWrapper.h>
#include <XnEnumerationErrors.h>

#include <opencv2\opencv.hpp>

#include <AntTweakBar.h>

#include "..\Kinect Explorer\Sources\GlobalUtility.h"
#include "..\Kinect Explorer\Sources\OpenCvUtility.h"
#include "..\Kinect Explorer\Sources\OpenNiUtility.h"
#include "..\Kinect Explorer\Sources\SimpleHeadTracking.h"

#include "..\Simple3DScene\Pos.h"
#include "..\Simple3DScene\Camera.h"
#include "..\Simple3DScene\Matrix.h"
#include "..\Simple3DScene\Vec.h"
#include "..\Simple3DScene\Quaternion.h"

#include "qimage.h"

#include "AvgHeadCoor.h"
#include "ProjScene.h"

// -------------------
//  MACRO DEFINITIONS
// -------------------

// 
// Head Tracking Related MACRO DEFINITIONS
// 

#define		CONFIG_XML_PATH			"Config\\OpenNiProductConfig.xml"
#define		EXTR_MATRIX_PATH		"Config\\extr.txt"

#define		IMAGE_WIN_NAME			"Color Viewer"
#define		DEPTH_WIN_NAME			"Depth Viewer"

#define		INVALID_KEY_VALUE		-1
#define		ESC_KEY_VALUE			27

#define		HANDLING_DEPTH_DATA
#define		HANDLING_IMAGE_DATA
#undef		HANDLING_IMAGE_DATA

#define		USE_ANT_TWEAK_BAR
#undef		USE_ANT_TWEAK_BAR

// 
// FORWARD DECLARATION
// 

void IdleProcessOfHeadTracking ( void );

// 
// Global Variables
// 

// 
// Kinect Related Variables
// 

xn::Context				g_Context;

xn::DepthGenerator		g_DepthGen;
xn::ImageGenerator		g_ImageGen;
xn::DepthMetaData		g_DepthMD;
xn::ImageMetaData		g_ImageMD;

double					g_fXToZ;
double					g_fYToZ;

// Projector Information
const int				g_ProjectorWidth  = 1024;
const int				g_ProjectorHeight = 768;

// 
// Head Tracking Related Variables
// 

cv::Mat					g_DepthImgShow;
cv::Mat					g_DepthImgMat;
cv::Mat					g_ColorImgMat;

// QImage				g_DepthImgBuff;

// cv::Mat depthImgShow ( g_DepthMD.YRes(), g_DepthMD.XRes(), CV_8UC3 );
// cv::Mat colorImgMat  ( g_ImageMD.YRes(), g_ImageMD.XRes(), CV_8UC3 );
// cv::Mat depthImgMat  ( g_DepthMD.YRes(), g_DepthMD.XRes(), CV_16UC1 );

// QImage depthImgBuff	( g_DepthMD.XRes(), g_DepthMD.YRes(), QImage::Format_RGB32 );

bool					g_FlipColor	= true;										// Flag for Fliping Color

int						g_CtlWndKey;
int						g_HeadRowIndex;
int						g_HeadColIndex;

cv::Mat					g_ExtrMat ( 4, 4, CV_32FC1 );							// Camera Extrinsic Matrix

AvgHeadCoor				g_AvgHeadCoor ( 7, 100 );

DWORD					g_StartTickCount;
DWORD					g_CurrTickCount;

long					g_HeadTrackingFrameCount;

// 
// FORWARD DECLARATION
// 

// Head Tracking Related
bool ReadExtrParaFile	 (	const char * extrFile, cv::Mat & extrMat );
void CalcWorldCoorOfHead (	XnPoint3D & headWorldCoor, 
							const XnPoint3D & headKinectCoor,
							const cv::Mat & extrMatrix );

void PrintMatrix		 (	const cv::Mat & matrix );

// ---------------
//  Main Function
// ---------------

int main ( int argc, char ** argv )
{
	// Get Screen Parameters
	const int screenX = GetSystemMetrics ( SM_CXSCREEN );
	const int screenY = GetSystemMetrics ( SM_CYSCREEN );

	// std::cout << "screenX = " << screenX << std::endl;
	// std::cout << "screenY = " << screenY << std::endl;

	// 
	// Initializing OpenNI Settings
	// 
	
	XnStatus nRetVal = XN_STATUS_OK;
	xn::ScriptNode scriptNode;
	xn::EnumerationErrors errors;
	
	// 
	// Initializing OpenNI Context Object
	// 
	
	nRetVal = g_Context.InitFromXmlFile ( CONFIG_XML_PATH, scriptNode, &errors );
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
		printf ( "Open FAILED:\n%s\n", xnGetStatusString(nRetVal) );	
		
		system ( "pause" );
		return ( nRetVal );
	}

	// 
	// Handle Image & Depth Generator Node
	// 

	bool depthFlag = true;
	bool colorFlag = true;

	nRetVal = g_Context.FindExistingNode ( XN_NODE_TYPE_DEPTH, g_DepthGen );
	if ( nRetVal != XN_STATUS_OK ) {
		depthFlag = false;
		printf ( "No Depth Node Exists!\n" );
	}	
	nRetVal = g_Context.FindExistingNode ( XN_NODE_TYPE_IMAGE, g_ImageGen );
	if ( nRetVal != XN_STATUS_OK ) {
		colorFlag = false;
		printf ( "No Image Node Exists!\n" );
	}

	// g_DepthGen.GetAlternativeViewPointCap().SetViewPoint ( g_ImageGen );
	
	if ( depthFlag ) {
		g_DepthGen.GetMetaData ( g_DepthMD );
		assert ( g_DepthMD.PixelFormat() == XN_PIXEL_FORMAT_GRAYSCALE_16_BIT );
	}
	if ( colorFlag ) {
		g_ImageGen.GetMetaData ( g_ImageMD );
		assert ( g_ImageMD.PixelFormat() == XN_PIXEL_FORMAT_RGB24 );
	}

	XnFieldOfView fov;
	g_DepthGen.GetFieldOfView ( fov );
	
	g_fXToZ = tan ( fov.fHFOV / 2.0f ) * 2.0f;
	g_fYToZ = tan ( fov.fVFOV / 2.0f ) * 2.0f;

	// 
	// Create OpenCV Showing Window
	// 

	ReadExtrParaFile ( EXTR_MATRIX_PATH, g_ExtrMat );
	PrintMatrix	( g_ExtrMat );
	
	g_DepthImgShow = cv::Mat ( ( g_DepthMD.YRes() + 1 ) / 2, ( g_DepthMD.XRes() + 1 ) / 2, CV_8UC1  );
	g_DepthImgMat  = cv::Mat ( ( g_DepthMD.YRes() + 1 ) / 2, ( g_DepthMD.XRes() + 1 ) / 2, CV_16UC1 );
	g_ColorImgMat  = cv::Mat ( ( g_ImageMD.YRes() ), g_ImageMD.XRes(), CV_8UC3 );

	// g_DepthImgBuff = QImage  ( g_DepthMD.XRes(), g_DepthMD.YRes(), QImage::Format_RGB32 );

	// ------------------------------------------------------------------
	// ------------------------------------------------------------------

	// 
	// Initializing Glut & 3D Scene
	// 

	// 
	// GLUT Initialization
	// 
	
	glutInit				( &argc, argv );
	glutInitDisplayMode		( GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH );
	glutInitWindowPosition	( screenX, 0 );
	glutInitWindowSize		( g_ProjectorWidth, g_ProjectorHeight );

	glutCreateWindow		( "Simple 3D Scene" );
	glutCreateMenu			( NULL );

	// GL & GLU Initialization
	GlInit ();
	SceneInit ();

#ifdef USE_ANT_TWEAK_BAR

	// Tw Initialization
	TwInit ( TW_OPENGL, NULL );                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          

#endif

	glutMouseFunc			( MouseFunc );														// - Directly redirect GLUT mouse button events to AntTweakBar
	glutMotionFunc			( MouseMotion );													// - Directly redirect GLUT mouse motion events to AntTweakBar
	glutKeyboardFunc		( Keyboard );														// - Directly redirect GLUT key events to AntTweakBar
	
	glutDisplayFunc			( Display );
	glutReshapeFunc			( Reshape );
	glutIdleFunc			( IdleProcessOfHeadTracking );

#ifdef USE_ANT_TWEAK_BAR

	glutSpecialFunc			( ( GLUTspecialfun ) ( TwEventSpecialGLUT ) );						// - Directly redirect GLUT special key events to AntTweakBar
	glutPassiveMotionFunc	( ( GLUTmousemotionfun ) ( TwEventMouseMotionGLUT ) );				// - Directly redirect GLUT mouse "passive" motion events to AntTweakBar ( same as MouseMotion )

	// - Send 'glutGetModifers' function pointer to AntTweakBar;
	//   required because the GLUT key event functions do not report key modifiers states.
	TwGLUTModifiersFunc ( glutGetModifiers );

	TwBar * bar = TwNewBar ( "TweakBar" );
	TwDefine ( " GLOBAL help='This example shows a Simple 3D scene.' " ); 

	TwAddVarRW ( bar, "Camera Rotation", TW_TYPE_QUAT4F, &g_CameraRotQuat, " label='Camera Rotation' opened=true help='Change the Camera orientation.' " );
	TwAddVarRW ( bar, "Scene Rotation", TW_TYPE_QUAT4F, &g_SceneRotQuat, " label='Scene Rotation' opened=true help='Change the Scene orientation.' " );

	TwAddVarRW ( bar, "Object Zoom", TW_TYPE_FLOAT, &g_ObjSize, " min=1.00 max=7.00 step=0.10 keyIncr=z keyDecr=Z help='Scale the object (1=original size).' ");

	TwAddVarRW ( bar, "Obj Ambient", TW_TYPE_COLOR3F, &g_ObjMatAmbient, " group='Object Material' " );
	TwAddVarRW ( bar, "Obj Diffuse", TW_TYPE_COLOR3F, &g_ObjMatDiffuse, " group='Object Material' " );
	TwAddVarRW ( bar, "Obj Specular", TW_TYPE_COLOR3F, &g_ObjMatSpecular, " group='Object Material' " );
	TwAddVarRW ( bar, "Obj Shininess", TW_TYPE_FLOAT, &g_ObjMatShininess, " min=0.00 max=128.00 step=1.00 group='Object Material' " );

	TwAddVarRW ( bar, "Box Ambient", TW_TYPE_COLOR3F, &g_BoxMatAmbient, " group='Box Material' " );
	TwAddVarRW ( bar, "Box Diffuse", TW_TYPE_COLOR3F, &g_BoxMatDiffuse, " group='Box Material' " );
	TwAddVarRW ( bar, "Box Specular", TW_TYPE_COLOR3F, &g_BoxMatSpecular, " group='Box Material' " );
	TwAddVarRW ( bar, "Box Shininess", TW_TYPE_FLOAT, &g_BoxMatShininess, " min=0.00 max=128.00 step=1.00 group='Box Material' " );

#endif 

	// Recording Start Tick Count
	g_StartTickCount = GetTickCount ();
	g_HeadTrackingFrameCount = 0;

	// Start to Loop
	glutMainLoop ();

#ifdef USE_ANT_TWEAK_BAR

	// Tw Terminate
	TwTerminate ();	

#endif

	g_Context.Release ();									// Release OpenNI

	system ( "pause" );
	exit ( EXIT_SUCCESS );

}

void IdleProcessOfHeadTracking ( void )
{
	// 
	// Try to Get New Frame From Kinect
	// 

	XnStatus nRetVal = g_Context.WaitAnyUpdateAll ();

#ifdef HANDLING_IMAGE_DATA

	// Get Image Data First
	g_ImageGen.GetMetaData ( g_ImageMD );

	// assert ( g_DepthMD.FullXRes() == g_DepthMD.XRes() && g_DepthMD.FullYRes() == g_DepthMD.YRes() );
	// assert ( g_ImageMD.FullXRes() == g_ImageMD.XRes() && g_ImageMD.FullYRes() == g_ImageMD.YRes() );

	// To Handle Image Data
	GlobalUtility::CopyColorRawBufToCvMat8uc3 ( (const XnRGB24Pixel *)(g_ImageMD.Data()), g_ColorImgMat );
	
	if ( g_CtlWndKey == 's' || g_CtlWndKey == 'S' ) {												// Switch
		g_FlipColor = !g_FlipColor;
	}
	if ( g_FlipColor ) {
		cv::cvtColor ( g_ColorImgMat, g_ColorImgMat, CV_RGB2BGR );
	}

	cv::namedWindow ( IMAGE_WIN_NAME, CV_WINDOW_AUTOSIZE );
	cv::imshow ( IMAGE_WIN_NAME, g_ColorImgMat );

#endif

#ifdef HANDLING_DEPTH_DATA

	// Get Depth Data First
	g_DepthGen.GetMetaData ( g_DepthMD );

	XnPoint3D headImgCoor, headKinectCoor, headWorldCoor;
	
	headImgCoor.X = headImgCoor.Y = headImgCoor.Z = 0.0f;
	headKinectCoor.X = headKinectCoor.Y = headKinectCoor.Z = 0.0f;
	headWorldCoor.X = headWorldCoor.Y = headWorldCoor.Z = 0.0f;

	// 
	// Handle Depth Data
	// 
	
	if ( SimpleHeadTracking::SimpleDepthHandlingFunc2 ( g_DepthImgMat, g_DepthGen, headImgCoor ) ) 
	{
		GlobalUtility::ConvertDepthCvMat16uToGrayCvMat ( g_DepthImgMat, g_DepthImgShow );
		// GlobalUtility::ConvertDepthCvMat16uToGrayQImage ( g_DepthImgMat, g_DepthImgShow );
		
		for ( int i = 0; i < 25; ++i ) {
			if ( headImgCoor.Y + i < g_DepthImgShow.size().height )	{ g_DepthImgShow.at<uchar>( headImgCoor.Y + i, headImgCoor.X) = 0; }
			if ( headImgCoor.Y - i >= 0 )							{ g_DepthImgShow.at<uchar>( headImgCoor.Y - i, headImgCoor.X) = 0; }
			if ( headImgCoor.X + i < g_DepthImgShow.size().width )	{ g_DepthImgShow.at<uchar>( headImgCoor.Y, headImgCoor.X + i) = 0; }
			if ( headImgCoor.X - i >= 0 )							{ g_DepthImgShow.at<uchar>( headImgCoor.Y, headImgCoor.X - i) = 0; }
		}

		/*
		GlobalUtility::ConvertDepthCvMat16uToGrayQImage ( g_DepthImgMat, g_DepthImgBuff );
		for ( int i = 0; i < 25; ++i ) {
			if ( headImgCoor.Y + i < g_DepthImgShow.size().height )	{ g_DepthImgBuff.setPixel ( headImgCoor.X, headImgCoor.Y + i, blackColor ); }
			if ( headImgCoor.Y - i >= 0 )							{ g_DepthImgBuff.setPixel ( headImgCoor.X, headImgCoor.Y - i, blackColor ); }
			if ( headImgCoor.X + i < g_DepthImgShow.size().width )	{ g_DepthImgBuff.setPixel ( headImgCoor.X + i, headImgCoor.Y, blackColor ); }
			if ( headImgCoor.X - i >= 0 )							{ g_DepthImgBuff.setPixel ( headImgCoor.X - i, headImgCoor.Y, blackColor ); }
		}
		GlobalUtility::CopyQImageToCvMat8uc3 ( g_DepthImgBuff, g_DepthImgShow );
		*/

		// Get Average Head Image Coor in Kinect's Depth Image to avoid vibration 
		cv::Point3f tmpPt ( headImgCoor.X, headImgCoor.Y, headImgCoor.Z );
		g_AvgHeadCoor.InsertNewHeadCoor ( tmpPt );
		g_AvgHeadCoor.GetAvgHeadCoor ( tmpPt );

		headImgCoor.X = tmpPt.x;
		headImgCoor.Y = tmpPt.y;
		headImgCoor.Z = tmpPt.z;

		OpenNiUtility::ConvertProjectiveToRealWorld ( 1, (g_DepthMD.XRes() + 1 ) / 2, (g_DepthMD.YRes() + 1 ) / 2, g_fXToZ, g_fYToZ, &headImgCoor, &headKinectCoor );												
		// g_DepthGen.ConvertProjectiveToRealWorld ( 1, &headImgCoor, &headKinectCoor );

		// Calculate Head Coordinate of the Whole Scene	
		CalcWorldCoorOfHead ( headWorldCoor, headKinectCoor, g_ExtrMat );

		/*
		char headImgCoorStr[100];
		char headKinectCoorStr[100];
		char headWorldCoorStr[100];

		sprintf_s ( headImgCoorStr, "< %.6f, %.6f, %.6f >", headImgCoor.X, headImgCoor.Y, headImgCoor.Z );
		sprintf_s ( headKinectCoorStr, "< %.6f, %.6f, %.6f >", headKinectCoor.X, headKinectCoor.Y, headKinectCoor.Z );
		sprintf_s ( headWorldCoorStr, "< %.6f, %.6f, %.6f >", headWorldCoor.X, headWorldCoor.Y, headWorldCoor.Z );

		PrintMatrix ( g_ExtrMat );

		std::cout << "Head Image  Coor = " << std::string ( headImgCoorStr ) << std::endl;
		std::cout << "Head Kinect Coor = " << std::string ( headKinectCoorStr ) << std::endl;
		std::cout << "Head World  Coor = " << std::string ( headWorldCoorStr ) << std::endl;
		*/

		// 
		// Reset New Head World Coordinate when people move
		// 
	
		const float camPosX = headWorldCoor.X / 10.0f * ( g_BoxSizeY / g_BoxMetricSizeY ) * scaleFactor;
		const float camPosY = headWorldCoor.Y / 10.0f * ( g_BoxSizeY / g_BoxMetricSizeY ) * scaleFactor;
		const float camPosZ = ( headWorldCoor.Z / 10.0f /* + g_BoxMetricSizeZ */) * ( g_BoxSizeY / g_BoxMetricSizeY ) * scaleFactor;

		g_CameraPos.Set ( camPosX, camPosY, camPosZ );
		g_Camera.SetCameraPos ( g_CameraPos );
	
		/*
		std::cout	<< "In IdleProcessOfHeadTracking"	<< std::endl;
		std::cout	<< "camPosX = "						<< camPosX	<< std::endl
					<< "camPosY = "						<< camPosY	<< std::endl 
					<< "camPosZ = "						<< camPosZ	<< std::endl;
		std::cout	<< "Out IdleProcessOfHeadTracking"	<< std::endl;
		*/
	}
	else {
		GlobalUtility::ConvertDepthCvMat16uToGrayCvMat ( g_DepthImgMat, g_DepthImgShow );
		// GlobalUtility::ConvertDepthCvMat16uToGrayQImage ( g_DepthImgMat, g_DepthImgShow );
	}

	// GlobalUtility::CopyDepthRawBufToCvMat16u			( g_DepthMD.Data(), depthImgMat );
	// GlobalUtility::ConvertDepthCvMat16uToYellowCvMat ( depthImgMat, depthImgShow );
	// GlobalUtility::ConvertDepthCvMat16uToGrayCvMat	( depthImgMat, depthImgShow );

	cv::namedWindow ( DEPTH_WIN_NAME, CV_WINDOW_AUTOSIZE );
	cv::imshow ( DEPTH_WIN_NAME, g_DepthImgShow );

#endif

	g_HeadTrackingFrameCount++;
	g_CurrTickCount = GetTickCount();
	std::cout	<< "FPS = " 
				<< 1000 / ( ( double )( g_CurrTickCount - g_StartTickCount ) / ( double )( g_HeadTrackingFrameCount ) ) 
				<< std::endl;

	g_CtlWndKey = cvWaitKey ( 5 );
	
	glutPostRedisplay ();								// Refresh Glut Window and Scene

}

// 
// Utility Function
// 

void CalcWorldCoorOfHead (	XnPoint3D & headWorldCoor, 
							const XnPoint3D & headKinectCoor,
							const cv::Mat & extrMatrix )
{
	assert ( extrMatrix.rows == 4 && extrMatrix.cols == 4 );

	cv::Mat vec = cv::Mat::zeros(4, 1, CV_32FC1);
	vec.at<float>(0, 0) = headKinectCoor.X;
	vec.at<float>(1, 0) = headKinectCoor.Y * ( -1.0f );						// Need to Pay Attention to Different Coordinate System
	vec.at<float>(2, 0) = headKinectCoor.Z;
	vec.at<float>(3, 0) = 1.0f;

	cv::Mat tmpWorldCoor = extrMatrix.inv ();

	assert ( tmpWorldCoor.type() == CV_32FC1 );
	assert ( tmpWorldCoor.rows == 4 && tmpWorldCoor.cols == 4 );
	
	float tmp[3];
	for ( int i = 0; i < 3; ++i ) {
		tmp[i] = tmpWorldCoor.at<float>(i, 0) * vec.at<float>(0, 0) + 
			tmpWorldCoor.at<float>(i, 1) * vec.at<float>(1, 0) + 
			tmpWorldCoor.at<float>(i, 2) * vec.at<float>(2, 0) + 
			tmpWorldCoor.at<float>(i, 3) * vec.at<float>(3, 0);
	}

	headWorldCoor.X = tmp[0];
	headWorldCoor.Y = tmp[1];
	headWorldCoor.Z = tmp[2];

	return;
}

bool ReadExtrParaFile ( const char * extrFile, cv::Mat & extrMat )
{
	// Set Pointers of Camera / Projector Parameters
	// Copy File Names

	assert ( extrMat.rows == 4 && extrMat.cols == 4 );
	assert ( extrMat.type() == CV_32FC1 );

	FILE * fp;															// Read parameters
	fopen_s ( &fp, extrFile, "r" );
	if ( fp == NULL ) { 
		return false;
	}

	float para;

	for ( int x = 0; x < 3; ++x )
	for ( int y = 0; y < 4; ++y ) {
		fscanf_s ( fp, "%f\t", &para );
		fscanf_s ( fp, "\n", &para );

		extrMat.at<float>(x, y) = para;
	}
	fclose ( fp );

	extrMat.at<float>(3, 0) = 0.0f;
	extrMat.at<float>(3, 1) = 0.0f;
	extrMat.at<float>(3, 2) = 0.0f;
	extrMat.at<float>(3, 3) = 1.0f;

	return true;
}

void PrintMatrix ( const cv::Mat & matrix )
{
	for ( int y = 0; y < matrix.size().height; ++y ) {
		for ( int x = 0; x < matrix.size().width; ++x ) {
			printf ( "%.6f\t", matrix.at<float>(y, x) ); 
		}
		printf ( "\n" );
	}

	printf ( "\n" );
	return;
}