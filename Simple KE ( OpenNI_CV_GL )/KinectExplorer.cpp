#include <cstdio>
#include <cstdlib>
#include <cmath>

#include <Windows.h>

#include <opencv2\opencv.hpp>

#include <XnOS.h>
#if ( XN_PLATFORM == XN_PLATFORM_MACOSX )
	#include <GLUT/glut.h>
#else
	#include <gl/glut.h>
#endif

#include	<XnCppWrapper.h>
// #include	<XnEnumerationErrors.h>
// #include	<XnPrdNode.h>
// #include	<XnPrdNodeInfo.h>

#include	"..\Kinect Explorer\Sources\GlobalUtility.h"
#include	"..\Kinect Explorer\Sources\OpenCvUtility.h"
#include	"..\Kinect Explorer\Sources\OpenNiUtility.h"

#define		CONFIG_XML_PATH			"OpenNiProductConfig.xml"

#define		IMAGE_WIN_NAME			"Color Viewer"
#define		DEPTH_WIN_NAME			"Depth Viewer"

#define		INVALID_KEY_VALUE		-1
#define		ESC_KEY_VALUE			27

// #define		HANDLING_IMAGE_DATA
// #define		HANDLING_DEPTH_DATA

// -----------------
//  Global Constant
// -----------------

xn::Context			g_Context;

xn::DepthGenerator	g_DepthGen;
xn::ImageGenerator	g_ImageGen;
xn::DepthMetaData	g_DepthMD;
xn::ImageMetaData	g_ImageMD;

cv::Mat		g_DepthImgShow;
cv::Mat		g_DepthImgMat;
cv::Mat		g_ColorImgMat;

// 
// Statistic Variables
// 

DWORD		g_StartTickCount;
DWORD		g_CurrTickCount;

long		g_HeadTrackingFrameCount;

// ---------------
//  Main Function 
// ---------------

int main ( int argc, char * argv[] )
{
	// 
	// Initialize OpenNI Settings
	// 

	XnStatus nRetVal = XN_STATUS_OK;
	xn::ScriptNode scriptNode;
	xn::EnumerationErrors errors;

	// 
	// Initialize Context Object
	// 

	nRetVal = g_Context.InitFromXmlFile ( CONFIG_XML_PATH, scriptNode, &errors );
	
	if ( nRetVal == XN_STATUS_NO_NODE_PRESENT ) {
		XnChar strError[1024];
		errors.ToString(strError, 1024);
		printf ( "XN_STATUS_NO_NODE_PRESENT:\n%s\n", strError );
		system ( "pause" );

		return ( nRetVal );
	}
	else if ( nRetVal != XN_STATUS_OK ) {
		printf ( "Open failed: %s\n", xnGetStatusString(nRetVal) );	
		system ( "pause" );

		return ( nRetVal );
	}

	// 
	// Handle Image & Depth Generator Node
	// 

	bool colorFlag = true;
	bool depthFlag = true;

	nRetVal = g_Context.FindExistingNode ( XN_NODE_TYPE_DEPTH, g_DepthGen );
	if ( nRetVal != XN_STATUS_OK ) {
		printf("No depth node exists!\n");
		depthFlag = false;
	}
	nRetVal = g_Context.FindExistingNode ( XN_NODE_TYPE_IMAGE, g_ImageGen );
	if ( nRetVal != XN_STATUS_OK ) {
		printf("No image node exists!\n");
		colorFlag = false;
	}

	// g_DepthGen.GetAlternativeViewPointCap().SetViewPoint( g_ImageGen );

	if ( depthFlag ) {
		g_DepthGen.GetMetaData ( g_DepthMD );
		assert ( g_DepthMD.PixelFormat() == XN_PIXEL_FORMAT_GRAYSCALE_16_BIT );
	}
	if ( colorFlag ) {
		g_ImageGen.GetMetaData ( g_ImageMD );
		assert ( g_ImageMD.PixelFormat() == XN_PIXEL_FORMAT_RGB24 );
	}

	g_DepthImgShow = cv::Mat ( g_DepthMD.YRes(), g_DepthMD.XRes(), CV_8UC1  );
	g_DepthImgMat  = cv::Mat ( g_DepthMD.YRes(), g_DepthMD.XRes(), CV_16UC1 );
	g_ColorImgMat  = cv::Mat ( g_ImageMD.YRes(), g_ImageMD.XRes(), CV_8UC3  );
	
	// 
	// Start to Loop
	// 

	bool flipColor = true;
	int ctlWndKey = -1;

	g_StartTickCount = GetTickCount();
	g_HeadTrackingFrameCount = 0;
	
	while ( ctlWndKey != ESC_KEY_VALUE ) 
	{
		nRetVal = g_Context.WaitOneUpdateAll ( g_DepthGen );
		// nRetVal = g_Context.WaitAnyUpdateAll();

#ifdef HANDLING_IMAGE_DATA

		if ( colorFlag ) 
		{
			g_ImageGen.GetMetaData ( g_ImageMD );

			assert ( g_ImageMD.FullXRes() == g_ImageMD.XRes() );
			assert ( g_ImageMD.FullYRes() == g_ImageMD.YRes() );

			GlobalUtility::CopyColorRawBufToCvMat8uc3 ( (const XnRGB24Pixel *)(g_ImageMD.Data()), g_ColorImgMat );
	
			if ( ctlWndKey == 's' || ctlWndKey == 'S' ) {												// Switch
				flipColor = !flipColor;
			}
			if ( flipColor ) {
				cv::cvtColor ( g_ColorImgMat, g_ColorImgMat, CV_RGB2BGR );
			}

			cv::namedWindow ( IMAGE_WIN_NAME, CV_WINDOW_AUTOSIZE );
			cv::imshow ( IMAGE_WIN_NAME, g_ColorImgMat );
		}

#endif

#ifdef HANDLING_DEPTH_DATA

		if ( depthFlag ) 
		{
			g_DepthGen.GetMetaData(g_DepthMD);
		
			// assert ( g_DepthMD.FullXRes() == g_DepthMD.XRes() );
			// assert ( g_DepthMD.FullYRes() == g_DepthMD.YRes() );

			GlobalUtility::CopyDepthRawBufToCvMat16u ( (const XnDepthPixel *)(g_DepthMD.Data()), g_DepthImgMat );
			GlobalUtility::ConvertDepthCvMat16uToGrayCvMat ( g_DepthImgMat, g_DepthImgShow );

			/*
			cv::putText(colorImgMat, 
						GlobalUtility::DoubleToString(g_ImageMD.FPS()) + " FPS", 
						cv::Point(10, 450), 
						cv::FONT_ITALIC, 
						0.7, 
						cv::Scalar(255, 255, 255, 0),
						2,
						8,
						false);
						*/

			cv::namedWindow ( DEPTH_WIN_NAME, CV_WINDOW_AUTOSIZE );
			cv::imshow ( DEPTH_WIN_NAME, g_DepthImgShow );
		}

#endif

		XnFieldOfView fov;
		g_DepthGen.GetFieldOfView( fov );

		std::cout	<< "HFov = " << fov.fHFOV << std::endl
					<< "VFov = " << fov.fVFOV << std::endl;

		ctlWndKey = cvWaitKey ( 5 );

		g_HeadTrackingFrameCount++;
		g_CurrTickCount = GetTickCount();
		std::cout	<< "FPS = " 
					<< 1000 / ( ( double )( g_CurrTickCount - g_StartTickCount ) / ( double )( g_HeadTrackingFrameCount ) ) 
					<< std::endl;
	}

	g_Context.Release ();

	exit ( EXIT_SUCCESS );
}

// --------------------------------------------------------------------------
// --------------------------------------------------------------------------

//#include <stdlib.h>
//#include <iostream>
//#include <string>
//#include <XnCppWrapper.h>
//#include "opencv/cv.h"
//#include "opencv/highgui.h"

//using namespace std;
//using namespace cv;
//
//void CheckOpenNIError( XnStatus eResult, string sStatus )
//{ 
// if( eResult != XN_STATUS_OK ) 
// cerr << sStatus << " Error: " << xnGetStatusString( eResult ) << endl;
//}
//
//int main( int argc, char** argv )
//{
// XnStatus eResult = XN_STATUS_OK;  
// // 1. initial val
// xn::DepthMetaData m_DepthMD;
// xn::ImageMetaData m_ImageMD;
// // for opencv Mat
// Mat  m_depth16u( 480,640,CV_16UC1);
// Mat  m_rgb8u( 480,640,CV_8UC3);
// Mat  m_DepthShow( 480,640,CV_8UC1);
// Mat  m_ImageShow( 480,640,CV_8UC3);
// cvNamedWindow("depth");
// cvNamedWindow("image");
// char key=0;
//
// // 2. initial context 
// xn::Context mContext; 
// eResult = mContext.Init(); 
// CheckOpenNIError( eResult, "initialize context" );  
//
 // // 3. create depth generator  
 // xn::DepthGenerator mDepthGenerator;  
 // eResult = mDepthGenerator.Create( mContext ); 
 // CheckOpenNIError( eResult, "Create depth generator" );  
 //
 // // 4. create image generator 
 // xn::ImageGenerator mImageGenerator;
 // eResult = mImageGenerator.Create( mContext ); 
 // CheckOpenNIError( eResult, "Create image generator" );
 //
 // // 5. set map mode  
 // XnMapOutputMode mapMode; 
 // mapMode.nXRes = 640;  
 // mapMode.nYRes = 480; 
 // mapMode.nFPS = 30; 
 // eResult = mDepthGenerator.SetMapOutputMode( mapMode );  
 // eResult = mImageGenerator.SetMapOutputMode( mapMode );  
 //
 // // 6. correct view port  
 // mDepthGenerator.GetAlternativeViewPointCap().SetViewPoint( mImageGenerator ); 
 //
 // // 7. tart generate data  
 // eResult = mContext.StartGeneratingAll();  
 //
 // // 8. read data  
 // eResult = mContext.WaitNoneUpdateAll();  
 // while( (key!=27) && !(eResult = mContext.WaitNoneUpdateAll( ))  ) 
 // {  
 // // 9a. get the depth map  
 // mDepthGenerator.GetMetaData(m_DepthMD);
 // memcpy(m_depth16u.data,m_DepthMD.Data(),640*480*2);
 // // 9b. get the image map  
 // mImageGenerator.GetMetaData(m_ImageMD);
 // memcpy(m_rgb8u.data,m_ImageMD.Data(),640*480*3);
 // m_depth16u.convertTo(m_DepthShow,CV_8U,255/2096.0);
 // cvtColor(m_rgb8u,m_ImageShow,CV_RGB2BGR);
 // imshow("depth", m_DepthShow);
 // imshow("image", m_ImageShow);
 //
 // key=cvWaitKey(20);
 // }
 //
 // // 10. stop  
 // mContext.StopGeneratingAll();
 // mContext.Shutdown();  
 // return 0;
 //}

//#include <XnCppWrapper.h>  
//#include "cv.h"  
//#include "highgui.h"  
// 
//#define SAMPLE_XML_PATH "F:/KFQ/downloads/openni_src/openni/Data/SamplesConfig.xml"  
//#define MAX_DEPTH 10000
//  
//float g_depthHist[MAX_DEPTH];  
//xn::ImageMetaData g_imd;  
//xn::DepthMetaData g_dmd;  
//xn::Context g_context;  
//xn::ImageGenerator g_image;   
//xn::DepthGenerator g_depth;  
//IplImage *g_pImgColor=0;  
//IplImage *g_pImgDepth=0;  
//  
////note that the channels' order is B G R in opencv  
//void UpdateColorImage(IplImage *pImg);  
//void UpdateDepthImage(IplImage *pImg);  
//int main(int argc, char* argv[])  
//{  
//    XnStatus nRetVal = XN_STATUS_OK;          
//      
//	nRetVal=g_context.InitFromXmlFile("NiProductConfig.xml");  
//    // g_context.OpenFileRecording("F:/xx.ONI");     
//    nRetVal=g_context.FindExistingNode(XN_NODE_TYPE_IMAGE,g_image);   
//    // CHECK_RC(nRetVal,"find image");  
//    nRetVal=g_context.FindExistingNode(XN_NODE_TYPE_DEPTH,g_depth);  
//    // CHECK_RC(nRetVal,"find depth");  
//  
//    g_image.GetMetaData(g_imd);  
//    g_depth.GetMetaData(g_dmd);  
//  
//    bool bRun=true;  
//    int nWidth;  
//    int nHeight;  
//      
//    while(bRun)  
//    {  
//        nRetVal=g_context.WaitAnyUpdateAll();  
//        if(nRetVal!=XN_STATUS_OK)  
//        {  
//            printf("failed update/n");  
//            continue;  
//        }  
//        g_image.GetMetaData(g_imd);  
//        g_depth.GetMetaData(g_dmd);  
//          
//        if(g_pImgColor==0)  
//        {  
//            nWidth=g_imd.XRes();  
//            nHeight=g_imd.YRes();  
//            g_pImgColor=cvCreateImage(cvSize(nWidth,nHeight),8,3);  
//            cvZero(g_pImgColor);  
//        }  
//        if(g_pImgDepth==0)  
//        {  
//            nWidth=g_dmd.XRes();  
//            nHeight=g_dmd.YRes();  
//            g_pImgDepth=cvCreateImage(cvSize(nWidth,nHeight),8,3);  
//            cvZero(g_pImgDepth);  
//        }  
//          
//        UpdateColorImage(g_pImgColor);  
//        UpdateDepthImage(g_pImgDepth);        
//          
//        cvShowImage("img",g_pImgColor);  
//        cvShowImage("depth",g_pImgDepth);  
//        char c=cvWaitKey(30);  
//        if(27==c)break;  
//    }  
//    if(g_pImgDepth)cvReleaseImage(&g_pImgDepth);  
//    if(g_pImgColor)cvReleaseImage(&g_pImgColor);  
//    g_context.Shutdown();  
//    return 0;  
//}  
//  
//void UpdateColorImage(IplImage *pImg)  
//{  
//    if(pImg->widthStep%4!=0)  
//    {  
//        const XnRGB24Pixel *pSrc=g_imd.RGB24Data();  
//          
//        for(int i=0;i<pImg->height;i++)  
//        {  
//            char *pDstRow=pImg->imageData+i*pImg->widthStep;  
//            for(int j=0;j<pImg->width;j++)  
//            {                 
//                *(pDstRow+3*j)=pSrc->nBlue;                
//                *(pDstRow+3*j+1)=pSrc->nGreen;                 
//                *(pDstRow+3*j+2)=pSrc->nRed;  
//                pSrc++;               
//            }  
//        }  
//    }  
//    else  
//    {  
//        xnOSMemCopy(pImg->imageData,g_imd.RGB24Data(),pImg->width*pImg->height*sizeof(XnUInt8)*3);  
//        cvCvtColor(pImg,pImg,CV_BGR2RGB);  
//    }     
//}  
//void UpdateDepthImage(IplImage *pImg)  
//{  
//    int numberOfPoints=0;  
//    const XnDepthPixel* pDepth = g_dmd.Data();  
//    xnOSMemSet(g_depthHist, 0, MAX_DEPTH*sizeof(float));  
//    unsigned long int total=0;  
//    int height=g_dmd.YRes();  
//    int width=g_dmd.XRes();  
//      
//    //Calculate the accumulative histogram  
//    for(int i=0;i<height;i++)  
//    {  
//        for(int j=0;j<width;j++)  
//        {  
//            unsigned int val=*pDepth;  
//            if(val!=0)  
//            {  
//                g_depthHist[val]++;  
//                  
//                numberOfPoints++;  
//            }  
//            pDepth++;  
//            total++;  
//        }  
//    }  
//    for (int nIndex=1; nIndex<MAX_DEPTH; nIndex++)  
//    {  
//        g_depthHist[nIndex] += g_depthHist[nIndex-1];  
//    }  
//    if (numberOfPoints)  
//    {  
//        for (int nIndex=1; nIndex<MAX_DEPTH; nIndex++)  
//        {  
//            g_depthHist[nIndex] = (unsigned int)(256 * (1.0f - (g_depthHist[nIndex] / numberOfPoints)));  
//        }  
//    }  
//      
//    const XnDepthPixel* pDepthRow = g_dmd.Data();  
//    for(int i=0;i<g_dmd.YRes();i++)  
//    {  
//        char *pDstRow=pImg->imageData+i*pImg->widthStep;  
//        for(int j=0;j<g_dmd.XRes();j++)  
//        {  
//            if(*pDepthRow!=0)  
//            {  
//                int val=g_depthHist[*pDepthRow];  
//                //B G R  
//                *(pDstRow+3*j)=0;  
//                *(pDstRow+3*j+1)=val;  
//                *(pDstRow+3*j+2)=val;  
//            }  
//              
//            pDepthRow++;  
//        }  
//    }  
//}  