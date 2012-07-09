#include	<cstdio>
#include	<cstdlib>
#include	<iostream>

#include	<windows.h>

#include	"ARTag\artag_rev2.h"
#include	"OpenCV_2.3.1\opencv2\opencv.hpp"
#include	"OpenGL\Glut\glut.h"

#include	"ARTagHelper.h"

// =============
//  ARTagHelper
// =============

ARTagHelper::ARTagHelper ( int cameraW, int cameraH, const char * fnConfig, const char * fnCornerPos ) :
	m_MarkerNum (0)
{
	assert ( fnConfig != NULL );
	assert ( fnCornerPos != NULL );

	m_CameraWidth = cameraW;
	m_CameraHeight = cameraH;

	// sprintf ( m_ConfigFile, "%s", fnConfig );
	// sprintf ( m_CornerPos, "%s", fnCornerPos );

	// Read Marker Information
	// Number of Markers / Positions of Marker Corners

	FILE * fp;
	char str[256];

	fp = fopen ( fnCornerPos, "r" );

	do { fgets ( str, 256, fp ); } while ( str[0] == '#' );
	sscanf ( str, "%d", &m_MarkerNum );
	assert ( m_MarkerNum > 0 );

	m_MarkerID_LUT = new int[m_MarkerNum];
	m_MarkerCornerPos3d = ( double(*)[3] )( malloc ( m_MarkerNum * 4 * 3 * sizeof ( double ) ) );
	m_MarkerCornerPosCam2d = ( double(*)[2] )( malloc ( m_MarkerNum * 4 * 2 * sizeof ( double ) ) );
	m_ValidFlagCam = new bool[m_MarkerNum];

	for ( int i = 0; i < m_MarkerNum; ++i ) {
		m_ValidFlagCam[i] = false;
	}

	for ( int id = 0; id < m_MarkerNum; ++id )
	for ( int i = 0; i < 4; ++i )
	{
		int h = id * 4 + i;
		do { fgets(str, 256, fp); } while ( str[0] == '#' );
		
		sscanf ( str, "%lf %lf %lf", &m_MarkerCornerPos3d[h][0], &m_MarkerCornerPos3d[h][1], &m_MarkerCornerPos3d[h][2] );
	}
	fclose ( fp );

	// 
	// Init ARTag
	// 

	init_artag ( m_CameraWidth, m_CameraHeight, 3 );
	
	int res = load_array_file ( (char *)(fnConfig) );
	if ( res == -1 )
	{
		printf( "%c is not found\n", fnConfig );
		return;
	}

	char id[8];
	for ( int i = 0; i < m_MarkerNum; ++i )
	{
		sprintf( id, "%d", i );
		m_MarkerID_LUT[i] = artag_associate_array( id );
	}

	return;
}

ARTagHelper::~ARTagHelper()
{
	if ( m_MarkerID_LUT ) { delete [] m_MarkerID_LUT; }

	if ( m_MarkerCornerPos3d ) { free ( m_MarkerCornerPos3d ); }
	if ( m_MarkerCornerPosCam2d ) { free ( m_MarkerCornerPosCam2d ); }

	if ( m_ValidFlagCam ) { delete [] m_ValidFlagCam; }
}

void ARTagHelper::FindMarkerCorners ( unsigned char * image )
{
	std::cout << "Start:\tvoid ARTagHelper::FindMarkerCorners ( unsigned char * )" << std::endl;
	std::cout << "Marker Number = " << m_MarkerNum << std::endl;

	artag_find_objects ( image, 1 );
	int nFound = 0;
	
	for ( int i = 0; i < m_MarkerNum; ++i )
	{
		if ( artag_is_object_found ( m_MarkerID_LUT[i] ) )
		{
			m_ValidFlagCam[i] = true;
			
			// All the corners of the marker should be visible from the camera.
			for ( int j = 0; j < 4; j++ )
			{
				float camX, camY;
				
				artag_project_point( m_MarkerID_LUT[i], 
									 m_MarkerCornerPos3d[i*4+j][0], 
									 m_MarkerCornerPos3d[i*4+j][1], 
									 m_MarkerCornerPos3d[i*4+j][2], 
									 &camX, 
									 &camY );
				
				m_MarkerCornerPosCam2d[i*4+j][0] = camX;
				m_MarkerCornerPosCam2d[i*4+j][1] = camY;
				
				if ( camX < 0.0 || camX >= m_CameraWidth || camY < 0.0 || camY >= m_CameraHeight ) {
					m_ValidFlagCam[i] = false;
				}
			}

			if ( m_ValidFlagCam[i] == true ) {
				nFound++;
			}
		}
	}

	std::cout << "Mark Found " << nFound << std::endl;
	std::cout << "End:\tvoid ARTagHelper::FindMarkerCorners ( unsigned char * )" << std::endl;

	return;
}

void ARTagHelper::PrintMarkerCornersPos2dInCam () const
{
	for ( int i = 0; i < m_MarkerNum; ++i ) 
	{
		if ( !m_ValidFlagCam[i] ) { continue; }
		
		for ( int j = 0; j < 4; ++j ) {
			printf("<%.3f, %.3f>\n", m_MarkerCornerPosCam2d[i*4+j][0], m_MarkerCornerPosCam2d[i*4+j][1]);
		}
	}
}

//void ARTagHelper::PrintMarkerCornersPos2dInProjector() const
//{
//	for ( int i = 0; i < m_MarkerNum; ++i ) 
//	{
//		if ( !m_ValidFlagPro[i] ) { continue; }
//		
//		for ( int j = 0; j < 4; ++j ) {
//			printf("<%.3f, %.3f>\n", m_MarkerCornerPosPro2d[i*4+j][0], m_MarkerCornerPosPro2d[i*4+j][1]);
//		}
//	}
//}

void ARTagHelper::PrintMarkerCornersPos3d() const
{
	for ( int i = 0; i < m_MarkerNum; ++i ) 
	for ( int j = 0; j < 4; ++j )
	{
		double x = m_MarkerCornerPos3d[i*4+j][0];
		double y = m_MarkerCornerPos3d[i*4+j][1];
		double z = m_MarkerCornerPos3d[i*4+j][2];
		
		printf("<%.3f, %.3f, %.3f>\n", x, y, z);
	}
}

//void ARTagHelper::GetMarkerCornerPos2dInProjector ( GrayCode * gc )
//{
//	for ( int i = 0; i < m_MarkerNum; ++i )
//	{
//		m_ValidFlagPro[i] = true;
//
//		for ( int j = 0; j < 4; ++j )
//		{
//			double x = m_MarkerCornerPosCam2d[i*4+j][0];
//			double y = m_MarkerCornerPosCam2d[i*4+j][1];
//
//			if ( gc->dblCode(gc->VERT, x, y) != 0.0 && 
//				 gc->dblCode(gc->HORI, x, y) != 0.0 &&
//				 gc->m_Mask[(int)(x) + (int)(y) * m_CameraWidth] )
//			{
//				// Gray Code can be obtained at the markers.
//				m_MarkerCornerPosPro2d[i*4+j][0] = gc->dblCode(gc->VERT, x, y);
//				m_MarkerCornerPosPro2d[i*4+j][1] = gc->dblCode(gc->HORI, x, y);
//			}
//			else { m_ValidFlagPro[i] = false; }
//		}
//	}
//}

void ARTagHelper::DrawMarkersInCameraImage ( float pixZoomX, float pixZoomY )
{
	for ( int i = 0; i < m_MarkerNum; ++i )
	{
		if ( m_ValidFlagCam[i] )
		{
			glColor3f(1.0f, 0.0f, 0.0f);

			glBegin(GL_LINE_LOOP);
				glVertex2f ( pixZoomX * m_MarkerCornerPosCam2d[i*4+0][0], pixZoomY * (m_CameraHeight - m_MarkerCornerPosCam2d[i*4+0][1]) );
				glVertex2f ( pixZoomX * m_MarkerCornerPosCam2d[i*4+1][0], pixZoomY * (m_CameraHeight - m_MarkerCornerPosCam2d[i*4+1][1]) );
				glVertex2f ( pixZoomX * m_MarkerCornerPosCam2d[i*4+2][0], pixZoomY * (m_CameraHeight - m_MarkerCornerPosCam2d[i*4+2][1]) );
				glVertex2f ( pixZoomX * m_MarkerCornerPosCam2d[i*4+3][0], pixZoomY * (m_CameraHeight - m_MarkerCornerPosCam2d[i*4+3][1]) );
			glEnd();
		}
	}

	return;
}

void 
ARTagHelper::DrawMarkersInCameraImage ( cv::Mat & img )
{
	for ( int i = 0; i < m_MarkerNum; ++i )
	{
		if ( m_ValidFlagCam[i] == true ) 
		{
			for ( int j = 0; j < 4; j++ )
			{
				float camX[2], camY[2];
				
				camX[0] = m_MarkerCornerPosCam2d[i*4+j][0];
				camY[0] = m_MarkerCornerPosCam2d[i*4+j][1];
				camX[1] = m_MarkerCornerPosCam2d[i*4+(j+1)%4][0];
				camY[1] = m_MarkerCornerPosCam2d[i*4+(j+1)%4][1];

				cv::line ( img, cv::Point2f ( camX[0], camY[0] ), cv::Point2f ( camX[1], camY[1] ), CV_RGB ( 255, 0, 0 ) );  
			}
		}
	}

	return;
}

void 
ARTagHelper::GetValidFlagArray ( const bool *& validArray, int & markerNum ) const 
{
	markerNum = m_MarkerNum;
	validArray = m_ValidFlagCam; 
}