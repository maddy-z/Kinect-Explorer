#include <cstdio>
#include <cstdlib>
#include <cfloat>
#include <iostream>

#include <vector>

#include <OpenCV_2.3.1\opencv2\opencv.hpp>

#include "ExtrCalibrator.h"
#include "ARTagHelper.h"

// 
// Constructor && Destructor
// 

ExtrCalibrator::
ExtrCalibrator ( int MarkerNum )
{
	m_MarkerNum = MarkerNum;

	m_Intr.create  ( 3, 3, CV_64F );
	m_Dist.create  ( 5, 1, CV_64F );
	m_Rot.create   ( 3, 1, CV_64F );
	m_Trans.create ( 3, 1, CV_64F );
	m_Extr.create  ( 3, 4, CV_64F );
}

ExtrCalibrator::
ExtrCalibrator ( int MarkerNum, 
				 const char * fnIntr, 
				 const char * fnDist )
{	
	m_MarkerNum = MarkerNum;

	m_Intr.create  ( 3, 3, CV_64F );
	m_Dist.create  ( 5, 1, CV_64F );
	m_Rot.create   ( 3, 1, CV_64F );
	m_Trans.create ( 3, 1, CV_64F );
	m_Extr.create  ( 3, 4, CV_64F );

	ReadIntrParaFile  ( fnIntr, fnDist );
}

ExtrCalibrator::
~ExtrCalibrator() 
{}

bool 
ExtrCalibrator::ReadIntrParaFile ( const char * intrFile, const char * distFile )
{
	// 
	// Set Pointers of Camera / Projector Parameter
	// Copy File Names
	// 

	cv::Mat intrinsic	= m_Intr;
	cv::Mat distortion	= m_Dist;

	FILE * fp;																// Read parameter
	double para;

	// 
	// Intrinsic parameter 
	// 

	if ( ( fp = fopen ( intrFile, "r" ) ) == NULL ) { 
		return false; 
	}
	for ( int y = 0; y < intrinsic.size().height; ++y )
	for ( int x = 0; x < intrinsic.size().width;  ++x )
	{
		fscanf ( fp, "%lf", &para );
		intrinsic.at<double>(y, x) = para;
	}
	fclose ( fp );

	// 
	// Distortion parameter
	// 

	if ( ( fp = fopen ( distFile, "r" ) ) == NULL )	{
		return false;
	}
	for ( int x = 0; x < distortion.size().height; ++x )
	{
		fscanf ( fp, "%lf", &para ); 
		distortion.at<double>(x, 0) = para;
	}
	fclose ( fp );

	return true;
}

bool 
ExtrCalibrator::ReadExtrParaFile ( const char * rotFile, const char * transFile )
{
	// 
	// Set pointers of Camera / Projector parameters
	// Copy file names
	// 

	cv::Mat extrinsic = m_Extr;
	cv::Mat translation31 = m_Trans;
	cv::Mat rotation31 = m_Rot;

	FILE * fp;															// Read parameters
	double para;
	
	// Rotation
	if ( (fp = fopen ( rotFile, "r" ) ) == NULL )	 { 
		return false;
	}
	for ( int x = 0; x < rotation31.size().height; ++x )
	{		
		fscanf ( fp, "%lf", &para ); 
		rotation31.at<double>(x, 0) = para;
	}
	fclose( fp );

	// Rotation Vector to matrix
	cv::Mat rotation33 ( 3, 3, CV_64F );
	cv::Rodrigues ( rotation31, rotation33 );

	// Translation
	if ( (fp = fopen( transFile, "r" )) == NULL ) {
		return false;
	}
	for ( int x = 0; x < translation31.size().height; ++x )
	{
		fscanf( fp, "%lf", &para ); 	
		translation31.at<double>(x, 0) = para;
	}
	fclose( fp );

	// Extrinsic = Translation | Rotation
	for ( int x = 0; x < extrinsic.size().height; ++x )
	{
		extrinsic.at<double>(x, 0) = rotation33.at<double>(x, 0);
		extrinsic.at<double>(x, 1) = rotation33.at<double>(x, 1);
		extrinsic.at<double>(x, 2) = rotation33.at<double>(x, 2);

		extrinsic.at<double>(x, 3) = translation31.at<double>(x, 0);
	}

	return true;
}

double 
ExtrCalibrator::ComputeReprojectionErr ( double (* markerPos2d)[2], double (* markerPos3d)[3], int num )
{
	if ( markerPos2d == NULL || markerPos3d == NULL ) {
		return FLT_MIN;
	}

	std::vector<cv::Point3f> pos3d		( num );
	std::vector<cv::Point2f> pos2d		( num );
	std::vector<cv::Point2f> pos2dCalc	( num );

	for ( int i = 0; i < num; ++i )
	{
		pos3d[i].x = markerPos3d[i][0];
		pos3d[i].y = markerPos3d[i][1];
		pos3d[i].z = markerPos3d[i][2];

		pos2d[i].x = markerPos2d[i][0];
		pos2d[i].y = markerPos2d[i][1];
	}

	// 
	// Reproject Object Points
	// 

	cv::projectPoints ( pos3d, m_Rot, m_Trans, m_Intr, m_Dist, pos2dCalc );

	// std::cout << pos2dCalc.size() << " " << pos2dCalc.size() << std::endl;
	// std::cout << pos2d.size().width << " " << pos2d.size().height << std::endl;
	// assert ( pos2dCalc.type() == CV_64FC1 && pos2d.type() == CV_64FC1 );
	// assert ( pos2dCalc.size() == pos2d.size() );

	double err = cv::norm ( pos2d, pos2dCalc, CV_L2 );
	return sqrt ( err * err / num );
}

double 
ExtrCalibrator::ComputeReprojectionErr ( const ARTagHelper & artagHelper )
{
	int ValidMarkerNum = 0;
	for ( int i = 0; i < m_MarkerNum; ++i ) {
		if ( artagHelper.m_ValidFlagCam[i] ) { ++ValidMarkerNum; }
	}

	if ( ValidMarkerNum == 0 ) {
		std::cout << "Valid Marker Number = " << ValidMarkerNum << std::endl;
		return FLT_MIN;
	}

	double (* pos3d)[3] = ( double(*)[3] )( malloc ( ValidMarkerNum * 4 * 3 * sizeof ( double ) ) );
	double (* posCam2d)[2] = ( double(*)[2] )( malloc ( ValidMarkerNum * 4 * 2 * sizeof ( double ) ) );

	int j = 0;
	for ( int i = 0; i < m_MarkerNum; ++i )
	{
		if ( artagHelper.m_ValidFlagCam[i] )
		{
			for ( int k = 0; k < 4; ++k, ++j )
			{
				pos3d[j][0] = artagHelper.m_MarkerCornerPos3d[i*4+k][0];
				pos3d[j][1] = artagHelper.m_MarkerCornerPos3d[i*4+k][1];
				pos3d[j][2] = artagHelper.m_MarkerCornerPos3d[i*4+k][2];
				
				posCam2d[j][0] = artagHelper.m_MarkerCornerPosCam2d[i*4+k][0];
				posCam2d[j][1] = artagHelper.m_MarkerCornerPosCam2d[i*4+k][1];
			}
		}
	}

	double err = ComputeReprojectionErr ( posCam2d, pos3d, ValidMarkerNum * 4 );
	
	free ( pos3d );
	free ( posCam2d );

	return err;
}

void 
ExtrCalibrator::ExtrCalib ( const ARTagHelper & artagHelper )
{
	ExtrCalib ( artagHelper.m_MarkerCornerPosCam2d, artagHelper.m_MarkerCornerPos3d, artagHelper.m_ValidFlagCam );
}

void 
ExtrCalibrator::ExtrCalib ( double (* markerPos2d)[2], double (* markerPos3d)[3], bool * valid_flag )
{
	// 
	// Set Pointers of Camera / Projector Parameters
	// Copy File Name
	// 

	cv::Mat intrinsic	= m_Intr;
	cv::Mat distortion	= m_Dist;
	cv::Mat rotation	= m_Rot;
	cv::Mat translation = m_Trans;
	cv::Mat extrinsic	= m_Extr;

	// 
	// Copy 2D / 3D position data of markers
	// 

	int ValidMarkerNum = 0;
	for ( int i = 0; i < m_MarkerNum; ++i ) {
		if ( valid_flag[i] ) { ++ValidMarkerNum; }
	}

	if ( ValidMarkerNum == 0 ) {
		std::cout << "Valid Marker Number = " << ValidMarkerNum << std::endl;
		return;
	}

	cv::Mat pos3d ( ValidMarkerNum * 4, 3, CV_64FC1 );
	cv::Mat pos2d ( ValidMarkerNum * 4, 2, CV_64FC1 );

	int j = 0;
	
	for ( int i = 0; i < m_MarkerNum; ++i )
	{
		if ( valid_flag[i] )
		{
			for ( int k = 0; k < 4; ++k, ++j )
			{
				pos3d.at<double>(j, 0) = markerPos3d[i*4+k][0];
				pos3d.at<double>(j, 1) = markerPos3d[i*4+k][1];
				pos3d.at<double>(j, 2) = markerPos3d[i*4+k][2];
				
				pos2d.at<double>(j, 0) = markerPos2d[i*4+k][0];
				pos2d.at<double>(j, 1) = markerPos2d[i*4+k][1];
			}
		}
	}

	// 
	// Extrinsic Parameter Calibration using an OpenCV Function
	// 

	cv::solvePnP ( pos3d, pos2d, intrinsic, distortion, rotation, translation );

	// 
	// Extrinsic = Rotation | Translation
	// 

	cv::Mat rotation33 ( 3, 3, CV_64FC1 );
	cv::Rodrigues ( rotation, rotation33 );
	
	for ( int x = 0; x < 3; ++x )
	{
		extrinsic.at<double>(x, 0) = rotation33.at<double>(x, 0);
		extrinsic.at<double>(x, 1) = rotation33.at<double>(x, 1);
		extrinsic.at<double>(x, 2) = rotation33.at<double>(x, 2);

		extrinsic.at<double>(x, 3) = translation.at<double>(x, 0);
	}

	return;
}

const cv::Mat & 
ExtrCalibrator::GetMatrix ( int matType )
{
	switch (matType) 
	{
	case INTR:		return m_Intr;
	case DIST:		return m_Dist;
	case ROT:		return m_Rot;
	case TRANS:		return m_Trans;
	case EXTR:		return m_Extr;
	}

	return m_Intr;
}

void 
ExtrCalibrator::PrintMatrix ( const cv::Mat & matrix )
{
	for ( int y = 0; y < matrix.size().height; ++y ) 
	{
		for ( int x = 0; x < matrix.size().width; ++x ) {
			printf ( "%.6f\t", matrix.at<double>(y, x) ); 
		}
		printf ( "\n" );
	}

	printf ( "\n" );
	return;
}

void 
ExtrCalibrator::SaveMatrix ( const cv::Mat & mat, const char * fname )
{
	FILE * fp = fopen ( fname, "w" );
	if ( fp == NULL ) { return; }

	for ( int y = 0; y < mat.rows; ++y ) 
	{
		for ( int x = 0; x < mat.cols; ++x ) {
			fprintf ( fp, "%f\t", mat.at<double>(y, x) ); 
		}
		fprintf ( fp, "\n" );
	}

	fclose ( fp );
}

void 
ExtrCalibrator::SaveMatrix ( int matType, const char * fname )
{
	SaveMatrix ( GetMatrix ( matType ), fname );
}
