#ifndef _EXTR_CALIBRATOR_H_
#define _EXTR_CALIBRATOR_H_

#include <OpenCV_2.3.1\opencv2\opencv.hpp>

// 
// FORWARD DECLARATION
// 

class ARTagHelper;

// 
// Class used to Calibrate Extrinsic Parameters
// 

class ExtrCalibrator

{

public:
	
	enum
	{
		INTR, 
		DIST,
		ROT,
		TRANS,
		EXTR,
	};

	/*
	*	Constructor & Destructor
	*
	*	- MarkerNum:	Number of printed ARTag markers
	*	- (Others):		File name of Calibration parameters
	*/

	ExtrCalibrator (	int MarkerNum );
	ExtrCalibrator (	int MarkerNum, 
						const char * fnIntr, 
						const char * fnDist
						);
	
	virtual ~ExtrCalibrator ();

	/*
	*	Read parameters from external files
	*
	*	- Intrinsic File & Distortion File:		File Name
	*	- Type:									Identifier ( Projector or Camera )
	*/

	bool ReadIntrParaFile ( const char * intrFile,	const char * distFile	);
	bool ReadExtrParaFile ( const char * rotFile,	const char * transFile	);

	/*
	*	To Compute Reprojection Errors given 
	*
	*/

	double ComputeReprojectionErr ( double (* markerPos2d)[2], double (* markerPos3d)[3], int num );
	double ComputeReprojectionErr ( const ARTagHelper & artagHelper );

	/*
	*	Extrinsic Calibration
	*	
	*	 - Type: identifier ( Projector or Camera )
	*	 - markerPos2d: marker position in Camera / Projector image
	*	 - markerPos3d: 3d position of markers in world-coordinate
	*	 - valid_flag: 
	*		True: each marker is visible from camera/projector
	*		false: is not visible
	*
	*/

	void ExtrCalib ( const ARTagHelper & artagHelper );
	void ExtrCalib ( double (* markerPos2d)[2], double (* markerPos3d)[3], bool * valid_flag );

	/*
	*	Save / Print cv::Mat Matrix
	*/ 

	const cv::Mat & GetMatrix	( int matType );
	void SaveMatrix				( int matType, const char * fname );

	static void PrintMatrix		( const cv::Mat & matrix );
	static void SaveMatrix		( const cv::Mat & mat, const char * fname );

private:

	int m_MarkerNum;										// Number of AR Markers

	// 
	// Camera Parameter Matries
	// 

	cv::Mat m_Intr;
	cv::Mat m_Dist;
	cv::Mat m_Rot;
	cv::Mat m_Trans;
	cv::Mat m_Extr;

};

#endif
