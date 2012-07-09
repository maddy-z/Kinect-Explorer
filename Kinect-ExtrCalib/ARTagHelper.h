#ifndef _ARTAG_HELPER_H_
#define _ARTAG_HELPER_H_

// #define		MARKER_CONFIG_FILE			"../../data/marker_cfg_RealPlay.txt"
// #define		MARKER_NUM_MAX				6

// 
// FORWARD DECLARATION
// 

class ExtrCalibrator;

// 
// Class -- ARTagHelper
// 

class ARTagHelper

{
	friend class ExtrCalibrator;
	friend int main ( int, char ** );

private:

	int m_CameraWidth;
	int m_CameraHeight;

	int * m_MarkerID_LUT;													// LUT for marker ID
	int m_MarkerNum;														// Number of markers

	// char m_ConfigFile[128];												// ARTag config file name
	// char m_CornerPos[128];												// name of a file which contains 3d position of corners of ARTag markers in world-coordinate

	double ( * m_MarkerCornerPos3d )[3];									// 3d marker corner position in world-coordinate
	double ( * m_MarkerCornerPosCam2d )[2];									// 2d marker corner position in camera-coordinate

	// Flag for each marker
	bool	* m_ValidFlagCam;

public:

	enum 
	{
		CAMERA,
		PROJECTOR
	};

	// =========================
	// Constructor & Destructor
	// =========================
	
	ARTagHelper ( int cameraW, int cameraH, const char * fnConfig, const char * fnCornerPos );
	virtual ~ARTagHelper();

	void FindMarkerCorners ( unsigned char * image );
	void DrawMarkersInCameraImage ( float pixZoomX, float pixZoomY );
	void DrawMarkersInCameraImage ( cv::Mat & img );

	void PrintMarkerCornersPos2dInCam () const;
	void PrintMarkerCornersPos3d() const;
	
	// 
	// Getters
	// 

	int GetMarkerNumber() const { return m_MarkerNum; }
	int GetValidMarkerNumber() const 
	{
		int validNum = 0;
		for ( int i = 0; i < m_MarkerNum; ++i ) {
			if ( m_ValidFlagCam[i] ) { validNum++; }
		}
		return validNum;
	}
	void GetValidFlagArray ( const bool *& validArray, int & markerNum ) const;

	void Clear ()
	{
		memset ( m_MarkerCornerPosCam2d, 0, m_MarkerNum * 4 * 2 * sizeof ( double ) );
		memset ( m_ValidFlagCam, false, m_MarkerNum );
	}

};

#endif