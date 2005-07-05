#ifndef _ARTAG_HELPER_H_
#define _ARTAG_HELPER_H_

// #define		MARKER_CONFIG_FILE			"../../data/marker_cfg_RealPlay.txt"
// #define		MARKER_NUM_MAX				6

// 
// FORWARD DECLARATION
// 

class GrayCode;
class ExtrCalibrator;

// 
// Class -- ARTagHelper
// 

class ARTagHelper

{

private:
	
	// Camera Size
	int m_CameraWidth;
	int m_CameraHeight;

	int * m_MarkerID_LUT;													// LUT for marker ID
	int m_MarkerNum;														// Number of markers

	// char m_ConfigFile[128];												// ARTag config file name
	// char m_CornerPos[128];												// name of a file which contains 3d position of corners of ARTag markers in world-coordinate

	double ( * m_MarkerCornerPos3d )[3];									// 3d marker corner position in world-coordinate
	double ( * m_MarkerCornerPosCam2d )[2];									// 2d marker corner position in camera-coordinate
	// double ( * m_MarkerCornerPosPro2d )[2];									// 2d marker corner position in projector-coordinate

	// Flag for each marker
	// 
	//	- True: marker corner is visible from Camera / Projector
	//	- False: marker corner is not visible
	//	( for projector, visible means gray code can be obtained at the marker corner )
	//

	bool	* m_ValidFlagCam;
	// bool	* m_ValidFlagPro;

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
	// void GetMarkerCornerPos2dInProjector ( GrayCode * gc );
	void DrawMarkersInCameraImage ( float pixZoomX, float pixZoomY );

	void PrintMarkerCornersPos2dInCam () const;
	// void PrintMarkerCornersPos2dInProjector() const;
	void PrintMarkerCornersPos3d() const;
	
	// 
	// Getters
	// 

	int GetMarkerNumber() const { return m_MarkerNum; }
	void GetValidFlagArray ( int deviceType, const bool *& validArray, int & markerNum ) const;

};

#endif