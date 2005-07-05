#ifndef	_CALIB_SETTINGS_H_
#define _CALIB_SETTINGS_H_

#include <iostream>
#include <sstream>
#include <ctime>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

// 
// Helper Function
// 

inline void CalibHelp ()
{
	std::cout	<<  "This is a Camera Calibration Sample."	<< std::endl 
				<<  "Usage: Calibration Configuration File" << std::endl
				<<  "Near the sample file you'll find the configuration file, which has detailed help of how to edit it. "
					"It may be any OpenCV supported file format XML / YAML." 

				<< std::endl << std::endl;
}

// 
// Calibration Settings Class
// 

class Settings

{

public:
	
	Settings() : goodInput ( false ) {}

	enum Pattern	{ NOT_EXISTING, CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };
	enum InputType	{ INVALID, CAMERA, VIDEO_FILE, IMAGE_LIST };
	
	void write ( cv::FileStorage & fs ) const					// Write Serialization for this class
	{
		fs << "{" << "BoardSize_Width"							<< boardSize.width 
                  << "BoardSize_Height"							<< boardSize.height 
                  << "Square_Size"								<< squareSize
                  << "Calibrate_Pattern"						<< patternToUse
                  << "Calibrate_NrOfFrameToUse"					<< nrFrames
                  << "Calibrate_FixAspectRatio"					<< aspectRatio
                  << "Calibrate_AssumeZeroTangentialDistortion" << calibZeroTangentDist
                  << "Calibrate_FixPrincipalPointAtTheCenter"	<< calibFixPrincipalPoint

                  << "Write_DetectedFeaturePoints"				<< bwritePoints
                  << "Write_extrinsicParameters"				<< bwriteExtrinsics
                  << "Write_outputFileName"						<< outputFileName
                  
                  << "Show_UndistortedImage"					<< showUndistorsed
                  
                  << "Input_FlipAroundHorizontalAxis"			<< flipVertical
                  << "Input_Delay"								<< delay
                  << "Input"									<< input
           << "}";
	}
	
	void read ( const cv::FileNode & node )						// Read serialization for this class
	{
		node["BoardSize_Width" ]								>> boardSize.width;
        node["BoardSize_Height"]								>> boardSize.height;
        node["Calibrate_Pattern"]								>> patternToUse;
        node["Square_Size"]										>> squareSize;
        node["Calibrate_NrOfFrameToUse"]						>> nrFrames;
        node["Calibrate_FixAspectRatio"]						>> aspectRatio;
        node["Write_DetectedFeaturePoints"]						>> bwritePoints;
        node["Write_extrinsicParameters"]						>> bwriteExtrinsics;
        node["Write_outputFileName"]							>> outputFileName;
        node["Calibrate_AssumeZeroTangentialDistortion"]		>> calibZeroTangentDist;
        node["Calibrate_FixPrincipalPointAtTheCenter"]			>> calibFixPrincipalPoint;
        node["Input_FlipAroundHorizontalAxis"]					>> flipVertical;
        node["Show_UndistortedImage"]							>> showUndistorsed;
        node["Input"]											>> input;
        node["Input_Delay"]										>> delay;
        
		Interprate();
	}

    void Interprate()
    {
        goodInput = true; 
        
		if ( boardSize.width <= 0 || boardSize.height <= 0 )
        {
            std::cerr << "Invalid Board size: " << boardSize.width << " " << boardSize.height << std::endl;
            goodInput = false;
        }
        if ( squareSize <= 10e-6 )
        {
            std::cerr << "Invalid square size " << squareSize << std::endl;
            goodInput = false;
        }
        if ( nrFrames <= 0 )
        {
            std::cerr << "Invalid number of frames " << nrFrames << std::endl;
            goodInput = false;
        }
		
		if ( input.empty() ) {							// Check for VALID Input
			inputType = INVALID;
		}
        else
        {
            if (input[0] >= '0' && input[0] <= '9')
            {
                std::stringstream ss(input); 
				ss >> cameraID;

                inputType = CAMERA;
            }
            else
            {
                if ( ReadStringList ( input, imageList ) )
                {
					inputType = IMAGE_LIST;
					nrFrames = ( nrFrames < imageList.size() ) ? nrFrames : imageList.size();
				}
				else { 
					inputType = VIDEO_FILE; 
				}
            }

			if ( inputType == CAMERA ) { inputCapture.open ( cameraID ); } 
			if ( inputType == VIDEO_FILE ) { inputCapture.open ( input ); }
			if ( inputType != IMAGE_LIST && !inputCapture.isOpened () ) { inputType = INVALID; }
        }

		if ( inputType == INVALID ) 
		{
			std::cerr << " INEXISTENT Input: " << input;
			goodInput = false;
		}

		flag = 0; 

		if ( calibFixPrincipalPoint ) { flag |= CV_CALIB_FIX_PRINCIPAL_POINT;	}
		if ( calibZeroTangentDist )   { flag |= CV_CALIB_ZERO_TANGENT_DIST;		}
		if ( aspectRatio )            { flag |= CV_CALIB_FIX_ASPECT_RATIO;		}
		
		calibrationPattern = NOT_EXISTING;

		if ( !patternToUse.compare ("CHESSBOARD") )					{ calibrationPattern = CHESSBOARD; }
		if ( !patternToUse.compare ("CIRCLES_GRID") )				{ calibrationPattern = CIRCLES_GRID; }
		if ( !patternToUse.compare ("ASYMMETRIC_CIRCLES_GRID") )	{ calibrationPattern = ASYMMETRIC_CIRCLES_GRID; }
		if ( calibrationPattern == NOT_EXISTING )
		{
			std::cerr << " Inexistent Camera Calibration Mode: " << patternToUse << std::endl;
			goodInput = false;
		}
        
		atImageList = 0;
    }
	
    cv::Mat NextImage()
    {
        cv::Mat result;

        if ( inputCapture.isOpened() )
        {
            cv::Mat view0;
            inputCapture >> view0;
            view0.copyTo(result);
        }
        else if ( atImageList < (int)(imageList.size()) ) 
		{
			result = cv::imread ( imageList[atImageList++], CV_LOAD_IMAGE_COLOR );
		}

        return result;
    }
	
	static bool ReadStringList ( const std::string & filename, std::vector<std::string> & l )
	{
		l.clear();

		cv::FileStorage fs ( filename, cv::FileStorage::READ );
		if ( !fs.isOpened() ) { 
			return false; 
		}

		cv::FileNode n = fs.getFirstTopLevelNode();
		if ( n.type() != cv::FileNode::SEQ ) {
			return false;
		}

        cv::FileNodeIterator it = n.begin(), it_end = n.end();
        for ( ; it != it_end; ++it ) {
			l.push_back ( (std::string)(*it) );
		}

        return true;
    }

public: 

    cv::Size boardSize;				// The size of the board -> Number of items by width and height
    Pattern calibrationPattern;		// One of the Chessboard, circles, or asymmetric circle pattern
    float squareSize;				// The size of a square in your defined unit ( point, millimeter, etc ).
    int nrFrames;					// The number of frames to use from the input for calibration
    float aspectRatio;				// The aspect ratio
    int delay;						// In case of a video input 
    bool bwritePoints;				// Write detected feature points
    bool bwriteExtrinsics;			// Write extrinsic parameters
    bool calibZeroTangentDist;		// Assume zero tangential distortion
    bool calibFixPrincipalPoint;	// Fix the principal point at the center
    bool flipVertical;				// Flip the captured images around the horizontal axis
    std::string outputFileName;		// The name of the file where to write
    bool showUndistorsed;			// Show undistorted images after calibration
    std::string input;              // The input -> 

	int cameraID;
	std::vector<std::string> imageList;
	int atImageList;
	cv::VideoCapture inputCapture;
	InputType inputType;
	bool goodInput; 
	int flag;

private: 

	std::string patternToUse;

};

inline void write ( cv::FileStorage & fs, const std::string & str, const Settings & x )
{
    x.write(fs);
}

inline void read ( const cv::FileNode & node, Settings & x, const Settings & default_value = Settings() )
{
    if ( node.empty() ) {
		x = default_value;
	}
    else {
		x.read ( node );
	}
}

#endif