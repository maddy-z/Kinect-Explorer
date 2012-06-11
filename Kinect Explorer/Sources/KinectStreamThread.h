#ifndef _KINECTSTREAM_THREAD_H_
#define _KINECTSTREAM_THREAD_H_

#include <QtCore>

#include <qimage.h>

#include <XnOpenNI.h>
#include <XnCppWrapper.h>

// 
// Define Function Pointer Type
// 

typedef bool ( * HandleDepthMapFunc ) ( QImage & destImg, const xn::DepthGenerator & depthGen );
typedef bool ( * HandleColorMapFunc )  ( QImage & destImg, const xn::ImageGenerator & colorGen  );

// 
// Kinect Qt Wrapper
// 

class KinectStreamThread : public QThread

{

	Q_OBJECT

public:

	enum KinectMode
	{
		DEPTH					= 0, 
		RGB24					= 1, 
		RGB24_DEPTH		= 2 
	};

	enum KinectStatus
	{
		IDLE						= 0,
		INITIALIZING			= 1,
		RUNNING				= 2,
		STOPPED				= 3,
		DEVICE_ERROR		= 4
	};

	enum KinectStreamType
	{
		DEPTH_TYPE			= 0,
		RGB24_TYPE			= 1
	};

	// 
	// Constructors & Destructors
	// 

	KinectStreamThread ( QObject * obj = NULL );
	virtual ~KinectStreamThread();

	static std::string getKinectStatusString ( KinectStatus status );

	// 
	// Initialize and Config Kinect Context
	// 

	bool initKinectFromXML ( const char * filePath );
	bool initKinect ( KinectStreamThread::KinectMode kMode = KinectStreamThread::RGB24_DEPTH );
	bool initKinect ( KinectStreamThread::KinectMode kMode, 
					const XnMapOutputMode & depthMode, 
					const XnMapOutputMode & colorMode );
	bool initKinect ( KinectStreamThread::KinectMode kMode,
					unsigned int depthHeight, unsigned int depthWidth, unsigned int depthFPS,
					unsigned int colorHeight, unsigned int colorWidth, unsigned int colorFPS );

	// 
	// Thread Operation Functions
	// 

	bool startThread();
	bool pauseThread();
	bool stopThread();

	// virtual void run();

	// 
	// Getters
	// 

	const XnDepthPixel * getDepthRawData ();
	const XnRGB24Pixel * getColorRawData ();

	const QImage & getQImage ( KinectStreamType type );
	const QImage & getDepthQImage();
	const QImage & getColorQImage();

	const QImage & getUnsafeQImage ( KinectStreamType type );
	const QImage & getUnsafeDepthQImage ();
	const QImage & getUnsafeColorQImage ();

	QImage getSafeQImage ( KinectStreamType type );
	QImage getSafeDepthQImage ();
	QImage getSafeColorQImage ();

	double getColorFPS ();
	double getDepthFPS ();

	KinectStatus getKinectStatus();
	KinectMode getKinectMode();

	// 
	// Regist Depth Data Rendering Method
	// 
	
	void registColorDataHandlingFunc(HandleColorMapFunc pFunc) { 
		QMutexLocker locker(&m_DataMutex); 
		m_HandleColorFunc = pFunc; 
	}
	void registDepthDataHandlingFunc(HandleDepthMapFunc pFunc) { 
		QMutexLocker locker(&m_DataMutex); 
		m_HandleDepthFunc = pFunc; 
	}

	void unregistColorDataHandlingFunc() { 
		QMutexLocker locker(&m_DataMutex); 
		m_HandleColorFunc = KinectStreamThread::DefaultHandleColorMapFunc; 
	}
	void unregistDepthDataHandlingFunc() { 
		QMutexLocker locker(&m_DataMutex); 
		m_HandleDepthFunc = KinectStreamThread::DefaultHandleDepthMapFunc; 
	}

	bool getMapOutputMode(KinectStreamType type, XnMapOutputMode & mode);

	// 
	// Boolean Property Getters
	// 
	
	// TODO:
	bool isDepthOutputModeValid() const { return false; }
	bool isColorOutputModeValid() const { return false; }

	bool isRunning() const { return ( m_KinectStatus == RUNNING ) ? ( true ) : ( false ); }
	bool isPaused() const { return ( m_KinectStatus == IDLE ) ? ( true ) : ( false ); }
	bool isStopped() const { return ( m_KinectStatus == STOPPED ) ? ( true ) : ( false ); }

protected:

	// 
	// Reimplement the Virtual Core Function
	// 

	virtual void run();

private:

	// 
	// Convert Raw Data to Qt specified Image Class -- QImage
	// 

	void createQImageFromDepthRawData();
	void createQImageFromColorRawData();

	// static bool ColorDepthData(QImage & destImg, const xn::DepthGenerator & depthGen);
	static bool DefaultHandleColorMapFunc ( QImage & destImg, const xn::ImageGenerator & colorGen );
	static bool DefaultHandleDepthMapFunc ( QImage & destImg, const xn::DepthGenerator & depthGen );
	
	static XnStatus CheckOpenNIStatus(const char * str, XnStatus status);

	// 
	// Kinect Configuration Information
	// 
	
	KinectStreamThread::KinectMode m_KinectMode;
	KinectStreamThread::KinectStatus m_KinectStatus;

	// 
	// Internal OpenNI Nodes
	// 

	xn::Context					m_Context;

	xn::DepthGenerator		m_DepthGen;
	xn::ImageGenerator		m_ColorGen;

	xn::DepthMetaData		m_DepthMD;
	xn::ImageMetaData		m_ColorMD;

	XnMapOutputMode		m_DepthOutputMode;
	XnMapOutputMode		m_ColorOutputMode;

	// 
	// Other Logical Components
	// 

	QMutex							m_DataMutex;
	bool								m_RequestStop;
	
	HandleDepthMapFunc	m_HandleDepthFunc;
	HandleColorMapFunc		m_HandleColorFunc;

	QImage							* m_DepthQImageBuff;
	QImage							* m_ColorQImageBuff;

	double							m_ColorFPS;
	double							m_DepthFPS;

signals:
	
	void dataNotification();
	void statusChangedNotification ( KinectStreamThread::KinectStatus );

};

// 
// Register types used in Signal / Slot mechanism
// 

Q_DECLARE_METATYPE ( KinectStreamThread::KinectStatus );

#endif