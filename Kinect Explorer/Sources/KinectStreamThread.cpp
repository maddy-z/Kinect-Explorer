#include <cstdio>
#include <cstdlib>

#include <QtCore>

#include <qimage.h>
#include <qdebug.h>

#include <XnOpenNI.h>
#include <XnCppWrapper.h>

#include "KinectStreamThread.h"

// 
// MACROS DEFINITIONS
// 

#define			OPENNI_MAX_DEPTH					10000
#define			MILLSECONDS_PER_SECOND		1000000.0f

// -------------
//  Contructors
// -------------

KinectStreamThread::KinectStreamThread ( QObject * obj ) : 
		QThread ( obj ),
		m_KinectMode ( RGB24_DEPTH ),
		m_KinectStatus ( STOPPED ),
		m_RequestStop ( true ),
		m_HandleDepthFunc ( KinectStreamThread::DefaultHandleDepthMapFunc ),
		m_HandleColorFunc ( KinectStreamThread::DefaultHandleColorMapFunc ),
		m_DepthQImageBuff ( NULL ),
		m_ColorQImageBuff ( NULL ),
		m_ColorFPS ( 0 ),
		m_DepthFPS ( 0 )
{
	// 
	// Register Types used in Signal / Slot mechanism
	// 

	qRegisterMetaType<KinectStreamThread::KinectStatus>( "KinectStreamThead::KinectStatus" );

	// 
	// Set Default Configuration
	// 

	m_DepthOutputMode.nXRes = 640;
	m_DepthOutputMode.nYRes = 480;
	m_DepthOutputMode.nFPS = 30;

	m_ColorOutputMode.nXRes = 640;
	m_ColorOutputMode.nYRes = 480;
	m_ColorOutputMode.nFPS = 30;

	emit statusChangedNotification(m_KinectStatus);
}
KinectStreamThread::~KinectStreamThread()
{
	if ( !stopThread() ) {
		return;
	}

	if ( m_ColorQImageBuff != NULL ) { delete m_ColorQImageBuff; }
	if ( m_DepthQImageBuff != NULL ) { delete m_DepthQImageBuff; }
}

std::string KinectStreamThread::getKinectStatusString ( KinectStatus status )
{
	switch (status)
	{

	case KinectStreamThread::IDLE:						return "Idle";
	case KinectStreamThread::INITIALIZING:			return "Initializing";
	case KinectStreamThread::RUNNING:				return "Running";
	case KinectStreamThread::STOPPED:				return "Stopped";
	case KinectStreamThread::DEVICE_ERROR:		return "Device Error";
	
	default:															return "Unknown Status";
	}
}

// 
// Initialize and Config OpenNI Environment for Kinect
// 

bool KinectStreamThread::initKinectFromXML(const char * str)
{
	qDebug() << "Entering:\tbool KinectStreamThread::initKinectFromXML(const char *)";

	// ----------------------------------------------

	m_DataMutex.lock();
	if (m_KinectStatus != KinectStreamThread::STOPPED) 
	{
		qDebug() << "Leaving:\tbool KinectStreamThread::initKinectFromXML(const char *)";

		m_DataMutex.unlock();
		return false;
	}
	m_DataMutex.unlock();

	// ----------------------------------------------

	XnStatus nRetVal = XN_STATUS_OK;
	
	xn::ScriptNode scriptNode;
	xn::EnumerationErrors errors;

	m_DataMutex.lock();
	m_KinectStatus = KinectStreamThread::INITIALIZING;
	m_DataMutex.unlock();

	emit statusChangedNotification(m_KinectStatus);
	
	m_DataMutex.lock();
	nRetVal = m_Context.InitFromXmlFile(str, scriptNode, &errors);
	m_DataMutex.unlock();
	if ( nRetVal == XN_STATUS_NO_NODE_PRESENT ) 
	{
		XnChar strError[1024]; errors.ToString(strError, 1024);
		qDebug("XN_STATUS_NO_NODE_PRESENT:\t%s\n", strError);
	
		m_DataMutex.lock();
		m_KinectStatus = KinectStreamThread::STOPPED;
		m_DataMutex.unlock();

		emit statusChangedNotification(m_KinectStatus);

		qDebug() << "Leaving:\tbool KinectStreamThread::initKinectFromXML(const char *)";

		system("pause");
		return false;
	}
	else if ( nRetVal != XN_STATUS_OK )
	{
		KinectStreamThread::CheckOpenNIStatus("OPEN FAILED", nRetVal);

		m_DataMutex.lock();
		m_KinectStatus = KinectStreamThread::STOPPED;
		m_DataMutex.unlock();

		emit statusChangedNotification(m_KinectStatus);

		qDebug() << "Leaving:\tbool KinectStreamThread::initKinectFromXML(const char *)";

		system("pause");
		return false;
	}

	bool isDepthNodeExist = true;
	bool isColorNodeExist = true;

	// 
	// Handle a Depth Generator node
	// 
	
	m_DataMutex.lock();
	nRetVal = m_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, m_DepthGen);
	m_DataMutex.unlock();

	if (nRetVal != XN_STATUS_OK) { isDepthNodeExist = false; }
	
	// 
	// Handle a Image Generator node
	// 
	
	m_DataMutex.lock();
	nRetVal = m_Context.FindExistingNode(XN_NODE_TYPE_IMAGE, m_ColorGen);
	m_DataMutex.unlock();
	
	if (nRetVal != XN_STATUS_OK) { isColorNodeExist = false; }

	if (isDepthNodeExist && isColorNodeExist) { 
		m_DataMutex.lock(); 
		m_KinectMode = KinectStreamThread::RGB24_DEPTH; 
		m_DataMutex.unlock(); 
	}
	else if (!isDepthNodeExist && isColorNodeExist) { 
		m_DataMutex.lock(); 
		m_KinectMode = KinectStreamThread::RGB24; 
		m_DataMutex.unlock();
	}
	else if (isDepthNodeExist && !isColorNodeExist) { 
		m_DataMutex.lock(); 
		m_KinectMode = KinectStreamThread::DEPTH; 
		m_DataMutex.unlock(); 
	}
	else {
		qDebug("No Image and Depth Node exist! Check your XML carefully!!\n");

		m_DataMutex.lock();
		m_KinectStatus = KinectStreamThread::STOPPED;
		m_DataMutex.unlock();

		emit statusChangedNotification(m_KinectStatus);
		
		qDebug() << "Leaving:\tbool KinectStreamThread::initKinectFromXML(const char *)";

		system("pause");
		return false;
	}

	m_DataMutex.lock();
	if ( isDepthNodeExist ) { m_DepthGen.GetMetaData(m_DepthMD); }
	if ( isColorNodeExist ) { m_ColorGen.GetMetaData(m_ColorMD); }
	m_DataMutex.unlock();

	m_DataMutex.lock();
	nRetVal = m_Context.StopGeneratingAll();
	m_DataMutex.unlock();

	if ( nRetVal != XN_STATUS_OK ) 
	{
		qDebug("Context Node cannot Stop Generating Data.");

		m_DataMutex.lock();
		m_KinectStatus = KinectStreamThread::STOPPED;
		m_DataMutex.unlock();
		
		emit statusChangedNotification(m_KinectStatus);
		
		qDebug() << "Leaving:\tbool KinectStreamThread::initKinectFromXML(const char *)";
		return false;
	}

	m_DataMutex.lock();
	m_KinectStatus = KinectStreamThread::IDLE;
	m_DataMutex.unlock();
	
	emit statusChangedNotification(m_KinectStatus);

	qDebug() << "Leaving:\tbool KinectStreamThread::initKinectFromXML(const char *)";
	
	return true;
}

bool KinectStreamThread::initKinect(KinectStreamThread::KinectMode kMode)
{
	return initKinect(kMode, 
					  480, 640, 30, 
					  480, 640, 30);
}
bool KinectStreamThread::initKinect(KinectStreamThread::KinectMode kMode, 
									const XnMapOutputMode & depthMode, 
									const XnMapOutputMode & colorMode)
{
	return initKinect(kMode, 
					  depthMode.nYRes, depthMode.nXRes, depthMode.nFPS,
					  colorMode.nYRes, colorMode.nXRes, colorMode.nFPS);
}
bool KinectStreamThread::initKinect(KinectStreamThread::KinectMode kMode, 
									unsigned int depthHeight, unsigned int depthWidth, unsigned int depthFPS,
									unsigned int colorHeight, unsigned int colorWidth, unsigned int colorFPS)
{
	qDebug() << "Entering:\tbool KinectStreamThread::initKinect(KinectStreamThread::KinectMode kMode,\n\
								                                unsigned int depthHeight, unsigned int depthWidth, unsigned int depthFPS,\n\
									                            unsigned int colorHeight, unsigned int colorWidth, unsigned int colorFPS)";
	
	m_DataMutex.lock();
	if (m_KinectStatus != KinectStreamThread::STOPPED) 
	{
		m_DataMutex.unlock();

		qDebug() << "Leaving:\tbool KinectStreamThread::initKinect(KinectStreamThread::KinectMode kMode,\n\
																   unsigned int depthHeight, unsigned int depthWidth, unsigned int depthFPS,\n\
									                               unsigned int colorHeight, unsigned int colorWidth, unsigned int colorFPS)";
		
		return false;
	}
	m_DataMutex.unlock();

	assert (m_KinectStatus == KinectStreamThread::STOPPED);

	switch (kMode) 
	{
	
	case DEPTH:
	case RGB24:
	case RGB24_DEPTH:

		m_DataMutex.lock();
		m_KinectMode = kMode;
		m_DataMutex.unlock();

		break;

	default:

		qDebug() << "Leaving:\tbool KinectStreamThread::initKinect(KinectStreamThread::KinectMode kMode,\n\
																   unsigned int depthHeight, unsigned int depthWidth, unsigned int depthFPS,\n\
									                               unsigned int colorHeight, unsigned int colorWidth, unsigned int colorFPS)";
		return false;
	
	}

	if ((depthHeight != 480 && depthHeight != 240) ||
		(depthWidth != 640 && depthWidth != 320) ||
		(depthFPS != 30))
	{
		qDebug() << "Leaving:\tbool KinectStreamThread::initKinect(KinectStreamThread::KinectMode kMode,\n\
																   unsigned int depthHeight, unsigned int depthWidth, unsigned int depthFPS,\n\
									                               unsigned int colorHeight, unsigned int colorWidth, unsigned int colorFPS)";
		return false;
	}
	
	XnMapOutputMode depthMode;
	depthMode.nXRes = depthWidth;
	depthMode.nYRes = depthHeight;
	depthMode.nFPS = depthFPS;

	if ((colorHeight != 480 && colorHeight != 240) ||
		(colorWidth != 640 && colorWidth != 320) ||
		(colorFPS != 30))
	{
		qDebug() << "Leaving:\tbool KinectStreamThread::initKinect(KinectStreamThread::KinectMode kMode,\n\
																   unsigned int depthHeight, unsigned int depthWidth, unsigned int depthFPS,\n\
									                               unsigned int colorHeight, unsigned int colorWidth, unsigned int colorFPS)";
		return false;
	}

	XnMapOutputMode colorMode;
	colorMode.nXRes = colorWidth;
	colorMode.nYRes = colorHeight;
	colorMode.nFPS = colorFPS;

	// -----------------------------------------------
	// -----------------------------------------------
	
	m_DataMutex.lock();
	m_KinectStatus = KinectStreamThread::INITIALIZING;
	m_DataMutex.unlock();
	
	emit statusChangedNotification(m_KinectStatus);
	
	m_DataMutex.lock();
	m_ColorOutputMode = colorMode;
	m_DepthOutputMode = depthMode;
	m_DataMutex.unlock();

	m_DataMutex.lock();
	XnStatus eResult = m_Context.Init();
	m_DataMutex.unlock();

	if (KinectStreamThread::CheckOpenNIStatus("Context Node Initialization Error", eResult) != XN_STATUS_OK) 
	{
		m_DataMutex.lock();
		m_KinectStatus = KinectStreamThread::STOPPED;
		m_DataMutex.unlock();
		
		emit statusChangedNotification(m_KinectStatus);

		qDebug() << "Leaving:\tbool KinectStreamThread::initKinect(KinectStreamThread::KinectMode kMode,\n\
																   unsigned int depthHeight, unsigned int depthWidth, unsigned int depthFPS,\n\
									                               unsigned int colorHeight, unsigned int colorWidth, unsigned int colorFPS)";

		return false;
	}
	
	m_DataMutex.lock();
	eResult = m_DepthGen.Create(m_Context);
	m_DataMutex.unlock();
	
	if (KinectStreamThread::CheckOpenNIStatus("Depth Generator Node Creating Error", eResult) != XN_STATUS_OK) 
	{
		m_DataMutex.lock();
		m_KinectStatus = KinectStreamThread::STOPPED;
		m_DataMutex.unlock();
		
		emit statusChangedNotification(m_KinectStatus);

		qDebug() << "Leaving:\tbool KinectStreamThread::initKinect(KinectStreamThread::KinectMode kMode,\n\
																   unsigned int depthHeight, unsigned int depthWidth, unsigned int depthFPS,\n\
									                               unsigned int colorHeight, unsigned int colorWidth, unsigned int colorFPS)";
		
		return false;
	}

	m_DataMutex.lock();
	eResult = m_ColorGen.Create(m_Context);
	m_DataMutex.unlock();

	if (KinectStreamThread::CheckOpenNIStatus("Image Generator Node Creating Error", eResult) != XN_STATUS_OK) 
	{
		m_DataMutex.lock();
		m_KinectStatus = KinectStreamThread::STOPPED;
		m_DataMutex.unlock();

		emit statusChangedNotification(m_KinectStatus);

		qDebug() << "Leaving:\tbool KinectStreamThread::initKinect(KinectStreamThread::KinectMode kMode,\n\
																   unsigned int depthHeight, unsigned int depthWidth, unsigned int depthFPS,\n\
									                               unsigned int colorHeight, unsigned int colorWidth, unsigned int colorFPS)";

		return false;
	}

	m_DataMutex.lock();
	eResult = m_DepthGen.SetMapOutputMode ( depthMode );
	m_DataMutex.unlock();	

	if (KinectStreamThread::CheckOpenNIStatus("Set Depth Map Output Mode Error", eResult) != XN_STATUS_OK) 
	{
		m_DataMutex.lock();
		m_KinectStatus = KinectStreamThread::STOPPED;
		m_DataMutex.unlock();

		emit statusChangedNotification(m_KinectStatus);

		qDebug() << "Leaving:\tbool KinectStreamThread::initKinect(KinectStreamThread::KinectMode kMode,\n\
																   unsigned int depthHeight, unsigned int depthWidth, unsigned int depthFPS,\n\
									                               unsigned int colorHeight, unsigned int colorWidth, unsigned int colorFPS)";

		return false;
	}

	m_DataMutex.lock();
	eResult = m_ColorGen.SetMapOutputMode ( colorMode ); 
	m_DataMutex.unlock();

	if (KinectStreamThread::CheckOpenNIStatus("Set Color Map Output Mode Error", eResult) != XN_STATUS_OK) 
	{
		m_DataMutex.lock();
		m_KinectStatus = KinectStreamThread::STOPPED;
		m_DataMutex.unlock();

		emit statusChangedNotification(m_KinectStatus);

		qDebug() << "Leaving:\tbool KinectStreamThread::initKinect(KinectStreamThread::KinectMode kMode,\n\
																   unsigned int depthHeight, unsigned int depthWidth, unsigned int depthFPS,\n\
									                               unsigned int colorHeight, unsigned int colorWidth, unsigned int colorFPS)";

		return false;
	}

	m_DataMutex.lock();
	m_KinectStatus = KinectStreamThread::IDLE;
	m_DataMutex.unlock();

	emit statusChangedNotification(m_KinectStatus);

	qDebug() << "Leaving:\tbool KinectStreamThread::initKinect(KinectStreamThread::KinectMode kMode,\n											\
								                               unsigned int depthHeight, unsigned int depthWidth, unsigned int depthFPS,\n		\
									                           unsigned int colorHeight, unsigned int colorWidth, unsigned int colorFPS)";
	

	return true;

}

bool KinectStreamThread::startThread()
{
	m_DataMutex.lock();
	
	if (m_KinectStatus == KinectStreamThread::RUNNING) { m_DataMutex.unlock(); return true; }
	if (m_KinectStatus != KinectStreamThread::IDLE) { m_DataMutex.unlock(); return false; }
	
	m_DataMutex.unlock();

	// 
	// Start to Produce Data
	// 

	m_DataMutex.lock();

	m_ColorGen.GetMetaData(m_ColorMD);
	m_DepthGen.GetMetaData(m_DepthMD);

	if ( m_ColorQImageBuff ) { delete m_ColorQImageBuff; }
	if ( m_DepthQImageBuff ) { delete m_DepthQImageBuff; }

	m_ColorQImageBuff = new QImage(m_ColorMD.XRes(), m_ColorMD.YRes(), QImage::Format_RGB32);
	m_DepthQImageBuff = new QImage(m_DepthMD.XRes(), m_DepthMD.YRes(), QImage::Format_RGB32);

	XnStatus nRetVal = m_Context.StartGeneratingAll();
	
	m_DataMutex.unlock();

	if (KinectStreamThread::CheckOpenNIStatus("Context Node cannot Generate Data.", nRetVal) != XN_STATUS_OK) {
		return false;
	}

	m_DataMutex.lock();
	m_RequestStop = false;
	m_DataMutex.unlock();

	// 
	// Start QThread actually here
	// 

	QThread::start();
	
	return true;
}
bool KinectStreamThread::pauseThread()
{
	m_DataMutex.lock();

	if ( m_KinectStatus == KinectStreamThread::IDLE ) { m_DataMutex.unlock(); return true; }
	if ( m_KinectStatus != KinectStreamThread::RUNNING ) { m_DataMutex.unlock(); return false; }

	m_RequestStop = true;
	
	m_DataMutex.unlock();

	// Block untill Kinect Stream Thread has been paused.
	QThread::wait();

	m_DataMutex.lock();
	XnStatus nRetVal = m_Context.StopGeneratingAll();
	m_DataMutex.unlock();
	
	if (KinectStreamThread::CheckOpenNIStatus("Context Node cannot Stop Generating Data.", nRetVal) != XN_STATUS_OK) 
	{
		m_DataMutex.lock();
		m_KinectStatus = KinectStreamThread::DEVICE_ERROR;
		m_DataMutex.unlock();
		
		emit statusChangedNotification(m_KinectStatus);

		return false;
	}

	m_DataMutex.lock();
	m_KinectStatus = KinectStreamThread::IDLE;
	m_DataMutex.unlock();
	
	emit statusChangedNotification(m_KinectStatus);

	return true;
}
bool KinectStreamThread::stopThread()
{
	m_DataMutex.lock();
	
	if (m_KinectStatus == KinectStreamThread::STOPPED) {
		m_DataMutex.unlock(); 
		return true; 
	}
	else if (m_KinectStatus == KinectStreamThread::DEVICE_ERROR) {
		m_DataMutex.unlock();
		return false;
	}
	
	m_DataMutex.unlock();

	// ---------------------------------
	// ---------------------------------

	m_DataMutex.lock();
	
	if (m_KinectStatus == KinectStreamThread::RUNNING) 
	{
		m_DataMutex.unlock();

		if (pauseThread() == false) 
		{
			m_DataMutex.lock();
			m_KinectStatus = KinectStreamThread::DEVICE_ERROR;
			m_DataMutex.unlock();
	
			emit statusChangedNotification(m_KinectStatus);

			return false;
		}
	}
	else {
		m_DataMutex.unlock();
	}
	
	//
	// If Kinect Thread is not running or stopped, its status must be initializing, idle.
	// 
	
	m_DataMutex.lock();
	
	assert (m_KinectStatus == KinectStreamThread::IDLE || 
			m_KinectStatus == KinectStreamThread::INITIALIZING);

	m_Context.Shutdown();
	m_KinectStatus = KinectStreamThread::STOPPED;

	m_DataMutex.unlock();

	emit statusChangedNotification(m_KinectStatus);

	return true;
}

// 
// Override Function
// 

void KinectStreamThread::run()
{
	qDebug("Entering:\tvoid KinectStreamThread::run()");
	
	m_DataMutex.lock();
	m_KinectStatus = KinectStreamThread::RUNNING;
	m_DataMutex.unlock();
	
	emit statusChangedNotification(m_KinectStatus);

	static XnUInt64 lastColorTS, curColorTS, 
					lastDepthTS, curDepthTS;

	if ( m_KinectMode == RGB24_DEPTH ) {
		lastDepthTS = m_DepthGen.GetTimestamp();
		lastColorTS = m_ColorGen.GetTimestamp();
	}
	else if ( m_KinectMode == RGB24 ) {
		lastColorTS = m_ColorGen.GetTimestamp();
	}
	else if ( m_KinectMode == DEPTH ) {
		lastDepthTS = m_DepthGen.GetTimestamp();
	}

	while ( !m_RequestStop ) 
	{
		XnStatus status = m_Context.WaitAndUpdateAll();
		if (status != XN_STATUS_OK) {
			qDebug() << "Frame Error";
			continue;
		}
		
		if ( m_KinectMode == RGB24_DEPTH )
		{
			if (m_ColorGen.IsDataNew()) 
			{
				m_DataMutex.lock();

				// m_ColorGen.GetMetaData(m_ColorMD);
				// createQImageFromColorRawData();
				// m_ColorQImage = createQImageFromColorRawData();

				m_HandleColorFunc((*m_ColorQImageBuff), m_ColorGen); 

				curColorTS = m_ColorGen.GetTimestamp();
				m_ColorFPS = MILLSECONDS_PER_SECOND / (float)(curColorTS - lastColorTS);
				lastColorTS = curColorTS;

				m_DataMutex.unlock();

				emit dataNotification();
			}
			
			if (m_DepthGen.IsDataNew()) 
			{
				m_DataMutex.lock();

				m_HandleDepthFunc(*m_DepthQImageBuff, m_DepthGen);

				curDepthTS = m_DepthGen.GetTimestamp();
				m_DepthFPS = MILLSECONDS_PER_SECOND / (float)(curDepthTS - lastDepthTS);
				lastDepthTS = curDepthTS;

				m_DataMutex.unlock();

				emit dataNotification();
			}
		}
		else if ( m_KinectMode == RGB24 ) 
		{
			m_DataMutex.lock();
			
			m_HandleColorFunc((*m_ColorQImageBuff), m_ColorGen);  

			curColorTS = m_ColorGen.GetTimestamp();
			m_ColorFPS = MILLSECONDS_PER_SECOND / (float)(curColorTS - lastColorTS);
			lastColorTS = curColorTS;
			
			m_DataMutex.unlock();
			
			emit dataNotification();
		}
		else if ( m_KinectMode == DEPTH ) {
			m_DataMutex.lock();

			m_HandleDepthFunc((*m_DepthQImageBuff), m_DepthGen);

			curDepthTS = m_DepthGen.GetTimestamp();
			m_DepthFPS = MILLSECONDS_PER_SECOND / (float)(curDepthTS - lastDepthTS);
			lastDepthTS = curDepthTS;
			
			m_DataMutex.unlock();

			emit dataNotification();
		}
		else {
			qDebug() << "Running Mode Error!!";
			
			return;
		}

		qDebug("Color FPS = %.2f", m_ColorFPS);
		qDebug("Depth FPS = %.2f", m_DepthFPS);
	}

	/*
	m_DataMutex.lock();

	XnStatus status = m_Context.StopGeneratingAll();
	if (KinectStreamThread::CheckOpenNIStatus("Context Node cannot Stop Generating Data", status)) {
		m_KinectStatus = KinectStreamThread::DEVICE_ERROR;
		emit statusChangedNotification(m_KinectStatus);
	}

	m_KinectStatus = KinectStreamThread::IDLE;
	emit statusChangedNotification(m_KinectStatus);

	m_DataMutex.unlock();
	*/

	qDebug("Leaving:\tvoid KinectStreamThread::run()");

	return;
}

//const XnDepthPixel * KinectStreamThread::getDepthRawData()
//{
//	QMutexLocker locker(&m_DataMutex);
//	return ( m_DepthRawData );
//}
//
//const XnRGB24Pixel * KinectStreamThread::getColorRawData()
//{
//	QMutexLocker locker(&m_DataMutex);
//	return ( m_ColorRawData );
//}


const QImage & KinectStreamThread::getQImage(KinectStreamType type)
{
	QMutexLocker locker(&m_DataMutex);

	switch (type)
	{
	case KinectStreamThread::DEPTH_TYPE : return (*m_DepthQImageBuff);
	case KinectStreamThread::RGB24_TYPE : return (*m_ColorQImageBuff);
	}

	return QImage(0, 0, QImage::Format_RGB32);
}

const QImage & KinectStreamThread::getColorQImage()
{
	QMutexLocker locker(&m_DataMutex);
	return (*m_ColorQImageBuff);
}
const QImage & KinectStreamThread::getDepthQImage()
{
	QMutexLocker locker(&m_DataMutex);
	return (*m_DepthQImageBuff);
}

const QImage & KinectStreamThread::getUnsafeQImage(KinectStreamType type)
{
	switch (type)
	{
	case KinectStreamThread::DEPTH_TYPE : return (*m_DepthQImageBuff);
	case KinectStreamThread::RGB24_TYPE : return (*m_ColorQImageBuff);
	}

	return QImage(0, 0, QImage::Format_RGB32);
}
const QImage & KinectStreamThread::getUnsafeDepthQImage() { return (*m_DepthQImageBuff); }
const QImage & KinectStreamThread::getUnsafeColorQImage() { return (*m_ColorQImageBuff); }

QImage KinectStreamThread::getSafeQImage(KinectStreamType type)
{
	if (type == KinectStreamThread::DEPTH_TYPE) {
		m_DataMutex.lock();
		QImage reImg = m_DepthQImageBuff->copy(0, 0, m_DepthQImageBuff->width(), m_DepthQImageBuff->height());
		m_DataMutex.unlock();

		return reImg;
	}
	else if (type == KinectStreamThread::RGB24_TYPE) {
		m_DataMutex.lock();
		QImage reImg = m_ColorQImageBuff->copy(0, 0, m_ColorQImageBuff->width(), m_ColorQImageBuff->height());
		m_DataMutex.unlock();

		return reImg;
	}
	else {
		return QImage(0, 0, QImage::Format_RGB32);
	}
}
QImage KinectStreamThread::getSafeDepthQImage()
{
	m_DataMutex.lock();
	QImage reImg = m_DepthQImageBuff->copy(0, 0, m_DepthQImageBuff->width(), m_DepthQImageBuff->height());
	m_DataMutex.unlock();

	return reImg;
}
QImage KinectStreamThread::getSafeColorQImage()
{
	m_DataMutex.lock();
	QImage reImg = m_ColorQImageBuff->copy(0, 0, m_ColorQImageBuff->width(), m_ColorQImageBuff->height());
	m_DataMutex.unlock();

	return reImg;
}

double KinectStreamThread::getColorFPS()
{
	QMutexLocker locker(&m_DataMutex);
	return m_ColorFPS;
}
double KinectStreamThread::getDepthFPS()
{
	QMutexLocker locker(&m_DataMutex);
	return m_DepthFPS;
}

KinectStreamThread::KinectMode KinectStreamThread::getKinectMode()
{
	QMutexLocker locker(&m_DataMutex);
	return m_KinectMode;
}
KinectStreamThread::KinectStatus KinectStreamThread::getKinectStatus(){
	QMutexLocker locker(&m_DataMutex);
	return m_KinectStatus;
}

bool KinectStreamThread::getMapOutputMode(KinectStreamType type, XnMapOutputMode & mode)
{
	if (type == KinectStreamThread::DEPTH_TYPE) {
		QMutexLocker locker(&m_DataMutex);

		m_DepthGen.GetMapOutputMode(mode);

		return true;
	}
	else if (type == KinectStreamThread::RGB24_TYPE) {
		QMutexLocker locker(&m_DataMutex);

		m_ColorGen.GetMapOutputMode(mode);

		return true;
	}
	else {
		return false;
	}
}

void KinectStreamThread::createQImageFromDepthRawData()
{
	xn::DepthMetaData depthMD; 
	m_DepthGen.GetMetaData(depthMD);
	
	const XnDepthPixel * depthRawData = m_DepthMD.Data();
	if (depthRawData == NULL) { return; }

	m_DepthGen.GetMetaData(m_DepthMD);
	return;
	// return m_RenderDepthMethod(m_DepthGen);
}
void KinectStreamThread::createQImageFromColorRawData()
{
	m_ColorGen.GetMetaData(m_ColorMD);
	
	const XnRGB24Pixel * colorRawData = m_ColorMD.RGB24Data();
	if (colorRawData == NULL) { return; }

	int nXRes = m_ColorMD.XRes(), 
		nYRes = m_ColorMD.YRes();

	const XnUInt8 * data = m_ColorMD.Data();

	for (unsigned y = 0; y < nYRes; ++y)
	{
		uchar * imagePtr = m_ColorQImageBuff -> scanLine(y);

		for (unsigned x = 0; x < nXRes; ++x, data += 3, imagePtr += 4)
		{
			imagePtr[0] = data[2];
			imagePtr[1] = data[1];
			imagePtr[2] = data[0];
			imagePtr[3] = 0xff;
		}
	}

	return;
}

// 
// Private Member Functions
// 

XnStatus KinectStreamThread::CheckOpenNIStatus(const char * str, XnStatus eResult)
{
	if ( eResult != XN_STATUS_OK ) {
		qDebug ( "%s:\t%s\n", str, xnGetStatusString ( eResult ) );
	}

	return eResult;
}

bool KinectStreamThread::DefaultHandleColorMapFunc(QImage & destImg, const xn::ImageGenerator & colorGen)
{
	xn::ImageMetaData colorMD;
	colorGen.GetMetaData(colorMD);
	
	const XnRGB24Pixel * colorRawData = colorMD.RGB24Data();
	if (colorRawData == NULL) {
		return false;
	}

	int nXRes = colorMD.XRes(), 
		nYRes = colorMD.YRes();

	assert (colorMD.DataSize() == nXRes * nYRes * sizeof(XnRGB24Pixel));

	const XnUInt8 * data = colorMD.Data();
	for (unsigned y = 0; y < nYRes; ++y)
	{
		uchar * imagePtr = destImg.scanLine(y);

		for (unsigned x = 0; x < nXRes; ++x, data += 3, imagePtr += 4)
		{
			imagePtr[0] = data[2];
			imagePtr[1] = data[1];
			imagePtr[2] = data[0];
			imagePtr[3] = 0xff;
		}
	}

	return true;
}
bool KinectStreamThread::DefaultHandleDepthMapFunc(QImage & destImg, const xn::DepthGenerator & depthGen)
{
	xn::DepthMetaData depthMD;
	depthGen.GetMetaData(depthMD);
	
	const XnDepthPixel * depthRawData = depthMD.Data();

	if (depthRawData == NULL) {
		return false;
	}

	assert (depthMD.DataSize() == depthMD.XRes() * depthMD.YRes() * sizeof(XnDepthPixel));

	int nXRes = depthMD.XRes();
	int nYRes = depthMD.YRes();

	float depthHist[OPENNI_MAX_DEPTH];
	xnOSMemSet(depthHist, 0, OPENNI_MAX_DEPTH * sizeof(float));

	unsigned int pointsNumber = 0;
	depthRawData = depthMD.Data();
	
	for (XnUInt y = 0; y < nYRes; ++y) 
	{
		for (XnUInt x = 0; x < nXRes; ++x, ++depthRawData) 
		{
			if ( *depthRawData ) {
				++depthHist[*depthRawData];
				++pointsNumber;
			}
		}
	}
	for (int i = 1; i < OPENNI_MAX_DEPTH; ++i) {
		depthHist[i] += depthHist[i-1];
	}
	if (pointsNumber > 0)
	{
		for (int i = 1; i < OPENNI_MAX_DEPTH; ++i) {
			depthHist[i] = (unsigned int)(256 * (1.0f - (depthHist[i] / pointsNumber)));
		}
	}
	
	depthRawData = depthMD.Data();
	
	for (XnUInt y = 0; y < nYRes; ++y) 
	{
		uchar * imageptr = destImg.scanLine(y);
	
		for (XnUInt x = 0; x < nXRes; ++x, ++depthRawData, imageptr += 4) 
		{
			if (*depthRawData)
			{
				imageptr[0] = 0;
				imageptr[1] = depthHist[*depthRawData];
				imageptr[2] = depthHist[*depthRawData];
				imageptr[3] = 0xff;
			}
			else 
			{
				imageptr[3] = imageptr[2] = imageptr[1] = imageptr[0] = (*depthRawData);
			}
		}
	}

	return true;
}