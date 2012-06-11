#ifndef _MAINWND_H_
#define _MAINWND_H_

#include	<QtCore>
#include	<QtGui>
#include	<qimage.h>
#include	<qdebug.h>

#include	"KinectStreamThread.h"

#include	"..\GeneratedFiles\ui_MainWnd.h"

class MainWnd : public QMainWindow

{

	Q_OBJECT

public:

	MainWnd ( QWidget * parent = 0, Qt::WFlags flags = 0 );
	virtual ~MainWnd()
	{
		m_kinectThread.stopThread();
	}

protected:

	virtual void closeEvent ( QCloseEvent * event );

private slots:
	
	// 
	// Private Slots related with Kinect Operations
	// 

	void startKinectThread();
	void pauseKinectThread();
	void stopKinectThread();

	void initKinectSensorFromXML();
	void initKinectSensorDirectly();

	void showKinectView();
	void showKinectStatus(KinectStreamThread::KinectStatus status);
	
	// 
	// Private Slots related with UI Actions
	// 
	
	void showAboutKinectExplorerDialog();
	void showAboutQtDialog();

	void setRelatedComboBox(const QString & text);
	void setKinectOperationEnabled(KinectStreamThread::KinectStatus status);

	void enableDefaultView(bool status);
	void enableFillShadow(bool status);
	void enableHumanDetection(bool status);
	void enableHumanDetectionWithoutProcessing(bool status);

	void setHoleFillingMode(bool status);
	void setDepthViewMode(bool status);

	void setCannyOperationThresholdValue(int value);

private:

	enum HoleFilling_Mode { HOLEFILLING_NONE, HOLEFILLING_NEARESTNEIGHBORHOOD, HOLEFILLING_FMMINPAINT };
	enum DepthView_Mode { DEPTHVIEW_YELLOW, DEPTHVIEW_GRAY, DEPTHVIEW_COLOR };

	static bool DefaultDepthView ( QImage & destImgBuff, const xn::DepthGenerator & depthGen );
	static std::string getXnMapOutputModeStdString ( const XnMapOutputMode & mode, KinectStreamThread::KinectStreamType type );

	void setComboBoxesEnabled ( bool flag );
	void setComboBoxesEditable ( bool flag );

	// 
	// UI Components Initialization Functions
	// 

	void initMenuToolBar();
	void initStatusBar();
	void initActionGroup();

	void initComboBoxes();
	void initRadioButtons();
	void initCannySettingsSlider();

	// ------------------------------------
	// ------------------------------------

	//
	// Logical Components
	// 
	
	KinectStreamThread m_kinectThread;
	bool m_isInitialized;

	QImage m_colorKinectImage;
	QImage m_depthKinectImage;

	static HoleFilling_Mode holeFillMode;
	static DepthView_Mode depthViewMode;

	// 
	// UI Components
	// 
	
	Ui::MainWndClass ui;

	QActionGroup m_HoleFillingActionGroup;
	QActionGroup m_DepthViewActionGroup;
	QActionGroup m_headTrackingActionGroup;
	
	QLabel * m_kinectStatusBarLabel;

	// 
	// Static Members
	// 

	static const QString KINECT_MODE_DEPTH;
	static const QString KINECT_MODE_COLOR;
	static const QString KINECT_MODE_COLOR_DEPTH;

	static const QString KINECT_DEPTH_640X480X30;
	static const QString KINECT_DEPTH_320X240X30;
	
	static const QString KINECT_COLOR_640X480X30;
	static const QString KINECT_COLOR_320X240X30;
};

#endif // MAINWND_H
