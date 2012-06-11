#include <cstdio>
#include <cstdlib>
#include <string>
#include <sstream>

#include <QtCore>
#include <QtGUI>
#include <qmessagebox.h>
#include <qdialogbuttonbox.h>

#include <XnOpenNI.h>
#include <XnCppWrapper.h>

#include "..\GeneratedFiles\ui_MainWnd.h"
#include "MainWnd.h"

#include "KinectStreamThread.h"
#include "KinectHoleFiller.h"
#include "SimpleHeadTracking.h"
#include "HumanDetection.h"

#include "GlobalUtility.h"

// -------------------------------
//  Static Members Initialization
// -------------------------------

const QString MainWnd::KINECT_MODE_COLOR_DEPTH = "Rgb24_Depth";
const QString MainWnd::KINECT_MODE_COLOR = "Rgb24";
const QString MainWnd::KINECT_MODE_DEPTH = "Depth";

const QString MainWnd::KINECT_DEPTH_640X480X30 = "Depth_640x480Fps30";
const QString MainWnd::KINECT_DEPTH_320X240X30 = "Depth_320x240Fps30";
const QString MainWnd::KINECT_COLOR_640X480X30 = "Rgb24_640x480Fps30";
const QString MainWnd::KINECT_COLOR_320X240X30 = "Rgb24_320x240Fps30";

MainWnd::HoleFilling_Mode MainWnd::holeFillMode = MainWnd::HOLEFILLING_NONE;
MainWnd::DepthView_Mode MainWnd::depthViewMode = MainWnd::DEPTHVIEW_YELLOW;

// --------------
//  Constructors
// --------------

MainWnd::MainWnd(QWidget * parent, Qt::WFlags flags) : 
	QMainWindow(parent, flags),
	m_kinectThread(parent),
	m_isInitialized(false),
	m_colorKinectImage(0, 0, QImage::Format_RGB32),
	m_depthKinectImage(0, 0, QImage::Format_RGB32),
	m_HoleFillingActionGroup(parent),
	m_DepthViewActionGroup(parent),
	m_headTrackingActionGroup(parent),
	m_kinectStatusBarLabel(new QLabel())

{

	ui.setupUi(this);

	// --------------------------
	//  Initialize UI Components
	// --------------------------

	initComboBoxes();
	initRadioButtons();
	initMenuToolBar();
	initStatusBar();
	initActionGroup();
	initCannySettingsSlider();
	
	m_kinectThread.registDepthDataHandlingFunc(MainWnd::DefaultDepthView);

	showMaximized();

	// -------------------
	//  Build Connections
	// -------------------

	// QAction Connection Configuration
	connect(ui.action_AboutQt, SIGNAL(triggered()), this, SLOT(showAboutQtDialog()));
	connect(ui.action_AboutKinectExplorer, SIGNAL(triggered()), this, SLOT(showAboutKinectExplorerDialog()));

	connect(ui.action_InitFromXML, SIGNAL(triggered()), this, SLOT(initKinectSensorFromXML()));
	connect(ui.action_InitDirectly, SIGNAL(triggered()), this, SLOT(initKinectSensorDirectly()));

	connect(ui.action_StartKinect, SIGNAL(triggered()), this, SLOT(startKinectThread()));
	connect(ui.action_PauseKinect, SIGNAL(triggered()), this, SLOT(pauseKinectThread()));
	connect(ui.action_StopKinect, SIGNAL(triggered()), this, SLOT(stopKinectThread()));
	
	// Internal Connection Configuration
	connect(ui.kinectModeComboBox, SIGNAL(currentIndexChanged(const QString &)), this, SLOT(setRelatedComboBox(const QString &)));

	// Build Connection between UI and underlying-running Kinect Thread
	connect(&m_kinectThread, SIGNAL(dataNotification()), this, SLOT(showKinectView()));
	connect(&m_kinectThread, SIGNAL(statusChangedNotification(KinectStreamThread::KinectStatus)), this, SLOT(showKinectStatus(KinectStreamThread::KinectStatus)));
	connect(&m_kinectThread, SIGNAL(statusChangedNotification(KinectStreamThread::KinectStatus)), this, SLOT(setKinectOperationEnabled(KinectStreamThread::KinectStatus)));

	return;
}

// --------------
//  Public Slots
// --------------

void MainWnd::initKinectSensorDirectly ()
{
	if ( !m_kinectThread.isStopped() ) 
	{
		QMessageBox::information ( 
			this, 
			"Information", 
			"Kinect Sensor has not been stopped. Please stop Kinect firstly !!!", 
			QMessageBox::Ok );

		return;
	}

	// ------------------------------------------------------------------
	//  Initialize Kinect Sensor according to Kinect Setting Dock Widget
	// ------------------------------------------------------------------

	KinectStreamThread::KinectMode kinectMode;

	const QString kModeStr = ui.kinectModeComboBox->currentText();
	if (kModeStr == MainWnd::KINECT_MODE_COLOR) { 
		kinectMode = KinectStreamThread::RGB24; 
	}
	else if (kModeStr == MainWnd::KINECT_MODE_DEPTH) { 
		kinectMode = KinectStreamThread::DEPTH; 
	}
	else { 
		kinectMode = KinectStreamThread::RGB24_DEPTH; 
	}

	unsigned int rgbHeight = 480, rgbWidth = 640, rgbFps = 30;
	unsigned int depthHeight = 480, depthWidth = 640, depthFps = 30;

	QString rgbSettings, depthSettings;

	if ( rgbSettings == MainWnd::KINECT_COLOR_640X480X30 ) { rgbHeight = 480, rgbWidth = 640, rgbFps = 30; }
	else if ( rgbSettings == MainWnd::KINECT_COLOR_320X240X30 ) { rgbHeight = 240, rgbWidth = 320, rgbFps = 30; }

	if ( depthSettings == MainWnd::KINECT_DEPTH_640X480X30 ) { depthHeight = 480, depthWidth = 640, depthFps = 30; }
	else if ( depthSettings == MainWnd::KINECT_DEPTH_320X240X30 ) { depthHeight = 240, depthWidth = 320, depthFps = 30; }

	// 
	// Try to Initialize Kinect Sensor
	// 

	if ( m_kinectThread.initKinect ( 
		kinectMode, 
		depthHeight, depthWidth, depthFps, 
		rgbHeight, rgbWidth, rgbFps) == false ) 
	{
		QMessageBox::information ( 
			this,
			"Information",
			"Kinect Sensor has been unsuccessfully initialized.",
			QMessageBox::Ok);

		return;
	}

	m_isInitialized = true;

	setComboBoxesEnabled ( false );
}
void MainWnd::initKinectSensorFromXML()
{
	if ( !m_kinectThread.isStopped() ) 
	{
		QMessageBox::information(this, 
			"Information", 
			"Kinect Sensor has not been stopped. Please stop Kinect firstly!!", 
			QMessageBox::Ok);

		return;
	}

	QString fileName = QFileDialog::getOpenFileName ( this, "Choose XML File for Kinect Initialization" );
	if ( fileName.isEmpty() ) {
		return;
	}
	
	if ( m_kinectThread.initKinectFromXML ( fileName.toStdString().c_str() ) == false ) 
	{
		QMessageBox::information (
			this,
			"Information",
			"Kinect Sensor Initialization From XML File has failed.",
			QMessageBox::Ok );

		return;
	}

	XnMapOutputMode outputMode;
	KinectStreamThread::KinectMode kMode = m_kinectThread.getKinectMode();

	setComboBoxesEnabled ( true );
	setComboBoxesEditable ( true );

	if (kMode == KinectStreamThread::DEPTH) {
		m_kinectThread.getMapOutputMode(KinectStreamThread::DEPTH_TYPE, outputMode);
		std::string str = MainWnd::getXnMapOutputModeStdString(outputMode, KinectStreamThread::DEPTH_TYPE);
		
		ui.depthSettingsComboBox->setEditText(QString::fromStdString(str));
		ui.rgbSettingsComboBox->setEditText("");

		ui.kinectModeComboBox->setEditText(MainWnd::KINECT_MODE_DEPTH);
	}
	else if (kMode == KinectStreamThread::RGB24) {
		m_kinectThread.getMapOutputMode(KinectStreamThread::RGB24_TYPE, outputMode);
		std::string str = MainWnd::getXnMapOutputModeStdString(outputMode, KinectStreamThread::RGB24_TYPE);
		
		ui.rgbSettingsComboBox->setEditText(QString::fromStdString(str));
		ui.depthSettingsComboBox->setEditText("");

		ui.kinectModeComboBox->setEditText(MainWnd::KINECT_MODE_COLOR);
	}
	else if (kMode == KinectStreamThread::RGB24_DEPTH) {
		qDebug() << "RGB24_Depth";
		
		m_kinectThread.getMapOutputMode(KinectStreamThread::DEPTH_TYPE, outputMode);
		std::string depthStr = MainWnd::getXnMapOutputModeStdString(outputMode, KinectStreamThread::DEPTH_TYPE);

		m_kinectThread.getMapOutputMode(KinectStreamThread::RGB24_TYPE, outputMode);
		std::string colorStr = MainWnd::getXnMapOutputModeStdString(outputMode, KinectStreamThread::RGB24_TYPE);

		ui.rgbSettingsComboBox->setEditText(QString::fromStdString(colorStr));
		ui.depthSettingsComboBox->setEditText(QString::fromStdString(depthStr));

		ui.kinectModeComboBox->setEditText(MainWnd::KINECT_MODE_COLOR_DEPTH);
	}
	else {
		return;
	}

	setComboBoxesEnabled(false);

	m_isInitialized = true;
}

void MainWnd::showKinectView()
{
	// qDebug() << "Entering:\tMainWnd::showKinectView()";

	KinectStreamThread::KinectMode kMode = m_kinectThread.getKinectMode();
	// KinectStreamThread::KinectStatus kStatus = m_kinectThread.getKinectStatus();

	ui.centerWidgetLayout->removeWidget(ui.depthImageLabel640x480);
	ui.centerWidgetLayout->removeWidget(ui.colorImageLabel640x480);

	if (kMode == KinectStreamThread::DEPTH) {
		ui.centerWidgetLayout->addWidget(ui.depthImageLabel640x480);

		m_depthKinectImage = m_kinectThread.getDepthQImage();
		ui.depthImageLabel640x480->setPixmap(QPixmap::fromImage(m_depthKinectImage));
	}
	else if (kMode == KinectStreamThread::RGB24) { 
		ui.centerWidgetLayout->addWidget(ui.colorImageLabel640x480);

		m_colorKinectImage = m_kinectThread.getColorQImage();
		ui.colorImageLabel640x480->setPixmap(QPixmap::fromImage(m_colorKinectImage));
	}
	else if (kMode == KinectStreamThread::RGB24_DEPTH) {
		ui.centerWidgetLayout->addWidget(ui.colorImageLabel640x480);
		ui.centerWidgetLayout->addWidget(ui.depthImageLabel640x480);
		
		m_colorKinectImage = m_kinectThread.getColorQImage();
		m_depthKinectImage = m_kinectThread.getDepthQImage();
		
		ui.colorImageLabel640x480->setPixmap(QPixmap::fromImage(m_colorKinectImage));
		ui.depthImageLabel640x480->setPixmap(QPixmap::fromImage(m_depthKinectImage));
	}
	else { 
		return; 
	}

	// qDebug() << "Leaving:\tMainWnd::showKinectView()";
}
void MainWnd::showKinectStatus(KinectStreamThread::KinectStatus status)
{
	qDebug() << "Entering:\tMainWnd::showKinectStatus(KinectStreamThread::KinectStatus status)";
	
	std::string str = "Kinect Status: " + KinectStreamThread::getKinectStatusString(status);

	m_kinectStatusBarLabel->setText(QString::fromStdString(str));
	statusBar()->showMessage(" ");
	
	qDebug() << "Leaving:\tMainWnd::showKinectStatus(KinectStreamThread::KinectStatus status)";
}

// ---------------
//  Private Slots
// ---------------

void MainWnd::setRelatedComboBox(const QString & str)
{
	qDebug() << "Entering:\tMainWnd::setRelatedComboBox";

	if (str == MainWnd::KINECT_MODE_COLOR) {
		ui.rgbSettingsComboBox->setEnabled(true);
		ui.depthSettingsComboBox->setDisabled(true);
	}

	if (str == MainWnd::KINECT_MODE_DEPTH) {
		ui.depthSettingsComboBox->setEnabled(true);
		ui.rgbSettingsComboBox->setDisabled(true);
	}

	if (str == MainWnd::KINECT_MODE_COLOR_DEPTH) {
		ui.depthSettingsComboBox->setEnabled(true);
		ui.rgbSettingsComboBox->setEnabled(true);
	}

	qDebug() << "Leaving:\tMainWnd::setRelatedComboBox";
}
void MainWnd::setKinectOperationEnabled(KinectStreamThread::KinectStatus status)
{
	if (status == KinectStreamThread::STOPPED) {
		ui.action_StartKinect->setEnabled(true);
		ui.action_PauseKinect->setEnabled(false);
		ui.action_StopKinect->setEnabled(false);
	}
	else if (status == KinectStreamThread::INITIALIZING) {
		ui.action_StartKinect->setEnabled(false);
		ui.action_PauseKinect->setEnabled(false);
		ui.action_StopKinect->setEnabled(false);
	}
	else if (status == KinectStreamThread::IDLE) {
		ui.action_StartKinect->setEnabled(true);
		ui.action_PauseKinect->setEnabled(false);
		ui.action_StopKinect->setEnabled(true);
	}
	else if (status == KinectStreamThread::RUNNING) {
		ui.action_StartKinect->setEnabled(false);
		ui.action_PauseKinect->setEnabled(true);
		ui.action_StopKinect->setEnabled(true);
	}
	else if (status == KinectStreamThread::DEVICE_ERROR) {
		ui.action_StartKinect->setEnabled(true);
		ui.action_PauseKinect->setEnabled(true);
		ui.action_StopKinect->setEnabled(true);
	}
}

void MainWnd::showAboutQtDialog()
{
	QMessageBox::aboutQt(this, "About Qt");
}
void MainWnd::showAboutKinectExplorerDialog()
{
	// TODO:
	return;
}

// 
// Reimplemented Protected Event
// 

void MainWnd::closeEvent(QCloseEvent * event)
{
	KinectStreamThread::KinectStatus status = m_kinectThread.getKinectStatus();

	if (status != KinectStreamThread::STOPPED) 
	{
		if (QMessageBox::information(
			this,
			"Warning",
			"Kinect is still initialing, idling or running.\nDo you want to close Kinect Stream and Shut Kinect Sensor ???",
			QMessageBox::Cancel | QMessageBox::Ok) == QMessageBox::Ok)
		{
			if (m_kinectThread.stopThread()) {
				close();
			}
		}
	}
	else { close(); }
}

// 
// Private Slots
// 

void MainWnd::startKinectThread()
{
	if ( !m_isInitialized ) {
		QMessageBox::information(this, "Hint", "Please Initialize Kinect First !!!", QMessageBox::Ok);
		return;
	}

	if (m_kinectThread.startThread() == false) 
	{
		QMessageBox::information(
			this, 
			"Hint", 
			"Kinect can not be Started. Please try again.",
			QMessageBox::Ok);
	}
}
void MainWnd::pauseKinectThread()
{
	m_kinectThread.pauseThread();
}
void MainWnd::stopKinectThread()
{
	// 
	// If stop Kinect Thread, then you need to reinitialize the Kinect Sensor
	// 

	if (m_kinectThread.stopThread()) {
		m_isInitialized = false;
		
		setComboBoxesEnabled(true);
		setComboBoxesEditable(false);

		setRelatedComboBox(ui.kinectModeComboBox->currentText());
	}
	else {
		QMessageBox::information(
			this, 
			"Hint", 
			"Kinect Sensor can not be Stopped. Please try again.",
			QMessageBox::Ok, 
			QMessageBox::Ok);
	}
}

// 
// Private Member Functions
// 

void MainWnd::initComboBoxes()
{
	// 
	// Initialize Kinect Mode Selection Combo Box
	// 

	QStringList texts;
	texts.append(MainWnd::KINECT_MODE_COLOR_DEPTH);
	texts.append(MainWnd::KINECT_MODE_COLOR);
	texts.append(MainWnd::KINECT_MODE_DEPTH);	

	ui.kinectModeComboBox->addItems(texts);

	// 
	// Initialize Depth Display Mode Combo Box
	// 

	texts.clear();
	texts.append(MainWnd::KINECT_DEPTH_640X480X30);
	texts.append(MainWnd::KINECT_DEPTH_320X240X30);

	ui.depthSettingsComboBox->addItems(texts);

	// 
	// Initialize Color Display Mode
	// 

	texts.clear();
	texts.append(MainWnd::KINECT_COLOR_640X480X30);
	texts.append(MainWnd::KINECT_COLOR_320X240X30);

	ui.rgbSettingsComboBox->addItems(texts);

	ui.rgbSettingsComboBox->setEnabled(true);
	ui.depthSettingsComboBox->setDisabled(true);
}
void MainWnd::initRadioButtons()
{
	ui.openNI->setChecked(true);
}
void MainWnd::initMenuToolBar()
{
	ui.mainToolBar->addAction(ui.action_StartKinect);
	ui.mainToolBar->addAction(ui.action_PauseKinect);
	ui.mainToolBar->addAction(ui.action_StopKinect);

	ui.action_PauseKinect->setDisabled(true);
	ui.action_StopKinect->setDisabled(true);
}
void MainWnd::initStatusBar()
{
	m_kinectStatusBarLabel->setWordWrap(true);
	m_kinectStatusBarLabel->setVisible(true);

	ui.statusBar->addPermanentWidget(m_kinectStatusBarLabel);

	m_kinectStatusBarLabel->setText("Kinect Status: Stopped");
}
void MainWnd::initActionGroup()
{
	m_DepthViewActionGroup.setExclusive(true);

	m_DepthViewActionGroup.addAction(ui.action_DepthViewYellow);
	m_DepthViewActionGroup.addAction(ui.action_DepthViewRGB);
	m_DepthViewActionGroup.addAction(ui.action_DepthViewGray);

	ui.action_DepthViewYellow->setChecked(true);

	connect(ui.action_DepthViewYellow, SIGNAL(toggled(bool)), this, SLOT(setDepthViewMode(bool)));
	connect(ui.action_DepthViewRGB, SIGNAL(toggled(bool)), this, SLOT(setDepthViewMode(bool)));
	connect(ui.action_DepthViewGray, SIGNAL(toggled(bool)), this, SLOT(setDepthViewMode(bool)));

	// ------------------------------------------------

	m_HoleFillingActionGroup.setExclusive(true);
	
	m_HoleFillingActionGroup.addAction(ui.action_DefaultNoFilling);
	m_HoleFillingActionGroup.addAction(ui.action_NearestNeighborFilling);
	m_HoleFillingActionGroup.addAction(ui.action_FMMInpaintFilling);

	(ui.action_DefaultNoFilling)->setChecked(true);

	connect(ui.action_DefaultNoFilling, SIGNAL(toggled(bool)), this, SLOT(setHoleFillingMode(bool)));
	connect(ui.action_NearestNeighborFilling, SIGNAL(toggled(bool)), this, SLOT(setHoleFillingMode(bool)));
	connect(ui.action_FMMInpaintFilling, SIGNAL(toggled(bool)), this, SLOT(setHoleFillingMode(bool)));

	// ------------------------------------------------

	m_headTrackingActionGroup.setExclusive(true);

	m_headTrackingActionGroup.addAction(ui.action_DefaultDepthView);
	m_headTrackingActionGroup.addAction(ui.action_SimpleHeadTracking);
	m_headTrackingActionGroup.addAction(ui.action_HumanDetection);
	m_headTrackingActionGroup.addAction(ui.action_HumanDetectionWithoutProcessing);
	
	(ui.action_DefaultDepthView)->setChecked(true);

	connect(ui.action_DefaultDepthView, SIGNAL(toggled(bool)), this, SLOT(enableDefaultView(bool)));
	connect(ui.action_SimpleHeadTracking, SIGNAL(toggled(bool)), this, SLOT(enableFillShadow(bool)));
	connect(ui.action_HumanDetection, SIGNAL(toggled(bool)), this, SLOT(enableHumanDetection(bool)));
	connect(ui.action_HumanDetectionWithoutProcessing, SIGNAL(toggled(bool)), this, SLOT(enableHumanDetectionWithoutProcessing(bool)));

	// -------------------------------------------------
}
void MainWnd::initCannySettingsSlider()
{
	connect(ui.cannyThresholdValueSlider_1, SIGNAL(valueChanged(int)), this, SLOT(setCannyOperationThresholdValue(int)));
	connect(ui.cannyThresholdValueSlider_2, SIGNAL(valueChanged(int)), this, SLOT(setCannyOperationThresholdValue(int)));
}

void MainWnd::enableDefaultView(bool status)
{
	qDebug() << "Entering:\tMainWnd::enableDefaultView(bool status)";

	if (status == true) {
		m_kinectThread.registDepthDataHandlingFunc(MainWnd::DefaultDepthView);
	}
	else {
		m_kinectThread.unregistDepthDataHandlingFunc();
	}

	qDebug() << "Leaving:\tMainWnd::enableDefaultView(bool status)";
}

void MainWnd::enableFillShadow(bool status)
{
	qDebug() << "Entering:\tMainWnd::enableFillShadow(bool status)";

	if (status == false) {
		m_kinectThread.unregistDepthDataHandlingFunc();
	}
	else {
		m_kinectThread.registDepthDataHandlingFunc(SimpleHeadTracking::SimpleDepthHandlingFunc);
	}

	qDebug() << "Leaving:\tMainWnd::enableFillShadow(bool status)";
}

void MainWnd::enableHumanDetection(bool status)
{
	qDebug() << "Entering:\tMainWnd::enableHumanDetection(bool status)";

	if (status == false) {
		m_kinectThread.unregistDepthDataHandlingFunc();
	}
	else {
		m_kinectThread.registDepthDataHandlingFunc(HumanDetection::HumanDetectingAlgo);
	}

	qDebug() << "Leaving:\tMainWnd::enableHumanDetection(bool status)";
}

void MainWnd::enableHumanDetectionWithoutProcessing(bool status)
{
	qDebug() << "Entering:\tMainWnd::enableHumanDetection(bool status)";

	if (status == false) {
		m_kinectThread.unregistDepthDataHandlingFunc();
	}
	else {
		m_kinectThread.registDepthDataHandlingFunc(HumanDetection::HumanDetectingAlgoWithoutProcessing);
	}

	qDebug() << "Leaving:\tMainWnd::enableHumanDetection(bool status)";
}

void MainWnd::setDepthViewMode(bool status)
{
	qDebug() << "Entering:\tvoid MainWnd::setDepthViewMode()";

	if (ui.action_DepthViewYellow->isChecked()) { MainWnd::depthViewMode = MainWnd::DEPTHVIEW_YELLOW; }
	else if (ui.action_DepthViewRGB->isChecked()) { MainWnd::depthViewMode = MainWnd::DEPTHVIEW_COLOR; }
	else if (ui.action_DepthViewGray->isChecked()) { MainWnd::depthViewMode = MainWnd::DEPTHVIEW_GRAY; }

	qDebug() << "Leaving:\tvoid MainWnd::setDepthViewMode()";
}

void MainWnd::setCannyOperationThresholdValue(int value)
{
	qDebug() << "Entering:\tvoid MainWnd::setCannyOperationThresholdValue()";

	QObject * sPtr = QObject::sender();
	if (sPtr == ui.cannyThresholdValueSlider_1) {
		HumanDetection::thresholdValue_1 = ui.cannyThresholdValueSlider_1->value();
	}
	else if (sPtr == ui.cannyThresholdValueSlider_2) {
		HumanDetection::thresholdValue_2 = ui.cannyThresholdValueSlider_2->value();
	}

	qDebug() << "Leaving:\tvoid MainWnd::setCannyOperationThresholdValue()";
}

void MainWnd::setHoleFillingMode(bool status)
{
	qDebug() << "Entering:\tvoid MainWnd::setHoleFillingMode()";

	if (ui.action_DefaultNoFilling->isChecked()) { MainWnd::holeFillMode = MainWnd::HOLEFILLING_NONE; }
	else if (ui.action_NearestNeighborFilling->isChecked()) { MainWnd::holeFillMode = MainWnd::HOLEFILLING_NEARESTNEIGHBORHOOD; }
	else if (ui.action_FMMInpaintFilling->isCheckable()) { MainWnd::holeFillMode = MainWnd::HOLEFILLING_FMMINPAINT; }

	qDebug() << "Leaving:\tvoid MainWnd::setHoleFillingMode()";
}

bool MainWnd::DefaultDepthView(QImage & destImgBuff, const xn::DepthGenerator & depthGen)
{
	qDebug() << "Entering:\tbool MainWnd::DefaultDepthView(QImage & destImgBuff, const xn::DepthGenerator & depthGen)";

	// 
	// Invalid Depth Stream Input, then return an empty QImage
	// 

	bool result;

	xn::DepthMetaData depthMD;
	depthGen.GetMetaData(depthMD);

	const XnDepthPixel * srcDepthData = depthMD.Data();
	if (srcDepthData == NULL) {
		qDebug("Leaving:\tbool MainWnd::DefaultDepthView(QImage & destImgBuff, const xn::DepthGenerator & depthGen)");
		return false;
	}

	int nXRes = depthMD.XRes();
	int nYRes = depthMD.YRes();
	
	cv::Mat depthMat(nYRes, nXRes, CV_16UC1);
	result = GlobalUtility::CopyDepthRawBufToCvMat16u(srcDepthData, depthMat);
	
	// 
	// Step 1:	Filling Holes
	// 
	switch (MainWnd::holeFillMode) 
	{
	case MainWnd::HOLEFILLING_NONE:
		break;
	case MainWnd::HOLEFILLING_NEARESTNEIGHBORHOOD:
		result = KinectHoleFiller::NearestNeighborhoodHoleFilling(depthMat, depthMat);
		break;
	case MainWnd::HOLEFILLING_FMMINPAINT:
		break;
	}

	// 
	// Step 2:	Show Depth Data
	// 
	switch (MainWnd::depthViewMode)
	{
	case MainWnd::DEPTHVIEW_YELLOW:
		result = GlobalUtility::ConvertDepthCvMat16uToYellowQImage(depthMat, destImgBuff);
		break;
	case MainWnd::DEPTHVIEW_COLOR:
		result = GlobalUtility::ConvertDepthCvMat16uToColorfulQImage(depthMat, destImgBuff, depthGen.GetDeviceMaxDepth());
		break;
	case MainWnd::DEPTHVIEW_GRAY:
		result = GlobalUtility::ConvertDepthCvMat16uToGrayQImage(depthMat, destImgBuff);
		break;
	}

	qDebug("Leaving:\tbool MainWnd::DefaultDepthView(QImage & destImgBuff, const xn::DepthGenerator & depthGen)");

	return result;
}


std::string MainWnd::getXnMapOutputModeStdString(
	const XnMapOutputMode & mode, 
	KinectStreamThread::KinectStreamType type)
{
	std::string typeStr, resStr;
	
	resStr = GlobalUtility::ValueToString(mode.nXRes) + "x" + GlobalUtility::ValueToString(mode.nYRes)
		+ "Fps" + GlobalUtility::ValueToString(mode.nFPS);

	if (type == KinectStreamThread::DEPTH_TYPE) { typeStr = "Depth_"; }
	else if (type == KinectStreamThread::RGB24_TYPE) { typeStr = "RGB24_"; }
	else {
		return "";
	}

	return typeStr + resStr;
}

void MainWnd::setComboBoxesEnabled(bool flag)
{
	ui.kinectModeComboBox->setEnabled(flag);
	ui.rgbSettingsComboBox->setEnabled(flag);
	ui.depthSettingsComboBox->setEnabled(flag);
}

void MainWnd::setComboBoxesEditable(bool flag)
{
	ui.kinectModeComboBox->setEditable(flag);
	ui.rgbSettingsComboBox->setEditable(flag);
	ui.depthSettingsComboBox->setEditable(flag);
}
