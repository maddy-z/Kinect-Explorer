#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <climits>

#include <iostream>

#include <Glew\glew.h>
#include <Glut\glut.h>

#include <AntTweakBar.h>

#include "Pos.h"
#include "Vec.h"
#include "Camera.h"
#include "Matrix.h"
#include "Quaternion.h"

// 
// MACROS DEFINITION
// 

#define		PI				3.14159265

#define		DEG2RAD			( PI / 180.0f )
#define		RAD2DEG			( 180.0f / PI )

#define		QUAT_DEBUG
#define		PROJ_DEBUG

// 
// FORWARD DECLARATIONS
// 

void		Reshape			( int width, int height );
void		Keyboard		( unsigned char key, int x, int y );
void		MouseMotion		( int x, int y );
void		MouseFunc		( int button, int state, int x, int y );

// 
// User-Interaction Mouse Settings
// 

#ifndef QUAT_DEBUG

int ui_LastMouseX			=	INT_MAX;
int ui_LastMouseY			=	INT_MAX;

bool IsDragRotEnable		=	false;

#endif

// 
// GLUT Window Settings
// 

int	glutMainWndHandler		=	-1;

const int glutMainWndWidth	=	800;
const int glutMainWndHeight	=	600;
const int glutMainWndPosX	=	150;
const int glutMainWndPosY	=	150;

// 
// Camera Arguments Settings
// 

Camera	g_Camera ( Camera::AIR_CAM );
Vec3f	g_CameraPos ( 0.0f, 13.0f, 15.0f );

float	g_CameraRotQuat[]	=	{ 0.0f, 0.0f, 0.0f, 1.0f };

// float g_CameraPosX		=	0.0f;
// float g_CameraPosY		=	13.0f;
// float g_CameraPosZ		=	15.0f;
//
// Vec3f g_CameraDirView  ( 0.0f, 1.0f, 0.0f );
// Vec3f g_CameraDirAlong ( 1.0f, 0.0f, 0.0f );
// Vec3f g_CameraDirUp	  ( 0.0f, 0.0f, 1.0f );
//
// float g_CameraDirView[]	=	{ 0.0f, 1.0f, 0.0f };
// float g_CameraDirSide[]	=	{ 1.0f, 0.0f, 0.0f };
// float g_CameraDirUp[]	=	{ 0.0f, 0.0f, 1.0f };

// 
// Scene Arguments Settings
// 

float g_SceneRotQuat[]		=	{ 0.0f, 0.0f, 0.0f, 1.0f };

float g_ObjMatAmbient[]		=	{ 0.7f, 0.5f, 0.8f, 1.0f };
float g_ObjMatDiffuse[]		=	{ 0.7f, 0.5f, 0.8f, 1.0f };
float g_ObjMatSpecular[]	=	{ 0.7f, 0.5f, 0.8f, 1.0f };
float g_ObjMatShininess[]	=	{ 128.0f };

float g_BoxMatAmbient[]		=	{ 0.4f, 0.2f, 0.1f, 1.0f };
float g_BoxMatDiffuse[]		=	{ 0.4f, 0.2f, 0.1f, 1.0f };
float g_BoxMatSpecular[]	=	{ 0.4f, 0.2f, 0.1f, 1.0f };
float g_BoxMatShininess[]	=	{ 128.0f };

// Box Size Settings
float g_BoxSizeX			=	11.0f;
float g_BoxSizeY			=	7.0f;
float g_BoxSizeZ			=	1.3f;
float g_BoxFaceThickness	=	0.1f;

const float g_BoxMetricSizeY	=	80.0f;															// Centimeter
const float g_BoxMetricSizeZ	=	( g_BoxSizeZ / g_BoxSizeY ) * g_BoxMetricSizeY;

float g_CameraMetricSizeX	=	0.0f;
float g_CameraMetricSizeY	=	100.0f;
float g_CameraMetricSizeZ	=	100.0f;

// Object and Cube Size Settings
float g_ObjSize				=	0.8f;
float g_CubeSize			=	1.0f;

// Global Light Arguments && Light 0 Settings
const float g_Light0_Ambient[]		=	{ 0.0f,  0.0f, 0.0f, 1.0f };
const float g_Light0_Diffuse[]		=	{ 1.0f,  1.0f, 1.0f, 1.0f };
const float g_Light0_Specular[]		=	{ 1.0f,  1.0f, 1.0f, 1.0f };
const float g_Light0_Position[]		=	{ 5.0f, -3.0f, 0.5f, 1.0f };

const float g_LightModelAmbient[]	=	{ 0.2f,  0.2f, 0.2f, 1.0f };

// Camera Projecting Frustum Settings
const float xLeft = ( -g_BoxSizeX ) / 2.0f - 5.0f;
const float xRight = ( g_BoxSizeX ) / 2.0f + 5.0f;
const float yBottom = ( -g_BoxSizeY ) / 2.0f - 5.0f;
const float yTop = ( g_BoxSizeY ) / 2.0f + 5.0f;

const float zNear = 1.0f;

// 
// OpenGL Initialization Function
// 

int GlInit ( void )
{
	// Set Clear State ( For Depth && Color Buffer )
	glClearDepth ( 1.0f );
	glClearColor ( 0.0f, 0.0f, 0.0f, 0.0f );

	glShadeModel ( GL_SMOOTH );
	glHint ( GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST );

	// glEnable ( GL_CULL_FACE );
	glEnable ( GL_DEPTH_TEST );
	glEnable ( GL_NORMALIZE );
	glEnable ( GL_LIGHTING );
	glEnable ( GL_LIGHT0 );

	glFrontFace ( GL_CCW );
	// glCullFace ( GL_FRONT );
	// glCullFace ( GL_BACK );

	// Set Window Size for Tweak Bar
	TwWindowSize ( glutMainWndWidth, glutMainWndHeight );

	return true;
}

void SceneInit ( void )
{
	// 
	// Initializing Global Light Model
	// 
	
	glLightModelfv	( GL_LIGHT_MODEL_AMBIENT,		g_LightModelAmbient );
	glLightModeli	( GL_LIGHT_MODEL_LOCAL_VIEWER,	GL_TRUE );
	glLightModeli	( GL_LIGHT_MODEL_TWO_SIDE,		GL_FALSE );
	glLightModeli	( GL_LIGHT_MODEL_COLOR_CONTROL, GL_SEPARATE_SPECULAR_COLOR );

	// Initializing LIGHT 0 Position and Materials
	glLightfv		( GL_LIGHT0, GL_AMBIENT,  g_Light0_Ambient  );
	glLightfv		( GL_LIGHT0, GL_DIFFUSE,  g_Light0_Diffuse  );
	glLightfv		( GL_LIGHT0, GL_SPECULAR, g_Light0_Specular );
	glLightfv		( GL_LIGHT0, GL_POSITION, g_Light0_Position );
	
	// 
	// Initializing Camera Position and Projection Arguments
	// 

	// float scaleFactor = 1.3f;
	float scaleFactor = 1.5f;

// #undef PROJ_DEBUG

#ifndef PROJ_DEBUG

	// Initializing Camera Position and Direction
	// g_CameraPos.Set ( 0.0f, 0.0f, ( 30.0f + g_BoxMetricSizeZ ) * ( g_BoxSizeY / g_BoxMetricSizeY ) * scaleFactor );
	
	// Vec3f	camAlong ( 1.0f, 0.0f, 0.0f );
	// Vec3f	camView ( 0.0f - g_CameraPos.x(), 0.0f - g_CameraPos.y(), 0.0f - g_CameraPos.z() );

	// g_Camera.SetCameraArg ( g_CameraPos, camView, camAlong );
	
	// Initializing Camera Position and Direction
	g_CameraPos.Set ( 0.0f, -g_CameraMetricSizeY * ( g_BoxSizeY / g_BoxMetricSizeY ) * scaleFactor, ( g_CameraMetricSizeZ + g_BoxMetricSizeZ ) * ( g_BoxSizeY / g_BoxMetricSizeY ) * scaleFactor );
	
	Vec3f	camAlong ( -g_CameraPos.y(), -(-g_CameraPos.x()), 0 );
	Vec3f	camView ( 0.0f - g_CameraPos.x(), 0.0f - g_CameraPos.y(), 0.0f - g_CameraPos.z() );
	
	g_Camera.SetCameraArg ( g_CameraPos, camView, camAlong );

#else
	
	// Initializing Camera Position and Direction
	// g_CameraPos.Set ( 0.0f, 0.0f, ( g_CameraMetricSizeZ + g_BoxMetricSizeZ ) * ( g_BoxSizeY / g_BoxMetricSizeY ) * scaleFactor );
	g_CameraPos.Set ( 0.0f * scaleFactor, ( -g_CameraMetricSizeY ) * ( g_BoxSizeY / g_BoxMetricSizeY ) * scaleFactor, ( g_CameraMetricSizeZ + g_BoxMetricSizeZ ) * ( g_BoxSizeY / g_BoxMetricSizeY ) * scaleFactor );
	
	Vec3f	camView  ( 0.0f, 0.0f, -1.0f );
	Vec3f	camAlong ( 1.0f, 0.0f,  0.0f );

	g_Camera.SetCameraArg ( g_CameraPos, camView, camAlong );

#endif

#ifndef PROJ_DEBUG

	// Viewport Settings
	glViewport ( 0, 0, glutMainWndWidth, glutMainWndHeight );

	// Camera Projection Settings
	glMatrixMode	( GL_PROJECTION );
	glLoadIdentity	();
	gluPerspective	( 45.0f, (GLdouble)(glutMainWndWidth) / (GLdouble)(glutMainWndHeight), 0.1f, 40.0f );

#else

	g_Camera.PrintPosition();

	const float camPosX = g_Camera.GetPosition().x();
	const float camPosY = g_Camera.GetPosition().y();
	const float camPosZ = g_Camera.GetPosition().z();
	
	// float xLeft = ( -g_BoxSizeX ) / 2.0f - 5.0f, xRight = ( g_BoxSizeX ) / 2.0f + 5.0f;
	// float yBottom = ( -g_BoxSizeY ) / 2.0f - 5.0f, yTop = ( g_BoxSizeY ) / 2.0f + 5.0f;
	/*
	float xLeft = ( -g_BoxSizeX ) / 2.0f - 5.0f, xRight = ( g_BoxSizeX ) / 2.0f + 5.0f;
	float yBottom = ( -g_BoxSizeY ) / 2.0f - 4.5f, yTop = ( g_BoxSizeY ) / 2.0f + 7.0f;
	*/
	// float zNear = 1.0f;

	const float x1 = ( xLeft - camPosX )	/ camPosZ * zNear;
	const float x2 = ( xRight - camPosX )	/ camPosZ * zNear;
	const float y1 = ( yBottom - camPosY )	/ camPosZ * zNear;
	const float y2 = ( yTop - camPosY )		/ camPosZ * zNear;

	char str[50];
	sprintf ( str, "< %.2f, %.2f, %.2f, %.2f >", x1, x2, y1, y2 );
	std::cout << "< x1, x2, y1, y2 > = " << std::string ( str ) << std::endl;

	// glViewport ( 0, 0, glutMainWndWidth, glutMainWndHeight );

	// Viewport Settings
	const float f = abs ( (x1 - x2) / (y1 - y2) );
	if ( f > (float)(glutMainWndWidth) / (float)(glutMainWndHeight) ) {
		glViewport ( 0, 0, glutMainWndWidth, (float)(glutMainWndWidth) / f );
	}
	else {
		glViewport ( 0, 0, (float)(glutMainWndHeight) * f, glutMainWndHeight );
	}

	// Camera Projection Settings
	glMatrixMode	( GL_PROJECTION );
	glLoadIdentity	();
	glFrustum		( x1, x2, y1, y2, zNear, camPosZ + 5.0f );

#endif

}

// 
// Main Loop Function
// 

void Display ( void )
{
	// 
	// Initialization
	// 

	glClear ( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

	// Vec3f camUp, camAlong ( g_CameraPosY, (-g_CameraPosX), 0 ), camView ( -g_CameraPosX, -g_CameraPosY, -g_CameraPosZ );
	// Vec3f::Cross3 ( camUp, camAlong, camView );

	// g_CameraPos = g_Camera.GetPosition();
	// g_CameraDirView = g_Camera.GetDirView();
	// g_CameraDirUp = g_Camera.GetDirUp();
	// g_CameraDirAlong = g_Camera.GetDirAlong();

#ifdef QUAT_DEBUG

	float camRotTmpMat[16] = { 0.0f };
	ConvertQuaternionToMatrix ( g_CameraRotQuat, camRotTmpMat );
	
	// Matrix camRotMat ( camRotTmpMat );

	// g_CameraDirView.Set ( 0.0f, 1.0f, 0.0f );
	// g_CameraDirSide.Set ( 1.0f, 0.0f, 0.0f );
	// g_CameraDirUp.Set	( 0.0f, 0.0f, 1.0f );
	// camRotMat.TransformDirection ( g_CameraDirView );
	// camRotMat.TransformDirection ( g_CameraDirAlong );
	// camRotMat.TransformDirection ( g_CameraDirUp );
	
	/*
	
	g_Camera.SetCameraArg ( g_Camera.GetPosition(), g_CameraDirView, g_CameraDirAlong );

	char str[50];
	sprintf ( str, "Vec4f < %.2f, %.2f, %.2f, %.2f >", camRotTmpMat[0], camRotTmpMat[1], camRotTmpMat[2], camRotTmpMat[3] );
	std::cout << "Camera Rot Quat:\t" << std::string ( str ) << std::endl;
	std::cout << "Camera Dir View:\t" << g_CameraDirView.toString() << std::endl;
	std::cout << "Camera Dir Side:\t" << g_CameraDirAlong.toString() << std::endl;
	std::cout << "Camera Dir Up:\t\t" << g_CameraDirUp.toString() << std::endl;
	std::cout << std::endl;
	
	*/

#endif

	glMatrixMode ( GL_MODELVIEW );
	glLoadIdentity ();
	// gluLookAt ( g_CameraPosX, g_CameraPosY, g_CameraPosZ, 0.0f, 0.0f, 0.0f, camUp.x(), camUp.y(), camUp.z() );

	/*
	gluLookAt (	g_CameraPosX, g_CameraPosY, g_CameraPosZ, 
				g_CameraPosX + g_CameraDirView.x(), g_CameraPosY + g_CameraDirView.y(), g_CameraPosZ + g_CameraDirView.z(), 
				g_CameraDirUp.x(), g_CameraDirUp.y(), g_CameraDirUp.z() );
	*/

	glMultMatrixf ( camRotTmpMat );
	g_Camera.Update ();
	
	float sceneRotTmpMat[16] = { 0.0f };
	ConvertQuaternionToMatrix ( g_SceneRotQuat, sceneRotTmpMat );
	glMultMatrixf ( sceneRotTmpMat );

	/*
	gluLookAt (	g_CameraPos.x(), g_CameraPos.y(), g_CameraPos.z(), 
				g_CameraPos.x() + g_CameraDirView.x(), g_CameraPos.y() + g_CameraDirView.y(), g_CameraPos.z() + g_CameraDirView.z(), 
				g_CameraDirUp.x(), g_CameraDirUp.y(), g_CameraDirUp.z() 
				);
	*/

	// 
	// Draw Camera Axes
	// 
	//
	// glColor3f ( 1.0f, 0.0f, 0.0f );						// Forward Axe
	// glBegin ( GL_LINES );
	//	glVertex3f (	g_CameraPos.x() + g_CameraDirView.x(), g_CameraPos.y() + g_CameraDirView.y(), g_CameraPos.z() + g_CameraDirView.z() );
	//	glVertex3f (	g_CameraPos.x() + 2.0f * g_CameraDirView.x(), 
	//					g_CameraPos.y() + 2.0f * g_CameraDirView.y(), 
	//					g_CameraPos.z() + 2.0f * g_CameraDirView.z() );
	// glEnd();

	// glColor3f ( 0.0f, 0.0f, 1.0f );						// Along Axe
	// glBegin ( GL_LINES );
	//	glVertex3f (	g_CameraPos.x() + g_CameraDirView.x(), g_CameraPos.y() + g_CameraDirView.y(), g_CameraPos.z() + g_CameraDirView.z() );
	//	glVertex3f (	g_CameraPos.x() + g_CameraDirView.x() + g_CameraDirAlong.x(), 
	//					g_CameraPos.y() + g_CameraDirView.y() + g_CameraDirAlong.y(), 
	//					g_CameraPos.z() + g_CameraDirView.z() + g_CameraDirAlong.z());
	// glEnd();

	// glColor3f ( 0.0f, 1.0f, 0.0f );						// Up Axe
	// glBegin ( GL_LINES );
	//	glVertex3f (	g_CameraPos.x() + g_CameraDirView.x(), g_CameraPos.y() + g_CameraDirView.y(), g_CameraPos.z() + g_CameraDirView.z() );
	//	glVertex3f (	g_CameraPos.x() + g_CameraDirView.x() + g_CameraDirUp.x(), 
	//					g_CameraPos.y() + g_CameraDirView.y() + g_CameraDirUp.y(), 
	//					g_CameraPos.z() + g_CameraDirView.z() + g_CameraDirUp.z() );
	// glEnd();

	// 
	// Draw Box
	// 
	
	// Set Box Material
    glMaterialfv ( GL_FRONT, GL_AMBIENT, g_BoxMatAmbient );
    glMaterialfv ( GL_FRONT, GL_DIFFUSE, g_BoxMatDiffuse );
	glMaterialfv ( GL_FRONT, GL_SPECULAR, g_BoxMatSpecular );
	glMaterialfv ( GL_FRONT, GL_SHININESS, g_BoxMatShininess );

	// Bottom Face
	glPushMatrix ();
	glScalef ( g_BoxSizeX, g_BoxSizeY, g_BoxFaceThickness );
	glBegin ( GL_QUADS );
		glutSolidCube ( g_CubeSize );
	glEnd ();
	glPopMatrix ();
 	
	// Front Face
	glPushMatrix ();
	glTranslatef ( 0.0f, -g_BoxSizeY / 2.0f, g_BoxSizeZ / 2.0f );
	glRotatef ( 90.0f, 1.0f, 0.0f, 0.0f );
	glScalef ( g_BoxSizeX, g_BoxSizeZ, g_BoxFaceThickness );
	glBegin ( GL_QUADS );
		glutSolidCube ( g_CubeSize );
	glEnd ();
	glPopMatrix ();

	// Right Face
	glPushMatrix ();
	glTranslatef ( g_BoxSizeX / 2.0f, 0.0f, g_BoxSizeZ / 2.0f );
	glRotatef ( 90.0f, 0.0f, 1.0f, 0.0f );
	glScalef ( g_BoxSizeZ, g_BoxSizeY, g_BoxFaceThickness );
	glBegin ( GL_QUADS );
		glutSolidCube ( g_CubeSize );
	glEnd ();
	glPopMatrix ();

	// Back Face
	glPushMatrix ();
	glTranslatef ( 0.0f, g_BoxSizeY / 2.0f, g_BoxSizeZ / 2.0f );
	glRotatef ( 90.0f, 1.0f, 0.0f, 0.0f );
	glScalef ( g_BoxSizeX, g_BoxSizeZ, g_BoxFaceThickness );
	glBegin ( GL_QUADS );
		glutSolidCube ( g_CubeSize );
	glEnd ();
	glPopMatrix ();

	// Left Face
	glPushMatrix ();
	glTranslatef ( (-g_BoxSizeX) / 2.0f, 0.0f, g_BoxSizeZ / 2.0f );
	glRotatef ( 90.0f, 0.0f, 1.0f, 0.0f );
	glScalef ( g_BoxSizeZ, g_BoxSizeY, g_BoxFaceThickness );
	glBegin ( GL_QUADS );
		glutSolidCube ( g_CubeSize );
	glEnd ();
	glPopMatrix ();
	
	// 
	// Draw Teapot
	// 
	// Set Box Material
    glMaterialfv ( GL_FRONT, GL_AMBIENT, g_ObjMatAmbient );
    glMaterialfv ( GL_FRONT, GL_DIFFUSE, g_ObjMatDiffuse );
	glMaterialfv ( GL_FRONT, GL_SPECULAR, g_ObjMatSpecular );
	glMaterialfv ( GL_FRONT, GL_SHININESS, g_ObjMatShininess );

	glPushMatrix ();
	glTranslatef ( 0.0f, 0.0f, g_ObjSize / 1.3f );
	// glMultMatrixf ( camRotTmpMat );
	glRotatef ( 90.0f, 1.0f, 0.0f, 0.0f );
		glutSolidTeapot ( g_ObjSize );
	glPopMatrix ();

	// Draw Tweak Bars
	TwDraw ();

	// Flush and Swap Buffer
	glutSwapBuffers ();

	// Recall Display at next frame
	glutPostRedisplay ();
}

// 
// Main Function
// 

int main ( int argc, char ** argv )
{
	//
	// GLUT Initialization
	// 
	
	glutInit ( &argc, argv );
	glutInitDisplayMode ( GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH );
	glutInitWindowPosition ( glutMainWndPosX, glutMainWndPosY );
	glutInitWindowSize ( glutMainWndWidth, glutMainWndHeight );

	// glutMainWndHandler = glutCreateWindow ( "Simple 3D Scene" );
	glutCreateWindow ( "Simple 3D Scene" );
	glutCreateMenu ( NULL );

	glutDisplayFunc ( Display );
	glutReshapeFunc ( Reshape );
	glutIdleFunc ( NULL );
	glutFullScreen ();

	// OpenGL Initialization
	GlInit ();
	SceneInit ();

	// Tw Initialization
	TwInit ( TW_OPENGL, NULL );                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          

	// Set GLUT event callbacks
	// - Directly redirect GLUT mouse button events to AntTweakBar
	glutMouseFunc ( MouseFunc );
	// glutMouseFunc ( ( GLUTmousebuttonfun ) ( TwEventMouseButtonGLUT ) );
	// - Directly redirect GLUT mouse motion events to AntTweakBar
	glutMotionFunc ( MouseMotion );
	// glutMotionFunc ( ( GLUTmousemotionfun ) ( TwEventMouseMotionGLUT ) );
	// - Directly redirect GLUT mouse "passive" motion events to AntTweakBar (same as MouseMotion)
	glutPassiveMotionFunc ( ( GLUTmousemotionfun ) ( TwEventMouseMotionGLUT ) );
	// - Directly redirect GLUT key events to AntTweakBar
	glutKeyboardFunc ( Keyboard );
	// glutKeyboardFunc ( ( GLUTkeyboardfun ) ( TwEventKeyboardGLUT ) );
	// - Directly redirect GLUT special key events to AntTweakBar
	glutSpecialFunc ( ( GLUTspecialfun ) ( TwEventSpecialGLUT ) );
	// - Send 'glutGetModifers' function pointer to AntTweakBar;
	//   required because the GLUT key event functions do not report key modifiers states.
	TwGLUTModifiersFunc ( glutGetModifiers );

	TwBar * bar = TwNewBar ( "TweakBar" );
	TwDefine ( " GLOBAL help='This example shows a Simple 3D scene.' " ); 

	TwAddVarRW ( bar, "Camera Rotation", TW_TYPE_QUAT4F, &g_CameraRotQuat, " label='Camera Rotation' opened=true help='Change the Camera orientation.' " );
	TwAddVarRW ( bar, "Scene Rotation", TW_TYPE_QUAT4F, &g_SceneRotQuat, " label='Scene Rotation' opened=true help='Change the Scene orientation.' " );

	TwAddVarRW ( bar, "Object Zoom", TW_TYPE_FLOAT, &g_ObjSize, " min=1.00 max=7.00 step=0.10 keyIncr=z keyDecr=Z help='Scale the object (1=original size).' ");

	TwAddVarRW ( bar, "Obj Ambient", TW_TYPE_COLOR3F, &g_ObjMatAmbient, " group='Object Material' " );
	TwAddVarRW ( bar, "Obj Diffuse", TW_TYPE_COLOR3F, &g_ObjMatDiffuse, " group='Object Material' " );
	TwAddVarRW ( bar, "Obj Specular", TW_TYPE_COLOR3F, &g_ObjMatSpecular, " group='Object Material' " );
	TwAddVarRW ( bar, "Obj Shininess", TW_TYPE_FLOAT, &g_ObjMatShininess, " min=0.00 max=128.00 step=1.00 group='Object Material' " );

	TwAddVarRW ( bar, "Box Ambient", TW_TYPE_COLOR3F, &g_BoxMatAmbient, " group='Box Material' " );
	TwAddVarRW ( bar, "Box Diffuse", TW_TYPE_COLOR3F, &g_BoxMatDiffuse, " group='Box Material' " );
	TwAddVarRW ( bar, "Box Specular", TW_TYPE_COLOR3F, &g_BoxMatSpecular, " group='Box Material' " );
	TwAddVarRW ( bar, "Box Shininess", TW_TYPE_FLOAT, &g_BoxMatShininess, " min=0.00 max=128.00 step=1.00 group='Box Material' " );

	// 
	// Start to Glut Loop
	// 
	glutMainLoop ();

	// Tw Terminate
	TwTerminate ();	

	exit ( EXIT_SUCCESS );
}

// 
// GLUT Callback Functions
// 

// 
// Reshape Function
// 

void Reshape ( int width, int height )

{
	std::cout << "Start:\tGLUT Window Reshape" << std::endl;

	if ( height == 0 ) {
		height = 1;
	}

#ifndef PROJ_DEBUG

	// Viewport Settings
	glViewport ( 0, 0, width, height );

	// Camera Projection Settings
	glMatrixMode	( GL_PROJECTION );
	glLoadIdentity	();
	gluPerspective	( 45.0f, (GLdouble)(width) / (GLdouble)(height), 0.1f, 30.0f );

#else

	g_Camera.PrintPosition();

	const float camPosX = g_Camera.GetPosition().x();
	const float camPosY = g_Camera.GetPosition().y();
	const float camPosZ = g_Camera.GetPosition().z();
	
	/*
	float xLeft = ( -g_BoxSizeX ) / 2.0f - 5.0f, xRight = ( g_BoxSizeX ) / 2.0f + 5.0f;
	float yBottom = ( -g_BoxSizeY ) / 2.0f - 5.0f, yTop = ( g_BoxSizeY ) / 2.0f + 5.0f;
	float zNear = 1.0f;
	*/

	const float x1 = ( xLeft - camPosX )	/ camPosZ * zNear;
	const float x2 = ( xRight - camPosX )	/ camPosZ * zNear;
	const float y1 = ( yBottom - camPosY )	/ camPosZ * zNear;
	const float y2 = ( yTop - camPosY )		/ camPosZ * zNear;

	char str[50];
	sprintf ( str, "< %.2f, %.2f, %.2f, %.2f >", x1, x2, y1, y2 );
	std::cout << "< x1, x2, y1, y2 > = " << std::string ( str ) << std::endl;

	// glViewport ( 0, 0, width, height );

	// Viewport Settings
	const float f = abs ( (x1 - x2) / (y1 - y2) ); 

	if ( f > (float)(width) / (float)(height) ) { glViewport ( 0, 0, width, (float)(width) / f ); }
	else { glViewport ( 0, 0, (float)(height) * f, height ); }

	// Camera Projection Settings
	glMatrixMode	( GL_PROJECTION );
	glLoadIdentity	();
	glFrustum		( x1, x2, y1, y2, zNear, camPosZ + 5.0f );

#endif

	// Send New Window Size to AntTweakBar
    TwWindowSize ( width, height );

	// Recall Display at Next Frame
	glutPostRedisplay ();

	std::cout << "End:\tGLUT Window Reshape" << std::endl;
}

// 
// Keyboard Function
// 

void Keyboard ( unsigned char key, int x, int y )

{
	float speed = 0.5f;
	bool wall[4] = { false, false, false, false };

	switch ( key ) 
	
	{
	
	case 27:	exit(0);

	case 'w':	g_Camera.Walk (  speed, wall );		break;
	case 's':	g_Camera.Walk ( -speed, wall );		break;
	case 'd':	g_Camera.Strafe (  speed, wall );	break;
	case 'a':	g_Camera.Strafe ( -speed, wall );	break;

	default:	break;
	
	}

	// Call AntTweakBar Event Function
	TwEventKeyboardGLUT ( key, x, y );

	// Recall Display at Next Frame
	glutPostRedisplay ();
}

// 
// Mouse Motion Function
// 

void MouseMotion ( int x, int y )

{

#ifndef QUAT_DEBUG

	if ( !IsDragRotEnable ) {
		return;
	}

	int deltaX = 150.0f;
	int deltaY = 150.0f;

	/*
	if ( ui_LastMouseX == INT_MAX || ui_LastMouseY == INT_MAX ) 
	{
		ui_LastMouseX = x;
		ui_LastMouseY = y;
	}
	*/

	GLfloat thetaX = ( float )( ui_LastMouseX - x ) / ( float )( deltaX ) * RAD2DEG;
	GLfloat thetaY = ( float )( y - ui_LastMouseY ) / ( float )( deltaY ) * RAD2DEG;

	g_Camera.Yaw ( thetaX );
	// g_Camera.Roll ( thetaY );
	g_Camera.Pitch ( thetaY );

	ui_LastMouseX = x;
	ui_LastMouseY = y;

#endif

	// Call AntTweakBar Event Function
	TwEventMouseMotionGLUT ( x, y );

	// Recall Display at Next Frame
	glutPostRedisplay ();
}

// Mouse Function
void MouseFunc ( int button, int state, int x, int y )

{

#ifndef QUAT_DEBUG

	// 
	// Customed Mouse Callback Function
	// 

	switch ( button )
	{
	
	case GLUT_LEFT_BUTTON:
		
		switch ( state )
		{
		case GLUT_DOWN:		
			IsDragRotEnable = true;		
			ui_LastMouseX = x; 
			ui_LastMouseY = y; 
			
			break;

		case GLUT_UP:		
			IsDragRotEnable = false;	
			
			break;
		}
		
		break;
	
	case GLUT_MIDDLE_BUTTON:
	case GLUT_RIGHT_BUTTON:		
		
		IsDragRotEnable = false; 
		break;
	
	}

#endif

	// Call AntTweakBar Event Function
	TwEventMouseButtonGLUT ( button, state, x, y );
	
	// Recall Display at Next Frame
	glutPostRedisplay ();

}
